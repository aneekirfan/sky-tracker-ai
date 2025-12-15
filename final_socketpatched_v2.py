import sys
import struct
import time
import math
import datetime
import numpy as np
import pandas as pd
import serial
from serial.tools import list_ports
import os

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QPushButton,
    QLabel, QVBoxLayout, QHBoxLayout, QFileDialog, QFrame,
    QComboBox, QTextEdit, QSizePolicy, QTabWidget, QDialog, QLineEdit, QMessageBox
)
from PyQt5.QtCore import Qt, QObject, QThread, pyqtSignal, QTimer, QPointF
from PyQt5.QtGui import QFont, QMatrix4x4, QVector3D, QPainter, QColor, QPen, QBrush

# pyqtgraph
try:
    import pyqtgraph.opengl as gl
    from pyqtgraph.opengl import MeshData
    _HAS_GL = True
except Exception:
    gl = None
    MeshData = None
    _HAS_GL = False

from drone3d_stable import Drone3DWidget
from ai_modules import KalmanFilter, AnomalyDetector, BatteryPredictor


# Allowed credentials (hardcoded)
ALLOWED_CREDENTIALS = [
    ("skyrobo", "eme1001"),
    ("skyrobo2", "drone21"),
    ("iqhfty", "962208")
]

# Secure cache location
CACHE_DIR = os.path.join(os.path.expanduser('~'), 'AppData', 'Roaming', 'SkyTracker')
CACHE_FILE = os.path.join(CACHE_DIR, 'login_cache.txt')

def check_login_cache():
    try:
        with open(CACHE_FILE, 'r') as f:
            date_str = f.read().strip()
            today = datetime.date.today().isoformat()
            return date_str == today
    except FileNotFoundError:
        return False

def mark_login():
    os.makedirs(CACHE_DIR, exist_ok=True)
    today = datetime.date.today().isoformat()
    with open(CACHE_FILE, 'w') as f:
        f.write(today)

# MSP commands
MSP_ATTITUDE = 108
MSP_ANALOG = 110
MSP_HEADING = 117

def build_msp_msg(cmd, payload=b''):
    header = b'$M<'
    size = len(payload).to_bytes(1, 'little')
    cmd_byte = cmd.to_bytes(1, 'little')
    checksum = (sum(size + cmd_byte + payload) & 0xFF).to_bytes(1, 'little')
    return header + size + cmd_byte + payload + checksum

def parse_msp_attitude(data):
    if len(data) < 6:
        return None, None, None
    try:
        roll, pitch, yaw = struct.unpack('<hhh', data[:6])
        return roll / 10.0, pitch / 10.0, (yaw / 10.0) % 360.0
    except Exception:
        return None, None, None

# ---------------- Serial worker with heading fallback ----------------
class SerialWorker(QObject):
    data_ready = pyqtSignal(dict)
    error = pyqtSignal(str)
    connected = pyqtSignal()
    disconnected = pyqtSignal()
    info = pyqtSignal(str)

    def __init__(self, port, baudrate=57600):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self._running = False
        self.recording = False
        self.min_stats = {'roll': None, 'pitch': None, 'yaw': None, 'voltage': None, 'current': None}
        self.max_stats = {'roll': None, 'pitch': None, 'yaw': None, 'voltage': None, 'current': None}

        # heading fallback state
        self._last_heading_ts = 0.0
        self._last_heading_val = None

        # throttle min/max updates
        self._minmax_update_counter = 0
        # ===== AI MODULES (Tier-1) =====
        self.roll_kf = KalmanFilter()
        self.pitch_kf = KalmanFilter()
        self.anomaly_detector = AnomalyDetector()
        self.battery_predictor = BatteryPredictor()


    def start_serial(self):
        try:
            self.info.emit(f"Opening serial {self.port} @ {self.baudrate}")
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            except Exception as e:
                self.error.emit(f"start_serial error: {e}")
                return

            time.sleep(2.0)
            self._running = True
            self.connected.emit()
            self.info.emit("Serial open — entering event loop")
            self.event_loop()
        except Exception as e:
            self.error.emit(f"start_serial error: {e}")

    def _parse_analog(self, payload_bytes):
        voltage = None
        current = None
        try:
            if len(payload_bytes) >= 1:
                vb = int(payload_bytes[0])
                if vb != 0:
                    voltage = vb / 10.0
            # prioritize common offset=2 for current, then others
            for off in (2, 3, 4, 5):
                if len(payload_bytes) >= off + 2:
                    val = struct.unpack_from('<H', bytes(payload_bytes), off)[0]
                    for div in (10.0, 100.0, 1000.0):
                        cand = val / div
                        if 0.0 < cand < 200.0:
                            current = cand
                            break
                    if current is not None:
                        break
        except Exception:
            pass
        return voltage, current

    def event_loop(self):
        while self._running:
            try:
                if not self.ser or not self.ser.is_open:
                    raise IOError("Serial port not open")

                # ATTITUDE
                self.ser.write(build_msp_msg(MSP_ATTITUDE))
                time.sleep(0.005)
                resp_att = self.ser.read(256)
                roll = pitch = yaw_att = None
                if resp_att.startswith(b'$M>') and len(resp_att) >= 6 and resp_att[4] == MSP_ATTITUDE:
                    size = resp_att[1]
                    payload = resp_att[5:5+size]
                    roll, pitch, yaw_att = parse_msp_attitude(payload)
                    # ----- AI: Kalman filtering -----
                    if roll is not None:
                        roll = self.roll_kf.update(roll)
                    if pitch is not None:
                        pitch = self.pitch_kf.update(pitch)

                # HEADING (try to get continuous 0..360)
                heading = None
                try:
                    self.ser.write(build_msp_msg(MSP_HEADING))
                    time.sleep(0.005)
                    resp_head = self.ser.read(128)
                    if resp_head.startswith(b'$M>') and len(resp_head) >= 7:
                        # payload starts at index 5 typically
                        size = resp_head[1]
                        p = resp_head[5:5+size]
                        if len(p) >= 2:
                            try:
                                raw = struct.unpack('<h', p[:2])[0]
                                heading = float(raw) % 360.0
                            except Exception:
                                try:
                                    raw_u = struct.unpack('<H', p[:2])[0]
                                    heading = float(raw_u) % 360.0
                                except Exception:
                                    heading = None
                    # update last-heading timestamp when valid
                    if heading is not None:
                        self._last_heading_ts = time.time()
                        self._last_heading_val = heading
                except Exception:
                    heading = None

                # ANALOG
                self.ser.write(build_msp_msg(MSP_ANALOG))
                time.sleep(0.005)
                resp_analog = self.ser.read(256)
                voltage = current = None
                if resp_analog.startswith(b'$M>') and len(resp_analog) >= 7:
                    size = resp_analog[1]
                    payload = resp_analog[5:5+size]
                    voltage, current = self._parse_analog(list(payload))

                # decide yaw_out: prefer recent heading, else fall back to attitude-yaw
                yaw_out = None
                now = time.time()
                if (self._last_heading_val is not None) and (now - self._last_heading_ts <= 1.0):
                    yaw_out = self._last_heading_val
                else:
                    # heading stale — fall back to attitude yaw if available
                    yaw_out = yaw_att
                # ----- AI: Anomaly detection -----
                anomaly = False
                if None not in (roll, pitch, yaw_out, voltage, current):
                    anomaly = self.anomaly_detector.update(
                        [roll, pitch, yaw_out, voltage, current]
                    )
                # ----- AI: Battery time estimation -----
                battery_time = self.battery_predictor.update(voltage)
                # update recording min/max
                if self.recording:
                    update_values = {'roll': roll, 'pitch': pitch, 'yaw': yaw_out, 'voltage': voltage, 'current': current}
                    changed = False
                    for k, v in update_values.items():
                        if v is None:
                            continue
                        if self.min_stats[k] is None or v < self.min_stats[k]:
                            self.min_stats[k] = v
                            changed = True
                        if self.max_stats[k] is None or v > self.max_stats[k]:
                            self.max_stats[k] = v
                            changed = True
                    if changed:
                        self._minmax_update_counter += 1
                        if self._minmax_update_counter % 10 == 0:  # throttle to every 10 updates
                            self.info.emit("Min/Max updated")

                # emit data packet
                self.data_ready.emit({
                    'roll': roll, 'pitch': pitch, 'yaw': yaw_out,
                    'voltage': voltage, 'current': current,
                    'anomaly': anomaly,
                    'battery_time': battery_time,
                    'min_stats': self.min_stats.copy(),
                    'max_stats': self.max_stats.copy(),
                    'recording': self.recording
                })

                # small responsive sleep
                for _ in range(6):
                    if not self._running:
                        break
                    time.sleep(0.01)

            except Exception as e:
                self.error.emit(f"Serial loop error: {e}")
                self._running = False
                break

        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.disconnected.emit()
        self.info.emit("Serial worker stopped")

    def stop(self):
        self._running = False

    def start_recording(self):
        self.recording = True
        self.min_stats = {'roll': None, 'pitch': None, 'yaw': None, 'voltage': None, 'current': None}
        self.max_stats = {'roll': None, 'pitch': None, 'yaw': None, 'voltage': None, 'current': None}
        self._minmax_update_counter = 0
        self.info.emit("Recording started")

    def stop_recording(self):
        self.recording = False
        self.info.emit("Recording stopped")



# ---------------- Attitude gauge (unchanged from previous correct behavior) ----------------
class AttitudeGauge(QWidget):
    def __init__(self, label="Roll", mode='roll', parent=None):
        super().__init__(parent)
        self.label = label
        self.mode = mode
        self.value = 0.0
        self.setMinimumSize(200, 200)
        self.base_font = QFont("Segoe UI", 9)
        self.label_font = QFont("Segoe UI", 11, QFont.Bold)

    def setValue(self, v):
        self.value = 0.0 if v is None else v
        self.update()

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w/2.0, h/2.0
        r = min(cx, cy) - 12

        # background circle
        p.setBrush(QBrush(QColor(30, 30, 40)))
        p.setPen(Qt.NoPen)
        p.drawEllipse(int(cx-r), int(cy-r), int(2*r), int(2*r))

        p.setPen(QPen(QColor(200, 200, 200), 1))
        p.setFont(self.base_font)

        if self.mode == 'roll':
            tick_vals = [-90, -60, -30, 0, 30, 60, 90]
            for tv in tick_vals:
                screen_deg = -90 + tv
                a = math.radians(screen_deg)
                x1 = cx + math.cos(a) * (r * 0.82)
                y1 = cy + math.sin(a) * (r * 0.82)
                x2 = cx + math.cos(a) * (r * 0.92)
                y2 = cy + math.sin(a) * (r * 0.92)
                p.drawLine(QPointF(x1, y1), QPointF(x2, y2))
                lbl = f"{int(tv)}" if tv != 0 else "0"
                fx = cx + math.cos(a) * (r * 0.66)
                fy = cy + math.sin(a) * (r * 0.66) + 4
                p.setFont(self.base_font)
                p.drawText(int(fx - 10), int(fy - 6), 40, 16, Qt.AlignCenter, lbl)
            ang_rad = math.radians(-90 + max(-180.0, min(180.0, self.value)))
        else:
            tick_vals = [90, 60, 30, 0, -30, -60, -90]
            for tv in tick_vals:
                screen_deg = 180 - tv
                a = math.radians(screen_deg)
                x1 = cx + math.cos(a) * (r * 0.82)
                y1 = cy + math.sin(a) * (r * 0.82)
                x2 = cx + math.cos(a) * (r * 0.92)
                y2 = cy + math.sin(a) * (r * 0.92)
                p.drawLine(QPointF(x1, y1), QPointF(x2, y2))
                lbl = f"{int(tv)}" if tv != 0 else "0"
                fx = cx + math.cos(a) * (r * 0.58)
                fy = cy + math.sin(a) * (r * 0.58) + 4
                p.setFont(self.base_font)
                p.drawText(int(fx - 12), int(fy - 6), 44, 16, Qt.AlignCenter, lbl)
            ang_rad = math.radians(180 - max(-180.0, min(180.0, self.value)))

        p.setPen(QPen(QColor(255, 200, 60), 3))
        nx = cx + math.cos(ang_rad) * (r * 0.70)
        ny = cy + math.sin(ang_rad) * (r * 0.70)
        p.drawLine(QPointF(cx, cy), QPointF(nx, ny))

        p.setBrush(QBrush(QColor(230, 230, 230)))
        p.setPen(Qt.NoPen)
        p.drawEllipse(int(cx-6), int(cy-6), 12, 12)

        p.setPen(QPen(QColor(200, 200, 200)))
        p.setFont(self.label_font)
        p.drawText(int(cx-50), int(cy + r + 18), 100, 24, Qt.AlignCenter, f"{self.label}: {self.value:5.1f}°")
        p.end()

# ---------------- Login Dialog ----------------
class LoginDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Login")
        self.setFixedSize(400, 250)
        self.setStyleSheet("""
            QDialog { background-color: #222436; }
            QLabel { color: #ffd700; font-size: 14px; }
            QLineEdit { background-color: #3b3e5e; color: #ffd700; border-radius: 8px; padding: 6px; font-size: 14px; }
            QPushButton { background-color: #686de0; color: #fff; border-radius: 8px; padding: 8px 16px; font-weight: 700; }
            QPushButton:hover { background-color: #5a5fc0; }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(15)

        title = QLabel("Sky Tracker Login")
        title.setFont(QFont("Segoe UI", 18, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        user_layout = QHBoxLayout()
        user_label = QLabel("User ID:")
        user_label.setFixedWidth(80)
        self.user_edit = QLineEdit()
        user_layout.addWidget(user_label)
        user_layout.addWidget(self.user_edit)
        layout.addLayout(user_layout)

        pass_layout = QHBoxLayout()
        pass_label = QLabel("Password:")
        pass_label.setFixedWidth(80)
        self.pass_edit = QLineEdit()
        self.pass_edit.setEchoMode(QLineEdit.Password)
        pass_layout.addWidget(pass_label)
        pass_layout.addWidget(self.pass_edit)
        layout.addLayout(pass_layout)

        self.login_btn = QPushButton("Login")
        self.login_btn.clicked.connect(self.attempt_login)
        layout.addWidget(self.login_btn, alignment=Qt.AlignCenter)

    def attempt_login(self):
        user = self.user_edit.text().strip()
        pwd = self.pass_edit.text().strip()
        if (user, pwd) in ALLOWED_CREDENTIALS:
            self.accept()
        else:
            QMessageBox.warning(self, "Login Failed", "Invalid user ID or password.")

# ---------------- Main UI (keeps layout and logic you had) ----------------
class FlightStatsApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sky Tracker v1.01")
        self.setMinimumSize(1300, 820)
        self.setStyleSheet("""
            QMainWindow, QWidget { background-color: #222436; }
            QFrame#panel { background: #2c2f4a; border-radius:12px; }
            QLabel.stat { background:#3b3e5e; color:#ffd700; border-radius:8px; padding:10px; font-weight:600; }
            QPushButton { background:#3b3e5e; color:#ffd700; border-radius:8px; padding:8px 12px; }
            QPushButton#primary { background:#686de0; color:#fff; font-weight:700; }
            QComboBox { background:#3b3e5e; color:#ffd700; border-radius:8px; padding:6px; }
            QTextEdit#console { background:#0f1112; color:#bfead5; border-radius:8px; padding:8px; font-family: Consolas, monospace; }
        """)

        # labels
        self.labels = {}
        for k in ['Roll','Pitch','Yaw','Voltage','Current','MinMax']:
            lbl = QLabel("0.0" if k != 'MinMax' else "Min-Max: Not Recording")
            lbl.setObjectName('stat')
            lbl.setProperty('class', 'stat')
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFont(QFont("Segoe UI", 14, QFont.Bold))
            self.labels[k.lower()] = lbl

        # controls
        self.port_selector = QComboBox(); self.port_selector.setFont(QFont("Segoe UI", 14))
        self.refresh_btn = QPushButton("Refresh Ports"); self.refresh_btn.setFont(QFont("Segoe UI", 13))
        self.connect_btn = QPushButton("Connect"); self.connect_btn.setObjectName("primary"); self.connect_btn.setFont(QFont("Segoe UI", 13))
        self.start_btn = QPushButton("Start Recording"); self.stop_btn = QPushButton("Stop Recording")
        self.save_btn = QPushButton("Save to Excel"); self.disconnect_btn = QPushButton("Disconnect")
        self.reset_btn = QPushButton("Reset Drone")

        # console
        self.console = QTextEdit(); self.console.setObjectName("console")
        self.console.setReadOnly(True)
        self.console.setFixedHeight(160)
        self.console.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        # 3D view + gauges
        self.drone_view = Drone3DWidget()
        self.roll_gauge = AttitudeGauge("Roll", mode='roll') if _HAS_GL else None
        self.pitch_gauge = AttitudeGauge("Pitch", mode='pitch') if _HAS_GL else None
        self.yaw_value_label = QLabel("Yaw: --")
        self.yaw_value_label.setFont(QFont("Segoe UI", 14, QFont.Bold))
        self.yaw_value_label.setStyleSheet("color:#ffd700;")

        self.thread = None
        self.worker = None
        self.last_minmax_snapshot = (None, None)

        # Orientation offsets for reset
        self.roll_offset = 0.0
        self.pitch_offset = 0.0
        self.yaw_offset = 0.0
        self.last_roll = None
        self.last_pitch = None
        self.last_yaw = None

        # connect signals
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.start_worker)
        self.disconnect_btn.clicked.connect(self.stop_worker)
        self.start_btn.clicked.connect(self.start_recording)
        self.stop_btn.clicked.connect(self.stop_recording)
        self.save_btn.clicked.connect(self.save_stats)
        self.reset_btn.clicked.connect(self.reset_drone)

        self.setup_ui()
        self.refresh_ports()
        self.log("App started")

    def setup_ui(self):
        main = QWidget()
        root = QHBoxLayout(main)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        left_panel = QFrame(); left_panel.setObjectName("panel")
        left_layout = QVBoxLayout(left_panel); left_layout.setContentsMargins(12, 12, 12, 12); left_layout.setSpacing(8)

        port_row = QHBoxLayout(); port_row.addWidget(self.port_selector, 1); port_row.addWidget(self.refresh_btn)
        left_layout.addLayout(port_row)

        stat_block = QFrame(); stat_block.setObjectName("panel")
        stat_layout = QVBoxLayout(stat_block); stat_layout.setSpacing(6)
        for key in ['roll','pitch','yaw','voltage','current']:
            stat_layout.addWidget(self.labels[key])
        left_layout.addWidget(stat_block)
        left_layout.addWidget(self.labels['minmax'])

        btn_row = QHBoxLayout()
        for b in [self.connect_btn, self.start_btn, self.stop_btn, self.save_btn, self.disconnect_btn]:
            btn_row.addWidget(b)
        left_layout.addLayout(btn_row)

        left_layout.addWidget(self.console)
        root.addWidget(left_panel, 1)

        mid_right = QHBoxLayout()

        # Drone container with reset button above
        drone_container = QWidget()
        drone_layout = QVBoxLayout(drone_container)
        drone_layout.setContentsMargins(0, 0, 0, 0)
        drone_layout.setSpacing(8)

        # Reset button at top right
        reset_row = QHBoxLayout()
        reset_row.addStretch()
        reset_row.addWidget(self.reset_btn)
        drone_layout.addLayout(reset_row)

        # Drone view below
        drone_layout.addWidget(self.drone_view)

        mid_right.addWidget(drone_container, 2)

        tabs = QTabWidget()
        # Gauges tab
        gauges_tab = QWidget()
        g_layout = QVBoxLayout(gauges_tab)
        if _HAS_GL:
            g_inner = QHBoxLayout()
            g_inner.addWidget(self.roll_gauge)
            g_inner.addWidget(self.pitch_gauge)
            g_layout.addLayout(g_inner)
            g_layout.addWidget(self.yaw_value_label, alignment=Qt.AlignCenter)
        else:
            g_layout.addWidget(QLabel("Gauges require pyqtgraph + PyOpenGL"))
        tabs.addTab(gauges_tab, "Gauges")

        # Info tab
        info_tab = QWidget()
        info_layout = QVBoxLayout(info_tab)
        card = QFrame(); card.setObjectName("panel")
        card.setStyleSheet("background: #2c2f4a; border-radius:12px; padding:14px;")
        card_layout = QVBoxLayout(card); card_layout.setSpacing(8)
        name_lbl = QLabel("Sky Robo Drones Pvt. Ltd.")
        name_lbl.setFont(QFont("Segoe UI", 18, QFont.Bold))
        name_lbl.setStyleSheet("color:#ffd700;")
        email_lbl = QLabel("Email: skyrobodrones@gmail.com")
        phone_lbl = QLabel("Phone: +91 60065 68924")
        email_lbl.setStyleSheet("color:#ffffff; font-size:20px;")
        phone_lbl.setStyleSheet("color:#ffffff; font-size:20px;")
        # improved warning: high-contrast, wrapped, padded
        warning_lbl = QLabel("⚠ Unauthorized distribution of this software is strictly prohibited and will attract legal action.")
        warning_lbl.setWordWrap(False)
        warning_lbl.setStyleSheet(
            "background-color:#3b0000; color:#ffd7d7; font-weight:350; font-size:10px;"
            "padding:17px; border:2px solid #ff4040; border-radius: 16px;"
        )
        card_layout.addWidget(name_lbl, 0, Qt.AlignHCenter)
        card_layout.addWidget(email_lbl, 0, Qt.AlignHCenter)
        card_layout.addWidget(phone_lbl, 0, Qt.AlignHCenter)
        card_layout.addSpacing(8)
        card_layout.addWidget(warning_lbl, 0, Qt.AlignHCenter)
        info_layout.addWidget(card, 0, Qt.AlignTop)
        tabs.addTab(info_tab, "Info")

        tabs.setMinimumWidth(320)
        mid_right.addWidget(tabs, 1)

        root.addLayout(mid_right, 3)
        self.setCentralWidget(main)

    def log(self, msg):
        now = datetime.datetime.now().strftime("%H:%M:%S")
        self.console.append(f"[{now}] {msg}")

    def is_fc_port(self, port):
        keywords = ['CP210', 'FTDI', 'Silicon Labs', 'STMicroelectronics', 'Arduino', 'USB Serial', 'SiLabs', 'UART']
        desc = (port.description or '').lower(); manuf = (port.manufacturer or '').lower()
        vid_pid = (str(port.vid or '') + ':' + str(port.pid or '')).lower()
        for kw in keywords:
            if kw.lower() in desc or kw.lower() in manuf:
                return True
        if vid_pid in ['10c4:ea60', '0403:6001', '0483:5740']:
            return True
        return False

    def refresh_ports(self):
        self.port_selector.blockSignals(True)
        self.port_selector.clear()
        ports = list_ports.comports()
        filtered = [p for p in ports if self.is_fc_port(p)]
        if filtered:
            for p in filtered:
                self.port_selector.addItem(f"{p.device}", p.device)
        else:
            if ports:
                for p in ports:
                    self.port_selector.addItem(f"{p.device}", p.device)
            else:
                self.port_selector.addItem("No ports detected", None)
        self.port_selector.blockSignals(False)
        self.log("Ports refreshed")

    def start_worker(self):
        port = self.port_selector.currentData()
        if port is None:
            self.statusBar().showMessage("No valid port selected")
            self.log("Connect attempted with no port")
            return
        if self.thread and self.thread.isRunning():
            self.statusBar().showMessage("Already connected")
            self.log("Connect attempted while already connected")
            return

        # start thread + worker
        self.thread = QThread()
        self.worker = SerialWorker(port, 57600)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.start_serial)
        self.worker.data_ready.connect(self.update_ui)
        self.worker.error.connect(lambda e: (self.statusBar().showMessage(f"Error: {e}"), self.log(f"ERROR: {e}")))
        self.worker.info.connect(lambda t: self.log(t))
        self.worker.connected.connect(lambda: (self.statusBar().showMessage(f"Connected on {port}"), self.log(f"Connected on {port}")))
        self.worker.disconnected.connect(lambda: (self.statusBar().showMessage("Disconnected"), self.log("Disconnected")))
        self.thread.start()
        self.log(f"Worker thread started for {port}")

    def stop_worker(self):
        self.log("Stopping worker...")
        if self.worker:
            try:
                self.worker.stop()
            except Exception as e:
                self.log(f"Error calling stop on worker: {e}")
        if self.thread:
            try:
                self.thread.quit()
                self.thread.wait(1500)
            except Exception as e:
                self.log(f"Thread quit/wait error: {e}")
        self.worker = None
        self.thread = None
        self.log("Worker and thread references cleared")

    def _format_minmax(self, min_s, max_s):
        def fm(v, fmt="{:5.1f}"):
            return "--" if v is None else fmt.format(v)
        return (f"Min → Roll: {fm(min_s.get('roll'))}° | Pitch: {fm(min_s.get('pitch'))}° | "
                f"Yaw: {fm(min_s.get('yaw'))}° | V: {fm(min_s.get('voltage'))} V | I: {fm(min_s.get('current') / 1000 if min_s.get('current') is not None else None)} A\n"
                f"Max → Roll: {fm(max_s.get('roll'))}° | Pitch: {fm(max_s.get('pitch'))}° | "
                f"Yaw: {fm(max_s.get('yaw'))}° | V: {fm(max_s.get('voltage'))} V | I: {fm(max_s.get('current') / 1000 if max_s.get('current') is not None else None)} A")

    def update_ui(self, data):
        # ----- AI feedback -----
        if data.get('anomaly'):
            self.statusBar().showMessage("⚠ Flight anomaly detected")
            self.log("AI anomaly detected in flight telemetry")

        bt = data.get('battery_time')
        if bt is not None:
            self.statusBar().showMessage(f"Estimated battery time: {bt:.1f} min")
        def safe_set(lbl, txt):
            if lbl.text() != txt:
                lbl.setText(txt)

        r, p, y = data['roll'], data['pitch'], data['yaw']
        v, c = data['voltage'], data['current']

        # Store last values for reset
        self.last_roll = r
        self.last_pitch = p
        self.last_yaw = y

        # Compute relative values
        rel_r = (r - self.roll_offset) if r is not None else None
        rel_p = (p - self.pitch_offset) if p is not None else None
        rel_y = (y - self.yaw_offset) if y is not None else None

        if rel_r is not None:
            safe_set(self.labels['roll'], f"Roll: {rel_r:.1f}°")
        if rel_p is not None:
            safe_set(self.labels['pitch'], f"Pitch: {rel_p:.1f}°")
        if rel_y is not None:
            safe_set(self.labels['yaw'], f"Yaw: {rel_y:.1f}°")

        safe_set(self.labels['voltage'], f"Voltage: {v:.2f} V" if v is not None else "Voltage: --")
        safe_set(self.labels['current'], f"Current: {c/1000:.2f} A" if c is not None else "Current: --")

        min_s = data.get('min_stats', {})
        max_s = data.get('max_stats', {})
        if (min_s, max_s) != self.last_minmax_snapshot:
            pretty = self._format_minmax(min_s, max_s)
            safe_set(self.labels['minmax'], pretty)
            self.log("MinMax changed")
            self.last_minmax_snapshot = (min_s.copy(), max_s.copy())

        # update 3D model with offsets (relative orientation) - only set targets for smoothing
        try:
            if r is not None and p is not None:
                rel_r = r - self.roll_offset
                rel_p = p - self.pitch_offset
                rel_y = (y - self.yaw_offset) if y is not None else 0.0
                # Set targets for worker thread to handle smoothing
                self.drone_view.worker.set_target_orientation(rel_r, rel_p)
                # Note: yaw is kept in state but not used for visuals
        except Exception as e:
            self.log(f"3D update error: {e}")

        if _HAS_GL and self.roll_gauge and self.pitch_gauge:
            try:
                self.roll_gauge.setValue(rel_r if rel_r is not None else 0.0)
                self.pitch_gauge.setValue(rel_p if rel_p is not None else 0.0)
                if rel_y is not None:
                    self.yaw_value_label.setText(f"Yaw: {rel_y:.1f}°")
                else:
                    self.yaw_value_label.setText("Yaw: --")
            except Exception as e:
                self.log(f"Gauge update error: {e}")

    def start_recording(self):
        if self.worker:
            try:
                self.worker.start_recording()
                self.labels['minmax'].setText("Recording...")
                self.log("Recording requested")
            except Exception as e:
                self.log(f"start_recording error: {e}")

    def stop_recording(self):
        if self.worker:
            try:
                self.worker.stop_recording()
                self.labels['minmax'].setText("Stopped recording.")
                self.log("Recording stopped by user")
            except Exception as e:
                self.log(f"stop_recording error: {e}")

    def save_stats(self):
        if not self.last_minmax_snapshot or self.last_minmax_snapshot[0] is None:
            self.statusBar().showMessage("No data to save.")
            self.log("Save attempted with no data")
            return
        fn, _ = QFileDialog.getSaveFileName(self, "Save Stats", "", "Excel Files (*.xlsx)")
        if fn:
            min_s, max_s = self.last_minmax_snapshot
            try:
                df = pd.DataFrame([min_s, max_s], index=['Min', 'Max'])
                with pd.ExcelWriter(fn) as w:
                    df.to_excel(w, sheet_name='MinMax')
                self.statusBar().showMessage("Saved.")
                self.log(f"Saved MinMax to {fn}")
            except Exception as e:
                self.statusBar().showMessage(f"Save failed: {e}")
                self.log(f"Save failed: {e}")

    def reset_drone(self):
        self.roll_offset = self.last_roll if self.last_roll is not None else 0.0
        self.pitch_offset = self.last_pitch if self.last_pitch is not None else 0.0
        self.yaw_offset = self.last_yaw if self.last_yaw is not None else 0.0
        self.log("Drone orientation reset to current position")

    def closeEvent(self, ev):
        self.log("Application closing...")
        self.stop_worker()
        ev.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    if not check_login_cache():
        login_dialog = LoginDialog()
        if login_dialog.exec_() == QDialog.Accepted:
            mark_login()
        else:
            sys.exit(0)
    wnd = FlightStatsApp()
    wnd.show()
    sys.exit(app.exec_())
