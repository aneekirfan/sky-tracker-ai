# drone3d_stable.py
# Stable 3D drone widget for use in PyQt5 + pyqtgraph applications.
# - Front cone attached to nose (follows pitch & roll, ignores yaw)
# - Thicker cross rods
# - Horizontal cylindrical central frame
# - Thicker colored static propellers (red/green)
# - Lightweight horizon grid (GLGridItem) below the drone
# - Reuses transforms for perfect synchronization

import math
import numpy as np
from PyQt5.QtCore import QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QMatrix4x4, QVector3D
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import MeshData

# ---------- mesh builders ----------
def make_cylinder(length=12.0, radius=1.2, slices=24, color=(0.12, 0.12, 0.12, 1.0)):
    """
    Cylinder oriented along X axis (centered at origin, length in X).
    We'll build as two rings and side faces for a solid bar.
    """
    # create two rings of vertices (front and back)
    angles = np.linspace(0, 2 * math.pi, slices, endpoint=False)
    front_x = length / 2.0
    back_x = -length / 2.0
    ring_front = np.column_stack([np.full(slices, front_x), np.cos(angles) * radius, np.sin(angles) * radius])
    ring_back = np.column_stack([np.full(slices, back_x), np.cos(angles) * radius, np.sin(angles) * radius])
    verts = np.vstack([ring_front, ring_back])
    faces = []
    for i in range(slices):
        a = i
        b = (i + 1) % slices
        # triangle pair for quad
        faces.append([a, b, slices + b])
        faces.append([a, slices + b, slices + a])
    md = MeshData(vertexes=verts, faces=np.array(faces, dtype=int))
    return gl.GLMeshItem(meshdata=md, smooth=True, shader='normalColor', color=color, drawEdges=False)

def make_propeller(radius=3.2, thickness=0.6, slices=20, color=(0.8, 0.1, 0.1, 0.9)):
    """
    Thick disc-like propeller oriented so its center sits on local origin.
    We'll create a thin cylinder (two rings) to represent the propeller visually.
    """
    angles = np.linspace(0, 2 * math.pi, slices, endpoint=False)
    half_t = thickness / 2.0
    # create top and bottom ring in local YZ plane, with X offset to center thickness around X axis
    top = np.column_stack([np.full(slices, half_t), np.cos(angles) * radius, np.sin(angles) * radius])
    bottom = np.column_stack([np.full(slices, -half_t), np.cos(angles) * radius, np.sin(angles) * radius])
    verts = np.vstack([top, bottom])
    faces = []
    for i in range(slices):
        a = i
        b = (i + 1) % slices
        faces.append([a, b, slices + b])
        faces.append([a, slices + b, slices + a])
    md = MeshData(vertexes=verts, faces=np.array(faces, dtype=int))
    return gl.GLMeshItem(meshdata=md, smooth=True, shader='normalColor', color=color, drawEdges=False)

def make_cone(radius=0.45, height=2.8, slices=18, color=(1.0, 0.94, 0.04, 0.95)):
    """
    Thin cone oriented along +X (tip at +X). Used as the front indicator (yellow).
    """
    angles = np.linspace(0, 2 * math.pi, slices, endpoint=False)
    tip = np.array([[height / 2.0, 0.0, 0.0]])
    base = np.column_stack([np.full(slices, -height / 2.0), np.cos(angles) * radius, np.sin(angles) * radius])
    verts = np.vstack([tip, base])
    faces = []
    for i in range(slices):
        faces.append([0, 1 + i, 1 + ((i + 1) % slices)])
    md = MeshData(vertexes=verts, faces=np.array(faces, dtype=int))
    return gl.GLMeshItem(meshdata=md, smooth=True, shader='normalColor', color=color, drawEdges=False)

# ---------- OrientationWorker ----------
class OrientationWorker(QThread):
    """
    Worker thread to compute interpolated orientations for smoother animation.
    Emits update signals to main thread for GL updates.
    """
    update_signal = pyqtSignal(float, float)  # roll, pitch

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = True
        self._target_roll = 0.0
        self._target_pitch = 0.0
        self._current_roll = 0.0
        self._current_pitch = 0.0
        self._alpha = 0.4  # smoothing factor for exponential smoothing

    def set_target_orientation(self, roll, pitch):
        self._target_roll = roll
        self._target_pitch = pitch

    def run(self):
        while self._running:
            # Exponential smoothing towards target
            self._current_roll += (self._target_roll - self._current_roll) * self._alpha
            self._current_pitch += (self._target_pitch - self._current_pitch) * self._alpha
            self.update_signal.emit(self._current_roll, self._current_pitch)
            self.msleep(16)  # ~60 Hz

    def stop(self):
        self._running = False

# ---------- Drone3DWidget ----------
class Drone3DWidget(gl.GLViewWidget):
    """
    Stable drone widget:
    - responds only to roll & pitch (yaw ignored)
    - central cylinder frame, thick cross rods, colored static propellers
    - attached yellow cone for nose indicator
    - horizon grid below the drone
    - multithreaded for smoother animation
    """

    def __init__(self):
        super().__init__()
        self.setCameraPosition(distance=85, elevation=18, azimuth=45)
        self.opts['center'] = QVector3D(0, 0, 0)

        # Create central horizontal cylinder (frame)
        self.body = make_cylinder(length=16.0, radius=1.2, slices=24, color=(0.72, 0.72, 0.72, 1.0))
        self.body.setGLOptions('opaque') 
        self.addItem(self.body)

        # Prop hub positions relative to body (X forward, Y right, Z up)
        arm = 14.0
        self.prop_positions = [
            np.array([ arm,  arm, 0.0]),  # front-right
            np.array([-arm,  arm, 0.0]),  # front-left
            np.array([-arm, -arm, 0.0]),  # back-left
            np.array([ arm, -arm, 0.0]),  # back-right
        ]

        # Cross rods (thicker)
        self.cross_rods = []
        rod_color = (0.72, 0.72, 0.72, 1.0)
        for pos in self.prop_positions:
            rod = gl.GLLinePlotItem(pos=np.array([[0.0, 0.0, 0.0], pos]),
                                    width=10.0, antialias=True, color=rod_color)
            self.cross_rods.append(rod)
            self.addItem(rod)

        # Propellers (thicker, colored pairs: front red, left green pattern)
        colors = [(0.8, 0.1, 0.1, 0.95), (0.1, 0.8, 0.1, 0.95),
                  (0.1, 0.8, 0.1, 0.95), (0.8, 0.1, 0.1, 0.95)]
        self.props = []
        for c in colors:
            p = make_propeller(radius=3.2, thickness=0.7, slices=20, color=c)
            self.props.append(p)
            self.addItem(p)

        # Front cone (yellow) attached to nose — will follow pitch & roll
        self.cone = make_cone(radius=0.45, height=3.0, slices=18, color=(1.0, 0.94, 0.05, 0.98))
        self.addItem(self.cone)

        # Horizon grid (lightweight)
        self.horizon = gl.GLGridItem()
        # Make grid subtle and large
        self.horizon.setSize(x=100, y=100, z=0)
        self.horizon.setSpacing(x=10, y=10, z=1)
        # Position horizon a bit below the drone (negative Z)
        self.horizon.translate(0, 0, -12)
        self.addItem(self.horizon)

        # state (we ignore yaw)
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0  # kept but unused for visuals

        # Worker thread for smooth interpolation
        self.worker = OrientationWorker()
        self.worker.update_signal.connect(self._on_worker_update)
        self.worker.start()

    def _on_worker_update(self, roll, pitch):
        """
        Slot to handle updates from the worker thread.
        """
        self.update_orientation(roll=roll, pitch=pitch, yaw=None)

    def _tick(self):
        # reapply last orientation to ensure transforms stay in sync
        self.update_orientation(None, None, None)

    def _build_matrix(self, roll, pitch):
        """
        Build QMatrix4x4 applying pitch (Y axis) then roll (X axis).
        Yaw intentionally omitted (visuals ignore yaw).
        Rotation order: pitch then roll for natural airplane-like behavior.
        """
        m = QMatrix4x4()
        # pitch about Y, then roll about X
        m.rotate(pitch, 0.0, 1.0, 0.0)
        m.rotate(roll, 1.0, 0.0, 0.0)
        return m

    def update_orientation(self, roll=None, pitch=None, yaw=None):
        """
        Update stored attitude and reapply transforms.
        Yaw parameter is accepted but ignored for visuals by design.
        """
        if roll is not None:
            self._roll = roll
        if pitch is not None:
            self._pitch = pitch
        if yaw is not None:
            # keep yaw in state but do not use it to rotate visuals
            self._yaw = yaw

        # Cache the matrix to avoid redundant computations
        if not hasattr(self, '_cached_matrix') or self._cached_roll != self._roll or self._cached_pitch != self._pitch:
            self._cached_matrix = self._build_matrix(self._roll or 0.0, self._pitch or 0.0)
            self._cached_roll = self._roll
            self._cached_pitch = self._pitch

        m = self._cached_matrix

        # Reset & apply same transforms to body and cone (no yaw)
        for item in (self.body, self.cone):
            item.resetTransform()
            # apply pitch then roll (same order as matrix)
            item.rotate(self._pitch or 0.0, 0.0, 1.0, 0.0)  # rotate about Y (pitch)
            item.rotate(self._roll or 0.0, 1.0, 0.0, 0.0)   # rotate about X (roll)

        # translate cone forward in local +X (after rotations have been applied)
        # chosen offset keeps cone snug near nose of cylinder
        self.cone.resetTransform()
        self.cone.rotate(self._pitch or 0.0, 0.0, 1.0, 0.0)
        self.cone.rotate(self._roll or 0.0, 1.0, 0.0, 0.0)
        self.cone.translate(9.0, 0.0, 0.0)

        # update cross rods endpoints and prop positions using the same matrix
        for i, pos in enumerate(self.prop_positions):
            hub_local = QVector3D(*pos)
            hub_world = m.map(hub_local)
            # update rod end position
            self.cross_rods[i].setData(pos=np.array([[0.0, 0.0, 0.0], [hub_world.x(), hub_world.y(), hub_world.z()]]))
            # place propeller at hub_world (static orientation relative to world)
            self.props[i].resetTransform()
            self.props[i].translate(hub_world.x(), hub_world.y(), hub_world.z())

        # also re-apply body transform via reset + rotate to ensure no drift
        self.body.resetTransform()
        self.body.rotate(self._pitch or 0.0, 0.0, 1.0, 0.0)
        self.body.rotate(self._roll or 0.0, 1.0, 0.0, 0.0)

    # convenience setters
    def set_roll(self, r):
        self.worker.set_target_orientation(r, self._pitch)

    def set_pitch(self, p):
        self.worker.set_target_orientation(self._roll, p)

    def set_yaw(self, y):
        # keep yaw in state but it won't visually rotate the model
        self.update_orientation(roll=None, pitch=None, yaw=y)
