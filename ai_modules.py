import time
import numpy as np
from collections import deque
from sklearn.ensemble import IsolationForest

class KalmanFilter:
    def __init__(self, q=0.02, r=0.6):
        self.q = q
        self.r = r
        self.x = 0.0
        self.p = 1.0

    def update(self, z):
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x += k * (z - self.x)
        self.p *= (1 - k)
        return self.x


class AnomalyDetector:
    def __init__(self, window=120):
        self.buf = deque(maxlen=window)
        self.model = IsolationForest(contamination=0.05, random_state=42)
        self.ready = False

    def update(self, vec):
        self.buf.append(vec)
        if len(self.buf) >= 60 and not self.ready:
            self.model.fit(np.array(self.buf))
            self.ready = True
        if self.ready:
            return self.model.predict([vec])[0] == -1
        return False


class BatteryPredictor:
    def __init__(self):
        self.hist = deque(maxlen=300)
        self.t0 = time.time()

    def update(self, voltage):
        if voltage is None:
            return None
        t = time.time() - self.t0
        self.hist.append((t, voltage))
        if len(self.hist) < 30:
            return None
        t0, v0 = self.hist[0]
        t1, v1 = self.hist[-1]
        dv_dt = (v0 - v1) / (t1 - t0 + 1e-6)
        if dv_dt <= 0:
            return None
        return max(0.0, (v1 / dv_dt) / 60.0)
 