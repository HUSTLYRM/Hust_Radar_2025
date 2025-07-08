import numpy as np
from collections import deque, Counter
from typing import List, Tuple, Optional, Dict, Set

###############################################################################
# CONFIGS
###############################################################################
_MAX_ID_HISTORY = 20          # keep last N external‑ids per track
_DIST_THRESH    = 15.0        # max Euclidean distance (m) to consider match

###############################################################################
# 2‑D CONSTANT‑VELOCITY KALMAN FILTER (unchanged core)
###############################################################################
class KalmanFilter2DConstVel:
    """Planar constant‑velocity KF: state [x,y,vx,vy], meas [x,y]"""

    def __init__(self, xy_init: np.ndarray, q: float = 1.0):
        self.x = xy_init.reshape(4, 1)
        self.P = np.diag([10, 10, 10, 10]).astype(float)
        self.R = np.diag([1, 1]).astype(float)
        self.q = q
        self._F_cache: Optional[Tuple[float, np.ndarray]] = None
        self._Q_cache: Optional[Tuple[float, np.ndarray]] = None

    def _F(self, dt: float) -> np.ndarray:
        F = np.eye(4)
        F[0, 2] = dt
        F[1, 3] = dt
        return F

    def _Q(self, dt: float) -> np.ndarray:
        dt2, dt3, dt4 = dt**2, dt**3, dt**4
        q11, q12, q22 = dt4/4*self.q, dt3/2*self.q, dt2*self.q
        return np.array([[q11, 0,   q12, 0],
                         [0,   q11, 0,   q12],
                         [q12, 0,   q22, 0],
                         [0,   q12, 0,   q22]], dtype=float)

    def predict(self, dt: float) -> None:
        if self._F_cache is None or self._F_cache[0] != dt:
            self._F_cache = (dt, self._F(dt))
        if self._Q_cache is None or self._Q_cache[0] != dt:
            self._Q_cache = (dt, self._Q(dt))
        F, Q = self._F_cache[1], self._Q_cache[1]
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z: np.ndarray) -> None:
        z = z.reshape(2, 1)
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        y = z - H @ self.x
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

    # helpers
    def z_pred(self) -> np.ndarray:
        return self.x[:2, 0]

###############################################################################
# TRACK (KF + ID history)
###############################################################################
class _Track:
    __slots__ = ("kf", "id_history", "age", "time_since_update")

    def __init__(self, kf: KalmanFilter2DConstVel, init_id: Optional[int]):
        self.kf = kf
        self.id_history: deque[int] = deque(maxlen=_MAX_ID_HISTORY)
        if init_id is not None:
            self.id_history.append(init_id)
        self.age = 0.0
        self.time_since_update = 0.0

    # id utils
    def add_id(self, eid: int):
        self.id_history.append(eid)

    def id_freq(self, eid: int) -> int:
        return self.id_history.count(eid)

    def last_id(self) -> Optional[int]:
        return self.id_history[-1] if self.id_history else None

###############################################################################
# TRACKER MANAGER (Euclidean distance + ID‑tie‑break)
###############################################################################
class TrackerManager:
    """Greedy matching per detection: choose nearest track; if ties use ID history."""

    def __init__(self, max_unmatched: float = 10.0):
        self._tracks: List[_Track] = []
        self._max_unmatched = max_unmatched

    # ---------------- update ----------------
    def update(self, cloud_pts: np.ndarray, id_pts: np.ndarray, dt: float):
        # 0. predict & age
        for trk in self._tracks:
            trk.kf.predict(dt)
            trk.age += dt
            trk.time_since_update += dt

        # 1. build detection list
        det_xy, det_ids = [], []
        if cloud_pts.size:
            det_xy.extend(cloud_pts[:, :2].astype(float))
            det_ids.extend([None]*len(cloud_pts))
        if id_pts.size:
            det_xy.extend(id_pts[:, 1:3].astype(float))
            det_ids.extend(id_pts[:, 0].astype(int))

        # 2. for each detection greedily match
        unmatched_dets: List[int] = []
        for di, (xy, eid) in enumerate(zip(det_xy, det_ids)):
            nearest_idx, nearest_dist = self._find_nearest_track(xy, eid)
            if nearest_idx is None or nearest_dist > _DIST_THRESH:
                # no suitable track
                unmatched_dets.append(di)
            else:
                self._commit(nearest_idx, xy, eid)

        # 3. spawn new for unmatched
        for di in unmatched_dets:
            self._spawn_track(det_xy[di], det_ids[di])

        # 4. prune
        self._prune()

    def query(self) -> List[Tuple[Optional[int], np.ndarray]]:
        return [(trk.last_id(), trk.kf.z_pred().copy()) for trk in self._tracks]

    # ---------------- helpers ----------------
    def _euclid(self, a: np.ndarray, b: np.ndarray) -> float:
        return float(np.linalg.norm(a - b))

    def _find_nearest_track(self, xy: np.ndarray, eid: Optional[int]):
        """Return (track_index, distance) of best candidate, or (None, inf)."""
        best_idx, best_dist = None, np.inf
        tie_candidates: List[int] = []
        for idx, trk in enumerate(self._tracks):
            dist = self._euclid(xy, trk.kf.z_pred())
            if dist < best_dist - 1e-6:  # strictly better
                best_dist = dist
                best_idx = idx
                tie_candidates = [idx]
            elif abs(dist - best_dist) <= 1e-6:  # tie within epsilon
                tie_candidates.append(idx)

        if len(tie_candidates) > 1 and eid is not None:
            # choose one with highest id frequency
            best_idx = max(tie_candidates, key=lambda i: self._tracks[i].id_freq(eid))
        return best_idx, best_dist

    def _commit(self, ti: int, xy: np.ndarray, eid: Optional[int]):
        trk = self._tracks[ti]
        trk.kf.update(xy)
        trk.time_since_update = 0.0
        if eid is not None:
            trk.add_id(int(eid))

    def _spawn_track(self, xy, eid):
        kf = KalmanFilter2DConstVel(np.hstack([xy, np.zeros(2)]))
        self._tracks.append(_Track(kf, eid))

    def _prune(self):
        self._tracks = [t for t in self._tracks if t.time_since_update <= self._max_unmatched]

###############################################################################
# DEMO
###############################################################################
if __name__ == "__main__":
    np.random.seed(0)
    tm = TrackerManager()

    n, dt, T = 5, 0.1, 5
    steps = int(T/dt)
    pos0 = np.random.randn(n, 2)*20
    vel  = np.random.randn(n, 2)*2

    for k in range(steps):
        t = k*dt
        truth = pos0 + vel*t
        cloud = truth + np.random.randn(*truth.shape)*0.7
        id_mask = (np.arange(n)%2==0) & (k%5==0)
        ids = np.arange(100,100+n)[id_mask]
        id_det = np.hstack([ids.reshape(-1,1), truth[id_mask] + np.random.randn(np.sum(id_mask),2)*0.2])
        tm.update(cloud, id_det, dt)
        if k%10==0:
            print(f"t={t:4.1f}s")
            for lid, xy in tm.query():
                print(f"  Track {lid}: {xy}")
            print("-")
