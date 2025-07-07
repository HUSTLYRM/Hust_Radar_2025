import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.stats import chi2
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict, Set

###############################################################################
# CONFIGURATION OBJECTS
###############################################################################
@dataclass
class KFConfig:
    """Tunable parameters for a *planar* constant‑velocity Kalman filter.

    Attributes
    ----------
    P_init : np.ndarray (4×4)
        Initial state covariance.
    R : np.ndarray (2×2)
        Measurement noise covariance.
    q : float
        Initial process‑noise spectral density.
    adapt_q : bool
        Enable very simple adaptive process‑noise scaling based on innovation.
    q_min, q_max : float
        Clamp range for adaptive *q*.
    adapt_factor : float
        Multiplicative step when innovation is large / small.
    init_speed : float
        When a new track is born, velocity magnitude is initialised to this value
        (direction unknown ⇒ we simply leave velocity = 0 but set larger cov).
    max_speed : float
        Hard cap (m/s) imposed after every predict & update.
    """

    P_init: np.ndarray = field(default_factory=lambda: np.diag([20.0, 20.0, 25.0, 25.0]))
    R: np.ndarray = field(default_factory=lambda: np.diag([0.8, 0.8]))
    q: float = 1.0

    # adaptive‑noise parameters
    adapt_q: bool = True
    q_min: float = 0.05
    q_max: float = 10.0
    adapt_factor: float = 1.5  # factor up/down when innovation large/small

    # speed control
    init_speed: float = 2.0  # m/s
    max_speed: float = 15.0  # m/s


@dataclass
class TrackerConfig:
    """Top‑level tracker hyper‑parameters."""

    mahalanobis_threshold_p: float = 0.99  # gating χ²‑CDF (df=2)
    max_unmatched_time: float = 10.0       # seconds before a track is pruned
    kf_cfg: KFConfig = KFConfig()          # Kalman parameters

###############################################################################
# 2‑D CONSTANT‑VELOCITY KALMAN FILTER (with adaptive noise & speed clamp)
###############################################################################
class KalmanFilter2DConstVel:
    """Planar constant‑velocity Kalman filter **with adaptive Q & speed clamp**."""

    def __init__(self, xy_init: np.ndarray, cfg: KFConfig):
        self.cfg = cfg
        # State: [x, y, vx, vy]
        self.x = xy_init.reshape(4, 1)
        self.P = cfg.P_init.copy().astype(float)
        self.R = cfg.R.copy().astype(float)
        self.q = cfg.q  # will be modified if adapt_q == True
        # caches to avoid re‑building F/Q for constant dt
        self._F_cache: Optional[Tuple[float, np.ndarray]] = None
        self._Q_cache: Optional[Tuple[float, np.ndarray]] = None

    # ---------------------------- helpers to build system matrices ----------
    @staticmethod
    def _F(dt: float) -> np.ndarray:
        F = np.eye(4)
        F[0, 2] = dt
        F[1, 3] = dt
        return F

    def _Q(self, dt: float) -> np.ndarray:
        dt2, dt3, dt4 = dt**2, dt**3, dt**4
        q11 = dt4/4 * self.q
        q12 = dt3/2 * self.q
        q22 = dt2     * self.q
        return np.array([[q11, 0,   q12, 0],
                         [0,   q11, 0,   q12],
                         [q12, 0,   q22, 0],
                         [0,   q12, 0,   q22]], dtype=float)

    # ---------------------------- predict / update -------------------------
    def predict(self, dt: float) -> None:
        # build / cache F & Q
        if self._F_cache is None or self._F_cache[0] != dt:
            self._F_cache = (dt, self._F(dt))
        F = self._F_cache[1]
        if self._Q_cache is None or self._Q_cache[0] != (dt, self.q):
            self._Q_cache = ((dt, self.q), self._Q(dt))
        Q = self._Q_cache[1]

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self._clamp_speed()

    def update(self, z: np.ndarray) -> None:
        z = z.reshape(2, 1)
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        y = z - H @ self.x  # innovation
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        self._clamp_speed()
        if self.cfg.adapt_q:
            # innovation Mahalanobis distance d² = yᵀ S⁻¹ y (df=2)
            d2 = float(y.T @ np.linalg.inv(S) @ y)
            if d2 > 6:               # large residual  ⇒ increase q
                self.q = min(self.q * self.cfg.adapt_factor, self.cfg.q_max)
            elif d2 < 1:             # very small residual ⇒ decrease q
                self.q = max(self.q / self.cfg.adapt_factor, self.cfg.q_min)
            # invalidate Q‑cache so new q reflected
            self._Q_cache = None

    # ---------------------------- helpers ----------------------------------
    def z_pred(self) -> np.ndarray:
        return self.x[:2, 0]

    def S_inv(self) -> np.ndarray:
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        S = H @ self.P @ H.T + self.R
        return np.linalg.inv(S)

    def _clamp_speed(self):
        vx, vy = self.x[2, 0], self.x[3, 0]
        speed = np.hypot(vx, vy)
        if speed > self.cfg.max_speed:
            scale = self.cfg.max_speed / speed
            self.x[2, 0] *= scale
            self.x[3, 0] *= scale

###############################################################################
# TRACK CONTAINER (internal)
###############################################################################
class _Track:
    __slots__ = ("kf", "id", "age", "time_since_update")
    def __init__(self, kf: KalmanFilter2DConstVel, tid: int):
        self.kf = kf
        self.id: int = tid
        self.age: float = 0.0
        self.time_since_update: float = 0.0

###############################################################################
# MULTI‑OBJECT TRACKER MANAGER (2‑D) – *unchanged API*
###############################################################################
class TrackerManager:
    """Multi‑object tracker using adaptive 2‑D Kalman filters."""

    def __init__(self, cfg: Optional[TrackerConfig] = None):
        self.cfg = cfg or TrackerConfig()
        self._tracks: List[_Track] = []
        self._next_id: int = 1
        self._gate_thresh2: float = chi2.ppf(self.cfg.mahalanobis_threshold_p, df=2)

    # ---------------------------------------------------------------- PUBLIC
    def update(self, cloud_pts: np.ndarray, id_pts: np.ndarray, dt: float) -> None:
        # 0. predict
        for trk in self._tracks:
            trk.kf.predict(dt)
            trk.age += dt
            trk.time_since_update += dt

        # 1. detections list
        det_xy, det_ids = self._build_detections(cloud_pts, id_pts)
        if not self._tracks:
            for xy, did in zip(det_xy, det_ids):
                self._start_new(xy, did)
            return
        if not det_xy:
            self._prune()
            return

        # 2. distances
        D, gated = self._mahalanobis(det_xy)
        # 3. association
        a_trk, a_det = self._id_priority(det_xy, det_ids, gated)
        self._hungarian(det_xy, det_ids, gated, a_trk, a_det)
        # 4. spawn new
        for di, xy in enumerate(det_xy):
            if di not in a_det:
                self._start_new(xy, det_ids[di])
        # 5. prune
        self._prune()

    def query(self) -> List[Tuple[int, np.ndarray]]:
        return [(t.id, t.kf.z_pred().copy()) for t in self._tracks]

    # ------------------------ internal helpers (unchanged) -------------------
    def _build_detections(self, cloud_pts: np.ndarray, id_pts: np.ndarray):
        det_xy, det_ids = [], []
        if cloud_pts.size:
            det_xy.extend(list(cloud_pts[:, :2].astype(float)))
            det_ids.extend([None] * len(cloud_pts))
        if id_pts.size:
            det_xy.extend(list(id_pts[:, 1:3].astype(float)))
            det_ids.extend(list(id_pts[:, 0].astype(int)))
        return det_xy, det_ids

    def _mahalanobis(self, det_xy: List[np.ndarray]):
        num_tracks, num_dets = len(self._tracks), len(det_xy)
        D = np.full((num_tracks, num_dets), np.inf)
        det_arr = np.vstack(det_xy)
        for ti, trk in enumerate(self._tracks):
            mu, Sinv = trk.kf.z_pred(), trk.kf.S_inv()
            diff = det_arr - mu
            D[ti] = np.einsum('ij,jk,ik->i', diff, Sinv, diff)
        gated = np.where(D <= self._gate_thresh2, D, np.inf)
        return D, gated

    def _id_priority(self, det_xy, det_ids, gated):
        assigned_trk, assigned_det, id2trk = set(), set(), {}
        for idx, trk in enumerate(self._tracks):
            id2trk.setdefault(trk.id, []).append(idx)
        for di, ext_id in enumerate(det_ids):
            if ext_id is None:
                continue
            cand = id2trk.get(ext_id)
            if not cand:
                continue
            ti_best, dist_best = min(((ti, gated[ti, di]) for ti in cand), key=lambda x: x[1])
            if np.isfinite(dist_best):
                self._assoc(ti_best, di, det_xy[di], ext_id, assigned_trk, assigned_det)
        return assigned_trk, assigned_det

    def _hungarian(self, det_xy, det_ids, gated, a_trk, a_det):
        untrk = [ti for ti in range(len(self._tracks)) if ti not in a_trk]
        undet = [di for di in range(len(det_xy)) if di not in a_det]
        if not untrk or not undet:
            return
        sub = gated[np.ix_(untrk, undet)]
        rows, cols = linear_sum_assignment(sub)
        for r, c in zip(rows, cols):
            if np.isfinite(sub[r, c]):
                self._assoc(untrk[r], undet[c], det_xy[undet[c]], det_ids[undet[c]], a_trk, a_det)

    def _assoc(self, ti, di, det_xy, ext_id, a_trk, a_det):
        trk = self._tracks[ti]
        trk.kf.update(det_xy)
        trk.time_since_update = 0.0
        a_trk.add
