import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.stats import chi2
from typing import List, Tuple, Optional, Dict, Set

###############################################################################
# 2‑D CONSTANT‑VELOCITY KALMAN FILTER
###############################################################################
class KalmanFilter2DConstVel:
    """Constant‑velocity KF in *planar* (x‑y) space.

    State  : [x, y, vx, vy]ᵀ  (4×1)
    Measure: [x, y]ᵀ          (2×1)
    """

    def __init__(self,
                 xy_init: np.ndarray,
                 P_init: Optional[np.ndarray] = None,
                 R: Optional[np.ndarray] = None,
                 q: float = 1.0):
        self.x = xy_init.reshape(4, 1)
        self.P = (np.diag([10, 10, 10, 10]) if P_init is None else P_init.astype(float))
        self.R = (np.diag([1, 1]) if R is None else R.astype(float))
        self.q = q
        self._F_cache: Optional[Tuple[float, np.ndarray]] = None
        self._Q_cache: Optional[Tuple[float, np.ndarray]] = None

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def _F(dt: float) -> np.ndarray:
        F = np.eye(4)
        F[0, 2] = dt
        F[1, 3] = dt
        return F

    def _Q(self, dt: float) -> np.ndarray:
        q = self.q
        dt2, dt3, dt4 = dt**2, dt**3, dt**4
        q11, q12, q22 = dt4/4*q, dt3/2*q, dt2*q
        # assemble block for x & y identically
        Q = np.array([[q11, 0,   q12, 0],
                      [0,   q11, 0,   q12],
                      [q12, 0,   q22, 0],
                      [0,   q12, 0,   q22]], dtype=float)
        return Q

    # ------------------------------------------------------------------ predict / update
    def predict(self, dt: float) -> None:
        if self._F_cache is None or self._F_cache[0] != dt:
            self._F_cache = (dt, self._F(dt))
        F = self._F_cache[1]
        if self._Q_cache is None or self._Q_cache[0] != dt:
            self._Q_cache = (dt, self._Q(dt))
        Q = self._Q_cache[1]
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z: np.ndarray) -> None:
        z = z.reshape(2, 1)
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]], dtype=float)
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        y = z - H @ self.x
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

    # ------------------------------------------------------------------ accessors
    def z_pred(self) -> np.ndarray:
        return self.x[:2, 0]

    def S_inv(self) -> np.ndarray:
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]], dtype=float)
        S = H @ self.P @ H.T + self.R
        return np.linalg.inv(S)

###############################################################################
# TRACK CONTAINER
###############################################################################
class _Track:
    __slots__ = ("kf", "id", "age", "time_since_update")

    def __init__(self, kf: KalmanFilter2DConstVel, tid: int):
        self.kf = kf
        self.id: int = tid  # external id or internally assigned
        self.age: float = 0.0
        self.time_since_update: float = 0.0

###############################################################################
# MULTI‑OBJECT TRACKER MANAGER (2‑D)
###############################################################################
class TrackerManager:
    """Multi‑object tracker based on 2‑D constant‑velocity Kalman filters.

    Matching策略：
    1. **ID‑priority pass** 任何 detection 若包含 external *id*，首先与 *同 id* track 中
       Mahalanobis 距离最近者匹配（若距離在閾值内）。
    2. **Proximity pass** 剩余 detection / track 再执行匈牙利算法在 gated 距离矩阵上
       获得全局最近组合。
    这样既充分利用 id 信息，又保证空间最近的关联；大大降低误配风险。
    """

    def __init__(self,
                 mahalanobis_threshold_p: float = 0.99,
                 max_unmatched_time: float = 10.0):
        self._tracks: List[_Track] = []
        self._next_id: int = 1
        self._gate_thresh2: float = chi2.ppf(mahalanobis_threshold_p, df=2)
        self._max_unmatched = max_unmatched_time

    # ------------------------------------------------------------------ PUBLIC
    def update(self,
               cloud_pts: np.ndarray,
               id_pts: np.ndarray,
               dt: float) -> None:
        """Update trackers with a new frame (x‑y only)."""
        ############ 0. 预测 ###################################################
        for trk in self._tracks:
            trk.kf.predict(dt)
            trk.age += dt
            trk.time_since_update += dt

        ############ 1. 组装检测列表 ##########################################
        det_xy: List[np.ndarray] = []
        det_ids: List[Optional[int]] = []
        if cloud_pts.size:
            det_xy.extend(list(cloud_pts[:, :2].astype(float)))
            det_ids.extend([None] * len(cloud_pts))
        if id_pts.size:
            det_xy.extend(list(id_pts[:, 1:3].astype(float)))
            det_ids.extend(list(id_pts[:, 0].astype(int)))

        num_dets = len(det_xy)
        if num_dets == 0 and not self._tracks:
            return  # nothing to do at all

        num_tracks = len(self._tracks)
        if num_tracks == 0:
            for xy, did in zip(det_xy, det_ids):
                self._start_new(xy, did)
            return
        if num_dets == 0:
            self._prune()
            return

        # convenience arrays
        det_xy_arr = np.vstack(det_xy)  # (D,2)

        ############ 2‑a. 预计算所有 Mahalanobis 距离矩阵 ######################
        D = np.full((num_tracks, num_dets), np.inf)
        for ti, trk in enumerate(self._tracks):
            mu = trk.kf.z_pred()
            Sinv = trk.kf.S_inv()
            diff = det_xy_arr - mu  # broadcast (D,2)
            D[ti] = np.einsum('ij,jk,ik->i', diff, Sinv, diff)

        gated = D.copy()
        gated[gated > self._gate_thresh2] = np.inf

        ############ 2‑b. 第一阶段：ID 优先匹配 ################################
        assigned_trk: Set[int] = set()
        assigned_det: Set[int] = set()

        id_to_trk_indices: Dict[int, List[int]] = {}
        for idx, trk in enumerate(self._tracks):
            id_to_trk_indices.setdefault(trk.id, []).append(idx)

        for d_i, ext_id in enumerate(det_ids):
            if ext_id is None or d_i in assigned_det:
                continue
            candidate_trk_idxs = id_to_trk_indices.get(ext_id)
            if not candidate_trk_idxs:
                continue
            # pick candidate with smallest gated distance
            cand_dists = [(ti, gated[ti, d_i]) for ti in candidate_trk_idxs]
            ti_best, dist_best = min(cand_dists, key=lambda x: x[1])
            if np.isfinite(dist_best):
                self._associate(ti_best, d_i, det_xy[d_i], ext_id,
                                assigned_trk, assigned_det)

        ############ 2‑c. 第二阶段：匈牙利对剩余 #################################
        # build reduced matrices
        untrk = [ti for ti in range(num_tracks) if ti not in assigned_trk]
        undet = [di for di in range(num_dets) if di not in assigned_det]
        if untrk and undet:
            sub_D = gated[np.ix_(untrk, undet)]
            rows, cols = linear_sum_assignment(sub_D)
            for r_idx, c_idx in zip(rows, cols):
                ti = untrk[r_idx]
                di = undet[c_idx]
                if np.isfinite(sub_D[r_idx, c_idx]):
                    self._associate(ti, di, det_xy[di], det_ids[di],
                                    assigned_trk, assigned_det)

        ############ 3. 对剩余检测启动新轨迹 ####################################
        for di in range(num_dets):
            if di not in assigned_det:
                self._start_new(det_xy[di], det_ids[di])

        ############ 4. 清理老轨迹 #############################################
        self._prune()

    def query(self) -> List[Tuple[int, np.ndarray]]:
        """Return list[(id, xy_estimate)] for *active* tracks."""
        return [(trk.id, trk.kf.z_pred().copy()) for trk in self._tracks]

    # ------------------------------------------------------------------ helpers
    def _associate(self, ti: int, di: int, det_xy: np.ndarray, ext_id: Optional[int],
                   assigned_trk: Set[int], assigned_det: Set[int]):
        trk = self._tracks[ti]
        trk.kf.update(det_xy)
        trk.time_since_update = 0.0
        assigned_trk.add(ti)
        assigned_det.add(di)
        if ext_id is not None:
            trk.id = ext_id

    def _start_new(self, xy: np.ndarray, ext_id: Optional[int]):
        x_init = np.hstack([xy, np.zeros(2)])
        kf = KalmanFilter2DConstVel(xy_init=x_init)
        tid = int(ext_id) if ext_id is not None else self._next_id
        if ext_id is None:
            self._next_id += 1
        self._tracks.append(_Track(kf, tid))

    def _prune(self):
        self._tracks = [t for t in self._tracks if t.time_since_update <= self._max_unmatched]

###############################################################################
# SIMPLE DEMO (RUN FILE) ######################################################
###############################################################################
if __name__ == "__main__":
    np.random.seed(42)
    tm = TrackerManager()

    n_obj, dt, T = 6, 0.1, 6
    steps = int(T/dt)
    pos0 = np.random.randn(n_obj, 2) * 30
    vel = np.random.randn(n_obj, 2) * 2

    for k in range(steps):
        t = k * dt
        true_pos = pos0 + vel*t
        cloud = true_pos + np.random.randn(*true_pos.shape)*0.7
        id_mask = (np.arange(n_obj) % 3 == 0) & (k % 4 == 0)
        ids = np.arange(100, 100+n_obj)[id_mask]
        id_det = np.hstack([ids.reshape(-1,1),
                            true_pos[id_mask] + np.random.randn(np.sum(id_mask),2)*0.2])
        tm.update(cloud, id_det, dt)
        if k % 10 == 0:
            print(f"t={t:4.1f}s")
            for tid, xy in tm.query():
                print(f"  Track {tid}: {xy}")
            print("-")
