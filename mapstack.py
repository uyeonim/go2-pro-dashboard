"""LiDAR 지도 쌓기 — 확률 지도 + 스캔 정합 (작은 2D SLAM)

1) 확률 지도 (occupancy grid, log-odds)
   각 칸에 "장애물일 확률"을 log-odds로 누적. 점이 맺힌 칸은 +, 로봇에서 그 점까지 빛줄기가
   지나간 칸은 − (비어 있다는 증거). 지나간 사람·순간 잡음은 다음에 그 자리가 비어 보이면 지워짐.
2) 스캔 정합 (likelihood-field / chamfer 정합, Gauss-Newton)
   새 LiDAR 점을 쌓기 전에, 이미 쌓인 지도의 거리장(distance transform)에서 점들이 벽에 가장 잘
   붙도록 (dx, dy, dθ)를 구해 odom 오차를 보정. 보정은 누적 변환 C (지도 = C · odom)로 관리.
3) 작은 조각 제거: 확률 지도에서 남은 아주 작은 점 덩어리(연결 성분)는 표시·길찾기에서 뺌.

좌표: "지도 좌표" = C·odom. 대시보드의 로봇 위치·경로·길찾기는 모두 지도 좌표를 씀.
저장: 처음 쌓기 시작한 자리(home) 기준으로 저장 → 같은 자리·방향에서 불러오면 맞고,
      조금 틀려도 스캔 정합이 로봇이 움직이면서 맞춰 줌.
"""
import json
import math
import os
import threading
import time

import cv2
import numpy as np

RES = 0.05                 # 격자 크기 (m)
SIZE = 1600                # 격자 한 변 칸 수 (80m)
SELF_R = 0.45              # 로봇 몸 근처 점은 무시
MAX_RAY = 8.0              # 이보다 먼 점은 빛줄기(빈 공간) 계산에 안 씀
L_OCC, L_FREE = 0.85, -0.30   # 맺힘 / 지나감 증거 (맺힘을 더 크게: 얇은 물체 보호)
L_MIN, L_MAX = -2.0, 3.5
L_THR = 0.9                # 이 이상이면 장애물 (확률 ≈ 0.71)
MIN_BLOB = 3               # 이보다 작은 덩어리(칸 수)는 잡음으로 보고 숨김


def _rot(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s], [s, c]], np.float32)


class MapStack:
    def __init__(self):
        self.lock = threading.Lock()
        self.enabled = True
        self.version = 0
        self.C = (0.0, 0.0, 0.0)          # 보정 변환 (tx, ty, θ): 지도 = R(θ)·odom + t
        self.clear()

    # ── 좌표 변환 ──
    def correct(self, pose):
        """odom 자세 → 지도 자세"""
        if pose is None:
            return None
        tx, ty, th = self.C
        c, s = math.cos(th), math.sin(th)
        x, y, yaw = pose[:3]
        return (c * x - s * y + tx, s * x + c * y + ty, (yaw + th + math.pi) % (2 * math.pi) - math.pi)

    def correct_pts(self, xy):
        tx, ty, th = self.C
        return (xy @ _rot(th).T + np.array([tx, ty], np.float32)).astype(np.float32)

    def clear(self, home=None):
        with self.lock:
            self.L = None                 # log-odds 격자 (처음 점이 들어올 때 만듦)
            self.i0 = self.j0 = 0         # 격자 [0,0]의 월드 칸 번호
            self.home = home              # 지도 기준점 (지도 좌표 자세)
            self.name = ""
            self.updates = 0
            self.relocalize = 0           # 불러온 직후 몇 번은 더 크게 보정 허용
            self.match = None             # 마지막 정합 결과 (화면 표시용)
            self._occ = None
            self._occ_t = 0.0
            self.version += 1

    # ── 메인: LiDAR 한 번 들어올 때 ──
    def process(self, xy_odom, pose_odom):
        """odom 좌표 점 → (정합 보정 후) 지도 좌표 점. 확률 지도도 갱신."""
        xy = self.correct_pts(xy_odom.astype(np.float32))
        if not self.enabled or pose_odom is None or not len(xy):
            return xy
        with self.lock:
            pose = self.correct(pose_odom)
            if self.L is None:
                self._alloc(pose)
            if self.home is None:
                self.home = tuple(pose)
            if self.updates >= 3:          # 지도가 조금 쌓인 뒤부터 정합
                d = self._register(xy, pose)
                if d is not None:
                    self._compose(d)
                    xy = self.correct_pts(xy_odom.astype(np.float32))
                    pose = self.correct(pose_odom)
            self._integrate(xy, pose)
            self.updates += 1
            self._occ = None
            if time.time() - self._occ_t > 1.0:      # 화면용 버전은 1초에 한 번만 올림
                self._occ_t = time.time()
                self.version += 1
        return xy

    def _alloc(self, pose):
        self.L = np.zeros((SIZE, SIZE), np.float32)
        self.i0 = int(math.floor(pose[0] / RES)) - SIZE // 2
        self.j0 = int(math.floor(pose[1] / RES)) - SIZE // 2

    def _idx(self, xy):
        return (np.floor(xy[:, 0] / RES).astype(np.int32) - self.i0,
                np.floor(xy[:, 1] / RES).astype(np.int32) - self.j0)

    def _integrate(self, xy, pose):
        far = np.hypot(xy[:, 0] - pose[0], xy[:, 1] - pose[1])
        keep = far > SELF_R
        ii, jj = self._idx(xy[keep])
        ok = (ii >= 1) & (ii < SIZE - 1) & (jj >= 1) & (jj < SIZE - 1)
        ii, jj = ii[ok], jj[ok]
        if not len(ii):
            return
        cells = np.unique(np.stack([ii, jj], 1), axis=0)
        ri, rj = int(math.floor(pose[0] / RES)) - self.i0, int(math.floor(pose[1] / RES)) - self.j0
        # 빛줄기가 지나간 칸 = 빈 공간 (가까운 점만, 로컬 창에서 cv2.line으로 한 번에)
        dist = np.hypot(cells[:, 0] - ri, cells[:, 1] - rj) * RES
        near = cells[dist < MAX_RAY]
        r = int(MAX_RAY / RES) + 2
        a0, b0 = max(0, ri - r), max(0, rj - r)
        a1, b1 = min(SIZE, ri + r), min(SIZE, rj + r)
        free = np.zeros((a1 - a0, b1 - b0), np.uint8)
        if len(near):
            # cv2 좌표는 (x=열, y=행) → (j, i)
            lines = [np.array([[rj - b0, ri - a0], [c[1] - b0, c[0] - a0]], np.int32) for c in near]
            cv2.polylines(free, lines, False, 1, 1)
        hit = np.zeros_like(free)
        inwin = (cells[:, 0] >= a0) & (cells[:, 0] < a1) & (cells[:, 1] >= b0) & (cells[:, 1] < b1)
        hc = cells[inwin]
        hit[hc[:, 0] - a0, hc[:, 1] - b0] = 1
        hit = cv2.dilate(hit, np.ones((3, 3), np.uint8))     # 점 바로 옆 칸은 "빈 공간"으로 깎지 않음
        win = self.L[a0:a1, b0:b1]
        win[(free == 1) & (hit == 0)] += L_FREE
        win[hc[:, 0] - a0, hc[:, 1] - b0] += L_OCC
        np.clip(win, L_MIN, L_MAX, out=win)
        out = cells[~inwin]                                   # 창 밖의 먼 점은 맺힘만 반영
        if len(out):
            self.L[out[:, 0], out[:, 1]] = np.minimum(self.L[out[:, 0], out[:, 1]] + L_OCC, L_MAX)

    # ── 스캔 정합 ──
    def _register(self, xy, pose):
        """지도 거리장에 점들이 붙도록 (dx, dy, dθ) 계산. 믿을 만하면 반환, 아니면 None."""
        ri, rj = int(math.floor(pose[0] / RES)) - self.i0, int(math.floor(pose[1] / RES)) - self.j0
        r = int(MAX_RAY / RES)
        a0, b0 = max(0, ri - r), max(0, rj - r)
        a1, b1 = min(SIZE, ri + r), min(SIZE, rj + r)
        occ = (self.L[a0:a1, b0:b1] > L_THR).astype(np.uint8)
        if occ.sum() < 80:
            return None
        df = cv2.distanceTransform(1 - occ, cv2.DIST_L2, 5)          # 칸 단위 거리
        df = np.minimum(df, 12).astype(np.float32)
        gi = cv2.Sobel(df, cv2.CV_32F, 0, 1, ksize=3) / 8.0          # ∂d/∂i (행 = x)
        gj = cv2.Sobel(df, cv2.CV_32F, 1, 0, ksize=3) / 8.0          # ∂d/∂j (열 = y)
        d = np.hypot(xy[:, 0] - pose[0], xy[:, 1] - pose[1])
        P = xy[(d > SELF_R) & (d < MAX_RAY)]
        if len(P) < 60:
            return None
        if len(P) > 1200:
            P = P[np.random.choice(len(P), 1200, replace=False)]
        c = np.array(pose[:2], np.float32)
        Q = P - c                                                     # 로봇 중심 기준
        tx = ty = th = 0.0
        # 한 번(0.5초)에 허용하는 보정 크기: odom 오차는 천천히 쌓이므로 작게 → 잡음으로 튀는 것 방지
        maxt = 0.5 if self.relocalize > 0 else 0.04
        maxr = math.radians(15 if self.relocalize > 0 else 0.8)

        def bilinear(img, fi, fj):
            i0 = np.clip(np.floor(fi).astype(int), 0, img.shape[0] - 2)
            j0 = np.clip(np.floor(fj).astype(int), 0, img.shape[1] - 2)
            a = np.clip(fi - i0, 0, 1)
            b = np.clip(fj - j0, 0, 1)
            return ((1 - a) * (1 - b) * img[i0, j0] + a * (1 - b) * img[i0 + 1, j0]
                    + (1 - a) * b * img[i0, j0 + 1] + a * b * img[i0 + 1, j0 + 1])

        def sample(T):
            R = _rot(T[2])
            W = Q @ R.T + c + [T[0], T[1]]
            # 칸 중심 기준 좌표 (거리장 값은 칸 중심에서 잰 것)
            fi = (W[:, 0] / RES) - self.i0 - a0 - 0.5
            fj = (W[:, 1] / RES) - self.j0 - b0 - 0.5
            return W, bilinear(df, fi, fj), bilinear(gi, fi, fj), bilinear(gj, fi, fj)

        def tcost(dd):   # Cauchy 비용: 벽에서 먼 점(사람·잡음)은 비용이 거의 안 늘어서 영향이 작음
            return float(np.mean(np.log1p((np.minimum(dd, 4) / 1.5) ** 2)))

        _, d0, _, _ = sample((0, 0, 0))
        inl0 = d0 < 4                                                 # 0.2m 안 = 벽에 붙은 점
        if inl0.mean() < 0.35:
            return None                                               # 지도와 너무 달라서 정합 안 함
        cost0 = tcost(d0)
        for _ in range(12):
            W, dd, ggi, ggj = sample((tx, ty, th))
            w = (dd < 4).astype(np.float32) / (1.0 + (dd / 1.5) ** 2)   # Cauchy 가중치: 벽에서 먼 점(잡음)은 무시
            R = _rot(th)
            Rq = Q @ R.T                                               # ∂W/∂θ = perp(R q)
            dth = -Rq[:, 1] * ggi + Rq[:, 0] * ggj
            J = np.stack([ggi, ggj, dth], 1) / RES                    # 칸 → m
            H = (J * w[:, None]).T @ J + np.diag([1e-3, 1e-3, 1e-3]) * len(J)
            g = (J * w[:, None]).T @ dd
            step = -np.linalg.solve(H, g)
            tx, ty, th = tx + step[0], ty + step[1], th + step[2]
            if abs(step[0]) + abs(step[1]) < 1e-4 and abs(step[2]) < 1e-4:
                break
        _, d1, _, _ = sample((tx, ty, th))
        cost1 = tcost(d1)
        # 방향이 한쪽으로만 정해지는 경우(긴 벽 하나만 보임 등) 거부: H의 가장 작은 고유값 확인
        Hn = H / max(1, len(J))
        well = np.linalg.eigvalsh(Hn[:2, :2]).min() > 0.02 * np.trace(Hn[:2, :2])
        ok = bool(cost1 < cost0 * 0.995 and well and math.hypot(tx, ty) <= maxt and abs(th) <= maxr)
        self.match = {"before": round(cost0, 4), "after": round(cost1, 4), "inlier": round(float(inl0.mean()), 2),
                      "dx": round(float(tx), 3), "dy": round(float(ty), 3), "dth": round(math.degrees(th), 2), "ok": ok}
        if self.relocalize > 0:
            self.relocalize -= 1
        if not ok:
            return None
        return (tx, ty, th, c)

    def _compose(self, d):
        """새 보정 d(로봇 중심 c 기준 회전 후 이동)를 누적 변환 C에 합침"""
        tx, ty, th, c = d
        R = _rot(th)
        Ct = np.array(self.C[:2], np.float32)
        # 지도' = R·(지도 − c) + c + t,  지도 = R_C·odom + t_C
        nt = R @ (Ct - c) + c + np.array([tx, ty], np.float32)
        self.C = (float(nt[0]), float(nt[1]), self.C[2] + th)

    # ── 출력 ──
    def _occupied_ij(self):
        if self._occ is None:
            if self.L is None:
                self._occ = np.zeros((0, 2), np.int32)
            else:
                m = (self.L > L_THR).astype(np.uint8)
                n, lab, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
                small = np.where(stats[:, cv2.CC_STAT_AREA] < MIN_BLOB)[0]
                if len(small):
                    m[np.isin(lab, small)] = 0                        # 작은 조각 제거
                ii, jj = np.nonzero(m)
                self._occ = np.stack([ii + self.i0, jj + self.j0], 1).astype(np.int32)
        return self._occ

    def occupied_xy(self):
        with self.lock:
            ij = self._occupied_ij()
            return (ij.astype(np.float32) + 0.5) * RES

    def cells(self):
        with self.lock:
            return {"res": RES, "version": self.version, "enabled": self.enabled, "name": self.name,
                    "cells": self._occupied_ij().ravel().tolist()}

    def info(self):
        with self.lock:
            return {"version": self.version, "enabled": self.enabled, "name": self.name,
                    "n": int(len(self._occupied_ij())), "has_home": self.home is not None,
                    "match": self.match, "corr": [round(self.C[0], 3), round(self.C[1], 3),
                                                  round(math.degrees(self.C[2]), 2)]}

    # ── 저장 / 불러오기 (출발점 기준) ──
    def save(self, path_noext, title=""):
        with self.lock:
            if self.home is None or self.L is None:
                return False, "아직 쌓인 지도가 없어요"
            ij = self._occupied_ij()
            if not len(ij):
                return False, "아직 쌓인 지도가 없어요"
            hx, hy, hyaw = self.home
            c, s = math.cos(hyaw), math.sin(hyaw)
            w = (ij.astype(np.float64) + 0.5) * RES - [hx, hy]
            pts = np.stack([c * w[:, 0] + s * w[:, 1], -s * w[:, 0] + c * w[:, 1]], 1)
            self.name = title
        os.makedirs(os.path.dirname(path_noext), exist_ok=True)
        with open(path_noext + ".json", "w", encoding="utf-8") as f:
            json.dump({"name": title, "frame": "home: x=forward, y=left (m)", "res": RES,
                       "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
                       "points": np.round(pts, 3).tolist()}, f)
        self._png(pts, path_noext + ".png")
        return True, f"{len(pts)}칸"

    def load(self, path_noext, pose_map):
        """지금 로봇이 저장할 때의 출발점에 서 있다고 보고 지도를 넣음. 이후 스캔 정합이 조금씩 맞춰 줌."""
        with open(path_noext + ".json", encoding="utf-8") as f:
            d = json.load(f)
        x0, y0, yaw0 = pose_map
        c, s = math.cos(yaw0), math.sin(yaw0)
        pts = np.array(d["points"], np.float64).reshape(-1, 2)
        with self.lock:
            self._alloc(pose_map)
            wx = x0 + c * pts[:, 0] - s * pts[:, 1]
            wy = y0 + s * pts[:, 0] + c * pts[:, 1]
            ii, jj = self._idx(np.stack([wx, wy], 1))
            ok = (ii >= 0) & (ii < SIZE) & (jj >= 0) & (jj < SIZE)
            self.L[ii[ok], jj[ok]] = L_MAX
            self.home = tuple(pose_map)
            self.name = d.get("name", "")
            self.updates = 3
            self.relocalize = 8            # 처음 몇 번은 크게 보정해서 출발 위치 오차를 맞춤
            self._occ = None
            self.version += 1
        return len(pts)

    @staticmethod
    def _png(pts, path):
        """보고서용 그림: 위쪽 = 출발 때 앞, 초록 화살표 = 출발점"""
        if not len(pts):
            return
        px_per_m = 40
        m = 1.0
        x0, x1 = pts[:, 0].min() - m, pts[:, 0].max() + m
        y0, y1 = pts[:, 1].min() - m, pts[:, 1].max() + m
        W, H = int((y1 - y0) * px_per_m) + 1, int((x1 - x0) * px_per_m) + 1
        img = np.full((H, W, 3), 255, np.uint8)
        for k in range(int(math.floor(y0)), int(math.ceil(y1)) + 1):   # 1m 격자
            u = int((y1 - k) * px_per_m)
            cv2.line(img, (u, 0), (u, H), (232, 232, 232), 1)
        for k in range(int(math.floor(x0)), int(math.ceil(x1)) + 1):
            v = int((x1 - k) * px_per_m)
            cv2.line(img, (0, v), (W, v), (232, 232, 232), 1)
        r = max(1, int(RES * px_per_m / 2))
        for x, y in pts:
            u, v = int((y1 - y) * px_per_m), int((x1 - x) * px_per_m)
            cv2.rectangle(img, (u - r, v - r), (u + r, v + r), (40, 40, 40), -1)
        hu, hv = int(y1 * px_per_m), int(x1 * px_per_m)
        cv2.arrowedLine(img, (hu, hv), (hu, hv - 30), (0, 160, 0), 3, tipLength=0.35)
        cv2.putText(img, "grid 1m, green arrow = start", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (90, 90, 90), 1, cv2.LINE_AA)
        with open(path, "wb") as f:                 # cv2.imwrite는 한글 파일 이름을 못 씀
            f.write(cv2.imencode(".png", img)[1].tobytes())
