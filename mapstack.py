"""LiDAR 지도 쌓기: 로봇이 본 장애물 점을 격자에 계속 누적해서, 지나간 곳이 지도에 남게 함.

- 좌표: 로봇 odom(월드) 좌표, 격자 RES(m).
- 같은 칸이 MIN_HITS번 이상 보여야 장애물로 침 (지나가는 사람·잡음 걸러냄).
- 저장할 때는 "출발점(home) 기준" 좌표로 저장 → 다음에 로봇을 같은 자리·방향에 두고 불러오면 맞음.
"""
import json
import math
import os
import threading
import time

import cv2
import numpy as np

RES = 0.05
MIN_HITS = 3
SELF_R = 0.45     # 로봇 몸 근처 점은 안 쌓음


class MapStack:
    def __init__(self):
        self.lock = threading.Lock()
        self.enabled = True
        self.version = 0              # 바뀔 때마다 증가 (화면이 새로 받아갈지 판단)
        self.clear()

    def clear(self, home=None):
        with self.lock:
            self.hits = {}            # (i, j) → 본 횟수
            self.version += 1
            self.home = home          # 지도 기준점 (x, y, yaw) — 처음 쌓을 때의 로봇 자리
            self.name = ""
            self._occ_cache = None

    def add(self, xy, pose):
        """xy: N x 2 (월드), pose: (x, y, yaw)"""
        if not self.enabled or pose is None or not len(xy):
            return
        far = np.hypot(xy[:, 0] - pose[0], xy[:, 1] - pose[1]) > SELF_R
        ij = np.floor(xy[far] / RES).astype(np.int32)
        if not len(ij):
            return
        cells, counts = np.unique(ij, axis=0, return_counts=True)
        with self.lock:
            if self.home is None:
                self.home = tuple(float(v) for v in pose)
            h = self.hits
            before = len(self._occupied_keys())
            for (i, j), c in zip(cells.tolist(), counts.tolist()):
                k = (i, j)
                h[k] = min(h.get(k, 0) + 1, 255)      # 한 번 업데이트에 1번만 셈 (점 개수와 무관)
            self._occ_cache = None
            if len(self._occupied_keys()) != before:
                self.version += 1

    def _occupied_keys(self):
        if self._occ_cache is None:
            self._occ_cache = [k for k, v in self.hits.items() if v >= MIN_HITS]
        return self._occ_cache

    def occupied_xy(self):
        """장애물 칸 중심 (월드) N x 2"""
        with self.lock:
            ks = self._occupied_keys()
            if not ks:
                return np.zeros((0, 2), np.float32)
            return (np.array(ks, np.float32) + 0.5) * RES

    def cells(self):
        with self.lock:
            return {"res": RES, "version": self.version, "enabled": self.enabled, "name": self.name,
                    "cells": [v for k in self._occupied_keys() for v in k]}

    def info(self):
        with self.lock:
            return {"version": self.version, "enabled": self.enabled, "name": self.name,
                    "n": len(self._occupied_keys()), "has_home": self.home is not None}

    # ── 저장 / 불러오기 (출발점 기준) ──
    def save(self, path_noext, title=""):
        with self.lock:
            if self.home is None or not self.hits:
                return False, "아직 쌓인 지도가 없어요"
            hx, hy, hyaw = self.home
            c, s = math.cos(hyaw), math.sin(hyaw)
            pts = []
            for (i, j), v in self.hits.items():
                if v >= MIN_HITS:
                    wx, wy = (i + 0.5) * RES - hx, (j + 0.5) * RES - hy
                    pts.append([round(c * wx + s * wy, 3), round(-s * wx + c * wy, 3)])
            self.name = title
        os.makedirs(os.path.dirname(path_noext), exist_ok=True)
        with open(path_noext + ".json", "w", encoding="utf-8") as f:
            json.dump({"name": title, "frame": "home: x=forward, y=left (m)", "res": RES,
                       "created": time.strftime("%Y-%m-%dT%H:%M:%S"), "points": pts}, f)
        self._png(np.array(pts), path_noext + ".png", title)
        return True, f"{len(pts)}칸"

    def load(self, path_noext, pose):
        """지금 로봇이 저장할 때의 출발점에 서 있다고 보고 지도를 맞춰 넣음."""
        with open(path_noext + ".json", encoding="utf-8") as f:
            d = json.load(f)
        x0, y0, yaw0 = pose
        c, s = math.cos(yaw0), math.sin(yaw0)
        with self.lock:
            self.hits = {}
            for lx, ly in d["points"]:
                wx, wy = x0 + c * lx - s * ly, y0 + s * lx + c * ly
                self.hits[(math.floor(wx / RES), math.floor(wy / RES))] = MIN_HITS + 2
            self.home = tuple(pose)
            self.name = d.get("name", "")
            self._occ_cache = None
            self.version += 1
        return len(d["points"])

    @staticmethod
    def _png(pts, path, title):
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
