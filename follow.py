"""1번 기능: 리모컨(UWB 태그) 든 사람 따라가기 + 로봇을 바라보면 하트/인사.

UWB (rt/uwbstate) 실측으로 확인한 값 (2026-11-10, 로봇 주위를 반시계로 한 바퀴 돌며 기록):
  distance_est    : 로봇 → 리모컨 거리 (m)
  orientation_est : 로봇 정면 기준 리모컨이 있는 방향 (rad, 0 = 정면, 왼쪽 +, 반시계로 돌면 증가)
  pitch_est       : 올려다보는 각도 (리모컨이 로봇보다 높음)
  tag_roll/pitch/yaw : 리모컨 자체의 기울기·방향 (손에 든 각도) — 위치가 아님! (처음에 이걸 써서 빙빙 돌았음)
  buttons         : 리모컨 버튼 비트 (1, 2, 4, 8),  joystick: [x, y] (−1~1)
방향 값은 가끔 크게 튀므로(몸에 가려질 때 등) 걸러서 씀.

바라보기 판단: 카메라 영상에서 얼굴(YuNet)을 찾고, 코가 두 눈 가운데에 있으면 정면(로봇을 보는 중).
그 얼굴이 리모컨 방향과 맞는 사람일 때만 → 1초 넘게 보면 하트(FingerHeart) / 인사(Hello) 번갈아.
"""
import math
import os
import threading
import time
from collections import deque

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
YUNET = os.path.join(HERE, "models", "face_detection_yunet_2023mar.onnx")

MAX_V, MAX_W = 0.6, 0.9
STALE = 0.6              # 리모컨 값이 이보다 오래되면 정지
TURN_FIRST = 0.4         # 방향이 이보다 틀어지면 제자리에서 먼저 돌기 (rad, 약 23°)
FRONT_STOP = 0.5         # 바로 앞 장애물 거리 (m)
CAM_HFOV = math.radians(100)   # 카메라 가로 화각 (대략)
LOOK_HOLD = 1.0          # 이만큼(초) 바라보면 동작
COOLDOWN = 12.0          # 동작 사이 최소 간격 (초)
ACTION_TIME = 4.5        # 동작하는 동안 이동 멈춤 (초)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clip(v, lo, hi):
    return max(lo, min(hi, v))


class UwbFilter:
    """리모컨 거리·방향 거르기: 거리 = 최근 값 중앙값, 방향 = 튀는 값(앞뒤 뒤집힘)은 무시하고 부드럽게."""

    def __init__(self):
        self.dist = deque(maxlen=7)
        self.bearing = None
        self.out = 0              # 연속으로 튄 값 개수
        self.out_vals = []
        self.t = 0.0

    def update(self, d):
        try:
            dist, b = float(d["distance_est"]), float(d["orientation_est"])
        except (KeyError, TypeError, ValueError):
            return
        if not (0.05 < dist < 30):
            return
        self.dist.append(dist)
        self.t = time.time()
        if self.bearing is None:
            self.bearing = b
            return
        e = wrap(b - self.bearing)
        if abs(e) < math.radians(60):
            self.bearing = wrap(self.bearing + 0.25 * e)
            self.out, self.out_vals = 0, []
        else:                                    # 크게 튄 값: 계속 그쪽이면(약 0.9초) 진짜로 이동한 것
            self.out += 1
            self.out_vals.append(b)
            if self.out >= 8 and abs(wrap(self.out_vals[-1] - self.out_vals[0])) < math.radians(45):
                self.bearing, self.out, self.out_vals = b, 0, []

    def get(self):
        if not self.dist or self.bearing is None:
            return None
        return float(np.median(self.dist)), self.bearing, self.t


class FaceWatch:
    """카메라에서 '로봇을 바라보는 얼굴' 찾기 (YuNet)."""

    def __init__(self):
        self.det = None
        if os.path.exists(YUNET):
            try:
                self.det = cv2.FaceDetectorYN.create(YUNET, "", (320, 320), 0.7, 0.3, 50)
            except Exception:
                self.det = None

    def look(self, jpg):
        """반환: [(가로 방향 rad, 정면 정도 0~1, 눈 사이 px), ...]"""
        if self.det is None or jpg is None:
            return []
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return []
        h, w = img.shape[:2]
        if w > 640:
            img = cv2.resize(img, (640, int(h * 640 / w)))
            h, w = img.shape[:2]
        self.det.setInputSize((w, h))
        _, faces = self.det.detect(img)
        out = []
        if faces is None:
            return out
        for f in faces:
            x, y, fw, fh = f[:4]
            rex, rey, lex, ley, nx, ny = f[4:10]
            eye = math.hypot(lex - rex, ley - rey)
            if eye < 6:
                continue
            mid = (rex + lex) / 2
            frontal = max(0.0, 1.0 - abs(nx - mid) / (0.5 * eye))   # 코가 눈 가운데 = 1
            cx = x + fw / 2
            ang = -(cx / w - 0.5) * CAM_HFOV                         # 화면 오른쪽 = 로봇 기준 오른쪽(−)
            out.append((ang, frontal, eye))
        return out


class Follower:
    def __init__(self, get_pose, get_live_obstacles, get_jpeg, send_move, send_stop, send_action, log=print):
        self.get_pose, self.get_live = get_pose, get_live_obstacles
        self.get_jpeg = get_jpeg
        self.send_move, self.send_stop, self.send_action, self.log = send_move, send_stop, send_action, log
        self.filter = UwbFilter()
        self.face = FaceWatch()
        self.lock = threading.Lock()
        self.active = False
        self.dist_goal = 1.3
        self.speed = 0.5
        self.heart = True
        self.sign = 1.0           # 방향 부호 (실측: 왼쪽 + 로 확인됨)
        self.msg = ""
        self.look_since = None
        self.last_action = 0.0
        self.action_until = 0.0
        self.actions = 0
        self.face_seen = None     # 마지막 얼굴 상태 (화면 표시)
        self._stop = threading.Event()
        threading.Thread(target=self._face_loop, daemon=True).start()

    # ── 외부에서 ──
    def feed_uwb(self, d):
        self.filter.update(d)

    def remote(self):
        """(거리, 로봇 기준 방향[부호 보정됨], 받은 시각) 또는 None"""
        g = self.filter.get()
        if not g:
            return None
        return g[0], self.sign * g[1], g[2]

    def status(self):
        r = self.remote()
        fresh = r is not None and time.time() - r[2] < STALE
        with self.lock:
            return {"active": self.active, "msg": self.msg, "dist_goal": self.dist_goal, "heart": self.heart,
                    "remote": [round(r[0], 2), round(r[1], 3)] if fresh else None,
                    "face": self.face_seen, "actions": self.actions, "has_face_model": self.face.det is not None}

    def start(self, dist_goal=None, heart=None, speed=None):
        with self.lock:
            if dist_goal:
                self.dist_goal = clip(float(dist_goal), 0.8, 3.0)
            if heart is not None:
                self.heart = bool(heart)
            if speed:
                self.speed = clip(float(speed), 0.2, MAX_V)
            if self.active:
                return
            self.active = True
            self.msg = "리모컨 찾는 중…"
        self._stop.clear()
        threading.Thread(target=self._run, daemon=True).start()
        self.log(f"🐕 따라가기 시작 (거리 {self.dist_goal}m, 하트 {'켬' if self.heart else '끔'})")

    def set(self, dist_goal=None, heart=None):
        with self.lock:
            if dist_goal:
                self.dist_goal = clip(float(dist_goal), 0.8, 3.0)
            if heart is not None:
                self.heart = bool(heart)

    def stop(self, why="정지"):
        was = self.active
        self._stop.set()
        with self.lock:
            self.active = False
            self.msg = f"멈춤 ({why})" if was else self.msg
        if was:
            self.send_stop()
            self.log(f"⏹ 따라가기 정지: {why}")

    # ── 따라가기 ──
    def _run(self):
        try:
            while not self._stop.is_set():
                time.sleep(0.1)
                now = time.time()
                if now < self.action_until:          # 하트/인사 하는 중 → 이동 명령 안 보냄
                    continue
                r = self.remote()
                p = self.get_pose()
                if r is None or now - r[2] > STALE:
                    self.send_stop()
                    with self.lock:
                        self.msg = "리모컨 신호가 없어요 — 리모컨을 켜고 로봇 근처에서 들어 주세요"
                    continue
                dist, b, _ = r
                err = dist - self.dist_goal
                vyaw = clip(1.3 * b, -MAX_W, MAX_W)
                if abs(b) > TURN_FIRST:
                    vx = 0.0
                    state = f"방향 맞추는 중 ({math.degrees(b):+.0f}°)"
                elif abs(err) < 0.2:
                    vx = 0.0
                    if abs(b) < 0.12:
                        vyaw = 0.0
                    state = "옆에 있어요"
                elif err > 0:
                    vx = clip(0.7 * err, 0.15, self.speed) * math.cos(b)
                    state = "따라가는 중"
                else:
                    vx = clip(0.5 * err, -0.25, -0.1) if dist < self.dist_goal - 0.35 else 0.0
                    state = "너무 가까워서 물러나는 중" if vx < 0 else "옆에 있어요"
                if vx > 0 and self._front_blocked(p):
                    vx = 0.0
                    state = "앞에 장애물 — 멈춤"
                if vx == 0 and vyaw == 0:
                    self.send_stop()
                else:
                    self.send_move(vx, vyaw)
                with self.lock:
                    self.msg = f"{state} · 리모컨 {dist:.1f}m, {math.degrees(b):+.0f}°"
        except Exception as e:
            self.send_stop()
            with self.lock:
                self.active = False
                self.msg = f"오류로 멈춤: {type(e).__name__}: {e}"
            self.log(self.msg)

    def _front_blocked(self, pose):
        obs, _ = self.get_live()
        if pose is None or not len(obs):
            return False
        dx, dy = obs[:, 0] - pose[0], obs[:, 1] - pose[1]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        fx, fy = c * dx + s * dy, -s * dx + c * dy
        return bool((((fx > 0.32) & (fx < 0.32 + FRONT_STOP) & (np.abs(fy) < 0.22)).sum()) >= 3)

    # ── 바라보면 하트/인사 ──
    def _face_loop(self):
        while True:
            time.sleep(0.2)
            if not (self.active and self.heart and self.face.det is not None):
                self.look_since = None
                continue
            try:
                faces = self.face.look(self.get_jpeg())
            except Exception:
                faces = []
            r = self.remote()
            target = None
            for ang, frontal, eye in faces:
                # 리모컨 방향과 맞는 얼굴만 (리모컨이 없으면 아무 얼굴)
                if r is not None and time.time() - r[2] < STALE and abs(wrap(ang - r[1])) > math.radians(30):
                    continue
                if target is None or frontal > target[1]:
                    target = (ang, frontal, eye)
            looking = target is not None and target[1] > 0.6
            with self.lock:
                self.face_seen = None if target is None else ("보는 중" if looking else "얼굴 보임")
            now = time.time()
            if not looking:
                self.look_since = None
                continue
            if self.look_since is None:
                self.look_since = now
            if now - self.look_since >= LOOK_HOLD and now - self.last_action > COOLDOWN:
                act = "FingerHeart" if self.actions % 2 == 0 else "Hello"
                self.send_stop()
                time.sleep(0.2)
                self.send_action(act)
                self.actions += 1
                self.last_action = now
                self.action_until = now + ACTION_TIME
                self.look_since = None
                self.log(f"💗 바라봐서 {'하트' if act == 'FingerHeart' else '인사'}!")
