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

from navigator import plan

HERE = os.path.dirname(os.path.abspath(__file__))
YUNET = os.path.join(HERE, "models", "face_detection_yunet_2023mar.onnx")

MAX_V, MAX_W = 1.2, 1.0  # 걷는 사람(~1.2m/s) 따라잡을 수 있게 / 회전은 너무 빠르면 UWB 지연 때문에 휘청임
STALE = 1.0              # 리모컨 값이 이보다 오래되면 정지
TURN_FIRST = 0.6         # 방향이 이보다 틀어지면 제자리에서 먼저 돌기 (rad, 약 34°)
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
    def __init__(self, get_pose, get_live_obstacles, get_jpeg, send_move, send_stop, send_action, log=print,
                 get_obstacles=None, robot_nav=None):
        self.get_pose, self.get_live = get_pose, get_live_obstacles
        # robot_nav: 로봇 내장 SLAM(robotslam.RobotSlam). 위치를 찾은 상태면 로봇 지도 길찾기로 따라감
        self.rnav = robot_nav
        self._goal_sent, self._goal_t = None, 0.0
        self.resting = False      # "엎드려" 뒤: "일어나" 할 때까지 따라가기 멈춤
        self.mode = ""            # "map"(로봇 지도 길찾기) / "direct"(직접)
        self.get_obstacles = get_obstacles or get_live_obstacles   # 길 찾기용 (지금 보이는 점 + 쌓은 지도)
        self._plan, self._plan_t = None, 0.0
        self._last_vx = 0.0
        self.get_jpeg = get_jpeg
        self.send_move, self.send_stop, self.send_action, self.log = send_move, send_stop, send_action, log
        self.filter = UwbFilter()
        self.face = FaceWatch()
        self.lock = threading.Lock()
        self.active = False
        self.dist_goal = 1.3
        self.speed = 0.8
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
        """방향은 '지도 기준 방향'(로봇 yaw + 측정값)으로 바꿔서 거름.
        → 로봇이 도는 동안 UWB 값이 늦게 따라와도(지연) 로봇 자신의 회전은 바로 반영돼서 덜 흔들림."""
        p = self.get_pose()
        if p is None:
            return
        try:
            d = dict(d, orientation_est=wrap(p[2] + self.sign * float(d["orientation_est"])))
        except (KeyError, TypeError, ValueError):
            return
        self.filter.update(d)

    def remote(self):
        """(거리, 로봇 기준 방향, 받은 시각) 또는 None"""
        g = self.filter.get()
        p = self.get_pose()
        if not g or p is None:
            return None
        return g[0], wrap(g[1] - p[2]), g[2]

    def status(self):
        r = self.remote()
        fresh = r is not None and time.time() - r[2] < STALE
        with self.lock:
            return {"active": self.active, "msg": self.msg, "dist_goal": self.dist_goal, "heart": self.heart,
                    "speed": self.speed, "plan": [list(q) for q in self._plan] if self._plan else None,
                    "remote": [round(r[0], 2), round(r[1], 3)] if fresh else None,
                    "face": self.face_seen, "actions": self.actions, "has_face_model": self.face.det is not None,
                    "mode": self.mode, "resting": self.resting}

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

    def set(self, dist_goal=None, heart=None, speed=None):
        with self.lock:
            if speed:
                self.speed = clip(float(speed), 0.2, MAX_V)
            if dist_goal:
                self.dist_goal = clip(float(dist_goal), 0.8, 3.0)
            if heart is not None:
                self.heart = bool(heart)

    def stop(self, why="정지"):
        was = self.active
        self._stop.set()
        with self.lock:
            self.active = False
            self.resting = False
            self.msg = f"멈춤 ({why})" if was else self.msg
        if was:
            self._stop_map_nav()
            self.send_stop()
            self.log(f"⏹ 따라가기 정지: {why}")

    def _stop_map_nav(self):
        if self.rnav is not None and self._goal_sent is not None:
            self.rnav.stop()
        self._goal_sent = None

    def perform(self, sport_name, kind):
        """따라가는 중에 동작하기: 멈추고 → 동작 → (엎드려가 아니면) 다시 따라가기.
        kind: 'act' 보통 동작 / 'rest' 엎드리기(일어나 할 때까지 쉼) / 'wake' 일어나서 다시 따라가기"""
        self._stop_map_nav()
        self.send_stop()
        time.sleep(0.25)
        self.send_action(sport_name)
        now = time.time()
        if kind == "rest":
            self.resting = True
            self.msg = "엎드려 쉬는 중 — '일어나' 하면 다시 따라가요"
        elif kind == "wake":
            time.sleep(1.5)
            self.send_action("BalanceStand")      # 일어선 뒤 걸을 수 있는 상태로
            self.resting = False
            self.action_until = time.time() + 1.0
        else:
            self.action_until = now + ACTION_TIME
            self.last_action = now

    # ── 따라가기 ──
    def _run(self):
        try:
            while not self._stop.is_set():
                time.sleep(0.1)
                now = time.time()
                if now < self.action_until or self.resting:   # 동작 중 / 엎드려 쉬는 중 → 이동 명령 안 보냄
                    continue
                r = self.remote()
                p = self.get_pose()
                if r is None or now - r[2] > STALE:
                    self._stop_map_nav()
                    self.send_stop()
                    with self.lock:
                        self.msg = "리모컨 신호가 없어요 — 리모컨을 켜고 로봇 근처에서 들어 주세요"
                    continue
                if self._map_follow(r, now):              # 로봇 지도 위치를 알면 로봇 내장 길찾기로
                    continue
                self.mode = "direct"
                dist, b, _ = r
                # UWB 거리는 약 0.4초 늦게 옴 → 그동안 우리가 앞으로 간 만큼 빼서 예측 (지나치게 다가가는 것 방지)
                dist = dist - 0.4 * self._last_vx
                err = dist - self.dist_goal
                head = b                                   # 향할 방향 (기본: 사람 쪽)
                state = "따라가는 중"
                # 사이에 장애물이 있으면 길 찾기(A*)로 돌아가는 길의 다음 점을 향함
                if err > 0.2 and p is not None:
                    way = self._detour(p, dist, b, now)
                    if way is not None:
                        head = wrap(math.atan2(way[1] - p[1], way[0] - p[0]) - p[2])
                        state = "장애물 돌아가는 중"
                else:
                    self._plan = None
                vyaw = clip(1.2 * head, -MAX_W, MAX_W)
                if abs(head) > TURN_FIRST:
                    vx = 0.0
                    state = f"방향 맞추는 중 ({math.degrees(head):+.0f}°)" if state == "따라가는 중" else state
                elif abs(err) < 0.2:
                    vx = 0.0
                    if abs(b) < 0.12:
                        vyaw = 0.0
                    state = "옆에 있어요"
                elif err > 0:
                    # 멀수록 빠르게 (걷는 사람 속도 따라잡기), 방향이 틀어진 만큼 감속
                    vx = clip(1.0 * err, 0.2, self.speed) * max(0.3, math.cos(head))
                else:
                    vx = clip(0.5 * err, -0.3, -0.1) if dist < self.dist_goal - 0.35 else 0.0
                    state = "너무 가까워서 물러나는 중" if vx < 0 else "옆에 있어요"
                if vx > 0 and self._front_blocked(p):
                    vx = 0.0
                    self._plan_t = 0.0                      # 바로 다시 길 찾기
                    state = "앞에 장애물 — 돌아갈 길 찾는 중"
                self._last_vx = vx
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

    def _map_follow(self, r, now):
        """로봇 내장 SLAM 위치를 알면: 사람 앞 지점을 목표로 로봇 내비게이션에 계속 새로 보냄. 처리했으면 True."""
        rn = self.rnav
        mp = rn.pose if rn is not None else None
        if rn is None or not rn.localized or not mp or now - mp[3] > 1.5:
            if self._goal_sent is not None:
                self._stop_map_nav()
            return False
        self.mode = "map"
        dist, b, _ = r
        x, y, yaw = mp[:3]
        px, py = x + dist * math.cos(yaw + b), y + dist * math.sin(yaw + b)      # 사람(리모컨) 위치 (지도 좌표)
        if dist < self.dist_goal + 0.3:                                          # 가까움 → 멈추고 사람 쪽 보기
            if self._goal_sent is not None:
                self._stop_map_nav()
            vyaw = clip(1.2 * b, -MAX_W, MAX_W) if abs(b) > 0.35 else 0.0
            if vyaw:
                self.send_move(0.0, vyaw)
            else:
                self.send_stop()
            self.msg = f"옆에 있어요 (로봇 지도) · 리모컨 {dist:.1f}m, {math.degrees(b):+.0f}°"
            return True
        ux, uy = (px - x) / dist, (py - y) / dist
        gx, gy = px - ux * self.dist_goal, py - uy * self.dist_goal
        gyaw = math.atan2(py - gy, px - gx)
        moved = self._goal_sent is None or math.hypot(gx - self._goal_sent[0], gy - self._goal_sent[1]) > 0.5
        if moved and now - self._goal_t > 1.0 or (self._goal_sent is not None and now - self._goal_t > 4.0):
            self._goal_sent, self._goal_t = (gx, gy), now
            rn.goto(gx, gy, gyaw)                        # 로봇이 지도·장애물 보고 스스로 길 찾음
        st = rn.nav_state or "준비"
        self.msg = f"로봇 지도로 따라가는 중 ({st}) · 리모컨 {dist:.1f}m, {math.degrees(b):+.0f}°"
        return True

    def _detour(self, p, dist, b, now):
        """로봇→사람 직선이 막혔으면 돌아가는 길의 다음 점(지도 좌표), 안 막혔으면 None."""
        ex, ey = p[0] + dist * math.cos(p[2] + b), p[1] + dist * math.sin(p[2] + b)   # 리모컨(사람) 위치
        ux, uy = (ex - p[0]) / max(dist, 1e-3), (ey - p[1]) / max(dist, 1e-3)
        gx, gy = ex - ux * self.dist_goal, ey - uy * self.dist_goal                   # 사람 앞 목표 지점
        obs, _ = self.get_obstacles()
        if not len(obs):
            self._plan = None
            return None
        # 직선 통로(폭 ±0.3m)에 장애물 점이 있나 — 로봇 몸 근처와 사람 주변(다리)은 제외
        dx, dy = obs[:, 0] - p[0], obs[:, 1] - p[1]
        along = dx * ux + dy * uy
        side = -dx * uy + dy * ux
        blocked = ((along > 0.45) & (along < dist - 0.6) & (np.abs(side) < 0.3)).sum() >= 3
        if blocked:
            self._blocked_t = now
        elif now - getattr(self, "_blocked_t", 0.0) > 2.0:          # 막힘이 풀리고 2초 지나야 직진으로 (왔다갔다 방지)
            self._plan = None
            return None
        if self._plan is None or now - self._plan_t > 2.0:            # 2초마다 다시 계산
            path, _ = plan(obs, (p[0], p[1]), (gx, gy))
            self._plan, self._plan_t = (path[1:] if path else None), now
        if not self._plan:
            return None
        while len(self._plan) > 1 and math.hypot(self._plan[0][0] - p[0], self._plan[0][1] - p[1]) < 0.3:
            self._plan.pop(0)
        return self._plan[0]

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
                self.perform(act, "act")              # 로봇 지도 길찾기 중이면 그것도 멈추고
                self.actions += 1
                self.look_since = None
                self.log(f"💗 바라봐서 {'하트' if act == 'FingerHeart' else '인사'}!")
