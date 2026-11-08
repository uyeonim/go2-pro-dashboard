"""정해진 경로 주행 — 로봇 자체 위치(odom)만 사용 (라이다·ROS 없이)

Go2는 다리 움직임 + IMU로 자기 위치를 계산해서 보내줌(LF_SPORT_MOD_STATE의 position, yaw).
이 값으로 "웨이포인트 → 웨이포인트"를 따라 걷게 한다. 좌표는 "출발할 때 로봇 기준":
  x = 출발 시 로봇이 보던 앞쪽(m), y = 왼쪽(m)

사용법 (.venv\\Scripts\\python route_drive.py ...)
  square --size 1.0          한 변 1m 정사각형 (앞 → 왼쪽으로 돌며) 후 제자리로
  line --dist 2              앞으로 2m 갔다가 돌아오기
  record 이름                리모컨으로 걸어서 경로 녹화 (Ctrl+C로 저장)
  run 이름 [--loop 3]        녹화한 경로 따라 걷기
  list                       저장된 경로 목록
  공통: --speed 0.3 (m/s, 최대 0.6)   --sim (로봇 없이 시뮬레이션)

안전: 시작 전에 리모컨으로 로봇을 세워 두고(L2+A), 리모컨을 손에 들 것(비상정지 L2+B).
      Ctrl+C → 즉시 정지. 위치 데이터가 0.5초 넘게 안 오면 자동 정지.
결과: routes/logs/<시각>_<이름>.csv (실제 지나간 위치) + .png (계획 vs 실제 그림)
"""
import argparse
import asyncio
import csv
import json
import math
import os
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
ROUTES = os.path.join(HERE, "routes")
LOGS = os.path.join(ROUTES, "logs")

MAX_SPEED = 0.6      # m/s 상한 (실내)
MAX_TURN = 0.8       # rad/s
REACH = 0.10         # 웨이포인트 도착 판정 (m)
STALE = 0.5          # 위치가 이 시간(s) 넘게 안 오면 정지
WP_TIMEOUT = 40      # 웨이포인트 하나에 이보다 오래 걸리면 중단 (s)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clip(v, lo, hi):
    return max(lo, min(hi, v))


# ───────────────────────── 로봇 연결 ─────────────────────────
class Robot:
    """실제 Go2 (WebRTC). pose()=(x, y, yaw, 받은시각), move(vx, vyaw), stop()"""

    def __init__(self, ip, key):
        self.ip, self.key = ip, key
        self._pose = None
        self.conn = None

    async def connect(self):
        from unitree_webrtc_connect import RTC_TOPIC, SPORT_CMD, UnitreeWebRTCConnection, WebRTCConnectionMethod
        self.RTC_TOPIC, self.SPORT_CMD = RTC_TOPIC, SPORT_CMD
        kw = {"aes_128_key": self.key} if self.key else {}
        self.conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=self.ip, **kw)
        print(f"로봇 {self.ip} 에 연결 중...")
        await asyncio.wait_for(self.conn.connect(), 25)
        self.conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LF_SPORT_MOD_STATE"], self._on_sport)
        for _ in range(50):                       # 첫 위치 기다리기 (최대 5초)
            if self._pose:
                break
            await asyncio.sleep(0.1)
        if not self._pose:
            raise RuntimeError("로봇 위치(odom)가 안 들어와요")
        print("연결됨 ✓")

    def _on_sport(self, msg):
        d = msg["data"]
        self._pose = (d["position"][0], d["position"][1], d["imu_state"]["rpy"][2], time.time())

    def pose(self):
        return self._pose

    def _req(self, api, param=None):
        opts = {"api_id": self.SPORT_CMD[api]}
        if param is not None:
            opts["parameter"] = param
        # 응답을 기다리지 않음(10Hz로 보내니 막히면 안 됨)
        t = asyncio.ensure_future(self.conn.datachannel.pub_sub.publish_request_new(self.RTC_TOPIC["SPORT_MOD"], opts))
        t.add_done_callback(lambda f: f.exception())

    def move(self, vx, vyaw):
        self._req("Move", {"x": round(vx, 3), "y": 0, "z": round(vyaw, 3)})

    def stop(self):
        self._req("StopMove")

    async def close(self):
        if self.conn:
            try:
                await asyncio.wait_for(self.conn.disconnect(), 5)
            except Exception:
                pass


class SimRobot:
    """로봇 없이 테스트용: 명령대로 움직이는 가짜 로봇 (+약간의 반응 지연)"""

    def __init__(self):
        self.x, self.y, self.yaw = 5.0, -2.0, 0.7   # 일부러 원점이 아닌 곳에서 시작
        self.vx = self.vyaw = 0.0
        self.cmd = (0.0, 0.0)
        self.t = time.time()

    async def connect(self):
        print("시뮬레이션 모드 (로봇 연결 안 함)")

    def _step(self):
        now = time.time()
        dt, self.t = now - self.t, now
        a = min(1.0, dt / 0.3)                       # 0.3초 정도 늦게 따라감
        self.vx += (self.cmd[0] - self.vx) * a
        self.vyaw += (self.cmd[1] - self.vyaw) * a
        self.yaw = wrap(self.yaw + self.vyaw * dt)
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt

    def pose(self):
        self._step()
        return (self.x, self.y, self.yaw, time.time())

    def move(self, vx, vyaw):
        self._step()
        self.cmd = (vx, vyaw)

    def stop(self):
        self.move(0.0, 0.0)

    async def close(self):
        pass


# ──────────────────── 출발 기준 좌표 ↔ 로봇 odom 좌표 ────────────────────
class Frame:
    def __init__(self, x0, y0, yaw0):
        self.x0, self.y0, self.yaw0 = x0, y0, yaw0
        self.c, self.s = math.cos(yaw0), math.sin(yaw0)

    def to_world(self, lx, ly):
        return self.x0 + self.c * lx - self.s * ly, self.y0 + self.s * lx + self.c * ly

    def to_local(self, wx, wy):
        dx, dy = wx - self.x0, wy - self.y0
        return self.c * dx + self.s * dy, -self.s * dx + self.c * dy


# ───────────────────────── 경로 따라가기 ─────────────────────────
async def follow(robot, points, speed, name, loops=1):
    """points: 출발 기준 [(x앞, y왼쪽), ...]"""
    p = robot.pose()
    frame = Frame(p[0], p[1], p[2])
    os.makedirs(LOGS, exist_ok=True)
    stamp = f"{datetime.now():%Y%m%d_%H%M%S}_{name}"
    log_path = os.path.join(LOGS, stamp + ".csv")
    trail = [(0.0, 0.0)]
    plan = [(0.0, 0.0)] + list(points) * loops
    t0 = time.time()
    ok = True
    with open(log_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x_local", "y_local", "yaw_local", "target_i", "target_x", "target_y", "cmd_vx", "cmd_vyaw"])
        for loop in range(loops):
            for i, (tx, ty) in enumerate(points):
                wx, wy = frame.to_world(tx, ty)
                print(f"\n→ [{loop + 1}/{loops}] 지점 {i + 1}/{len(points)}: 앞 {tx:+.2f}m, 왼쪽 {ty:+.2f}m")
                t_wp = time.time()
                while True:
                    x, y, yaw, stamp_t = robot.pose()
                    if time.time() - stamp_t > STALE:
                        robot.stop()
                        print("\r⚠ 위치 데이터가 끊겼어요 → 정지하고 기다리는 중...", end="")
                        await asyncio.sleep(0.2)
                        continue
                    dx, dy = wx - x, wy - y
                    dist = math.hypot(dx, dy)
                    if dist < REACH:
                        break
                    if time.time() - t_wp > WP_TIMEOUT:
                        print("\n⚠ 이 지점에 너무 오래 걸려서 중단해요 (막혔거나 미끄러짐)")
                        ok = False
                        break
                    err = wrap(math.atan2(dy, dx) - yaw)
                    vyaw = clip(1.5 * err, -MAX_TURN, MAX_TURN)
                    if abs(err) > 0.5:          # 방향이 많이 틀어졌으면 제자리 회전 먼저
                        vx = 0.0
                    else:                        # 가까워질수록 천천히, 방향 틀어진 만큼 감속
                        vx = clip(0.8 * dist, 0.12, speed) * max(0.0, math.cos(err))
                    robot.move(vx, vyaw)
                    lx, ly = frame.to_local(x, y)
                    w.writerow([round(time.time() - t0, 2), round(lx, 3), round(ly, 3),
                                round(wrap(yaw - frame.yaw0), 3), i, tx, ty, round(vx, 3), round(vyaw, 3)])
                    if math.hypot(lx - trail[-1][0], ly - trail[-1][1]) > 0.03:
                        trail.append((lx, ly))
                    print(f"\r   남은 거리 {dist:4.2f}m  방향오차 {math.degrees(err):+5.0f}°  "
                          f"명령 앞 {vx:.2f}m/s 회전 {vyaw:+.2f}rad/s   ", end="")
                    await asyncio.sleep(0.1)
                robot.stop()
                if not ok:
                    break
                await asyncio.sleep(0.3)
            if not ok:
                break
    robot.stop()
    x, y, _, _ = robot.pose()
    lx, ly = frame.to_local(x, y)
    end = plan[-1]
    print(f"\n{'✅ 완주' if ok else '⏹ 중단'}  걸린 시간 {time.time() - t0:.0f}s  "
          f"마지막 목표와 차이 {math.hypot(lx - end[0], ly - end[1]) * 100:.0f}cm")
    png = draw(plan, trail, os.path.join(LOGS, stamp + ".png"), name)
    print(f"기록: {log_path}\n그림: {png}")
    return ok


def draw(plan, trail, path, title):
    """계획 경로(파랑) vs 실제 지나간 길(빨강) — OpenCV로 그림"""
    import cv2
    import numpy as np
    pts = plan + trail
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
    W = H = 700
    m = 70
    sc = (W - 2 * m) / span
    cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2

    def px(p):  # 위쪽 = 출발 방향(앞), 왼쪽 = 왼쪽
        return int(W / 2 - (p[1] - cy) * sc), int(H / 2 - (p[0] - cx) * sc)

    img = np.full((H, W, 3), 255, np.uint8)
    g = 0.5 if span > 3 else 0.25                   # 격자
    k = math.floor((cx - span) / g)
    while k * g < cx + span:
        y0 = px((k * g, cy))[1]
        cv2.line(img, (0, y0), (W, y0), (235, 235, 235), 1)
        k += 1
    k = math.floor((cy - span) / g)
    while k * g < cy + span:
        x0 = px((cx, k * g))[0]
        cv2.line(img, (x0, 0), (x0, H), (235, 235, 235), 1)
        k += 1
    cv2.polylines(img, [np.array([px(p) for p in plan], np.int32)], False, (200, 120, 40), 2, cv2.LINE_AA)
    for i, p in enumerate(plan):
        cv2.circle(img, px(p), 6, (200, 120, 40), -1, cv2.LINE_AA)
    if len(trail) > 1:
        cv2.polylines(img, [np.array([px(p) for p in trail], np.int32)], False, (40, 40, 220), 2, cv2.LINE_AA)
    s = px((0, 0))
    cv2.arrowedLine(img, s, (s[0], s[1] - 35), (0, 150, 0), 2, tipLength=0.3)
    cv2.putText(img, f"route: {title}   grid {g}m", (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 60, 60), 1, cv2.LINE_AA)
    cv2.putText(img, "blue = plan   red = actual   green = start (facing)", (12, H - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 60), 1, cv2.LINE_AA)
    cv2.imwrite(path, img)
    return path


# ───────────────────────── 녹화 ─────────────────────────
async def record(robot, name, gap=0.25):
    p = robot.pose()
    frame = Frame(p[0], p[1], p[2])
    pts = []
    last = (0.0, 0.0)
    print("녹화 중 — 리모컨으로 원하는 길을 걸어 주세요. 끝나면 Ctrl+C")
    try:
        while True:
            x, y, _, _ = robot.pose()
            lx, ly = frame.to_local(x, y)
            if math.hypot(lx - last[0], ly - last[1]) >= gap:
                last = (round(lx, 3), round(ly, 3))
                pts.append(last)
            print(f"\r  지점 {len(pts)}개   지금 위치: 앞 {lx:+.2f}m, 왼쪽 {ly:+.2f}m   ", end="")
            await asyncio.sleep(0.1)
    except asyncio.CancelledError:
        pass
    finally:
        x, y, _, _ = robot.pose()
        lx, ly = frame.to_local(x, y)
        if not pts or math.hypot(lx - pts[-1][0], ly - pts[-1][1]) > 0.05:
            pts.append((round(lx, 3), round(ly, 3)))
        save(name, pts)


def save(name, pts):
    os.makedirs(ROUTES, exist_ok=True)
    path = os.path.join(ROUTES, name + ".json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"name": name, "frame": "start: x=forward, y=left (m)",
                   "created": datetime.now().isoformat(timespec="seconds"), "points": pts}, f, ensure_ascii=False, indent=1)
    print(f"\n✅ 저장: {path} ({len(pts)}개 지점)\n   다음: route_drive.py run {name}  (같은 출발 자리·방향에서)")


def load(name):
    path = os.path.join(ROUTES, name + ".json")
    if not os.path.exists(path):
        raise SystemExit(f"❌ 경로가 없어요: {path}  (먼저 record {name})")
    with open(path, encoding="utf-8") as f:
        return [tuple(p) for p in json.load(f)["points"]]


def thin(pts, gap=0.4):
    """녹화 점이 너무 촘촘하면 멈칫거림 → 간격 gap 이상만 남김 (마지막 점은 유지)"""
    out = []
    for p in pts:
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= gap:
            out.append(p)
    if pts and out[-1] != pts[-1]:
        out.append(pts[-1])
    return out


# ───────────────────────── 실행 ─────────────────────────
async def main(a):
    if a.cmd == "list":
        os.makedirs(ROUTES, exist_ok=True)
        names = sorted(f[:-5] for f in os.listdir(ROUTES) if f.endswith(".json"))
        print("저장된 경로:", ", ".join(names) if names else "(없음)")
        return
    if a.cmd == "square":
        s = a.size
        pts, name = [(s, 0), (s, s), (0, s), (0, 0)], f"square{s:g}m"
    elif a.cmd == "line":
        pts, name = [(a.dist, 0), (0, 0)], f"line{a.dist:g}m"
    elif a.cmd == "run":
        pts, name = thin(load(a.name)), a.name
    else:
        pts, name = None, a.name

    robot = SimRobot() if a.sim else Robot(a.ip, a.key)
    await robot.connect()
    try:
        if a.cmd == "record":
            if a.sim:
                raise SystemExit("record는 실제 로봇에서만 돼요")
            await record(robot, a.name)
        else:
            speed = clip(a.speed, 0.1, MAX_SPEED)
            print(f"경로 '{name}': {len(pts)}개 지점, 최고속도 {speed} m/s, {a.loop}바퀴")
            print("⚠ 리모컨을 손에 들고 (비상정지 L2+B). 3초 뒤 출발 — 멈추려면 Ctrl+C")
            await asyncio.sleep(0 if a.sim else 3)
            await follow(robot, pts, speed, name, a.loop)
    finally:
        robot.stop()
        await asyncio.sleep(0.3)
        await robot.close()


if __name__ == "__main__":
    import sensors  # noqa: F401  (.env 읽기)

    common = argparse.ArgumentParser(add_help=False)   # 옵션을 명령 뒤에 써도 되게
    common.add_argument("--ip", default=os.environ.get("UNITREE_ROBOT_IP"))
    common.add_argument("--key", default=os.environ.get("UNITREE_AES_128_KEY"))
    common.add_argument("--speed", type=float, default=0.3, help="최고 속도 m/s (기본 0.3, 최대 0.6)")
    common.add_argument("--loop", type=int, default=1, help="몇 바퀴 반복")
    common.add_argument("--sim", action="store_true", help="로봇 없이 시뮬레이션")
    p = argparse.ArgumentParser(description="Go2 정해진 경로 주행 (odom)")
    sp = p.add_subparsers(dest="cmd", required=True)
    q = sp.add_parser("square", parents=[common]); q.add_argument("--size", type=float, default=1.0)
    q = sp.add_parser("line", parents=[common]); q.add_argument("--dist", type=float, default=2.0)
    q = sp.add_parser("record", parents=[common]); q.add_argument("name")
    q = sp.add_parser("run", parents=[common]); q.add_argument("name")
    sp.add_parser("list", parents=[common])
    a = p.parse_args()
    if not a.sim and a.cmd != "list" and not a.ip:
        p.error(".env에 UNITREE_ROBOT_IP가 없어요")
    try:
        asyncio.run(main(a))
    except KeyboardInterrupt:
        print("\n⏹ Ctrl+C — 정지")
