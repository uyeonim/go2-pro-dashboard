"""Go2 실시간 대시보드 (카메라 · 센서 · LiDAR · 모터 온도 · 경로 · 녹화).

실행:
  .venv\\Scripts\\python dashboard.py            # 실제 로봇 (IP와 키는 .env에서 읽음)
  .venv\\Scripts\\python dashboard.py --demo     # 로봇 없이 가짜 데이터로 화면 개발/확인
  .venv\\Scripts\\python dashboard.py --ip 192.168.x.x --key <AES키>

실행하면 브라우저가 자동으로 열리고, 로봇에 자동 연결돼요.
로봇은 WebRTC를 한 번에 하나만 받으니, 폰 앱은 종료해 두세요.
"""
import argparse
import asyncio
import csv
import hashlib
import hmac
import logging
import math
import os
import threading
import time
import webbrowser
from datetime import timedelta

import cv2
import numpy as np
from flask import Flask, Response, jsonify, redirect, request, send_from_directory, session

from sensors import make_conn  # import 시 .env도 읽음
from unitree_webrtc_connect import RTC_TOPIC

# 접속 정보는 .env에서 읽음 (sensors.py의 load_env). 코드에 키를 적지 않음.
DEFAULT_IP = os.environ.get("UNITREE_ROBOT_IP")
DEFAULT_KEY = os.environ.get("UNITREE_AES_128_KEY")
PASSWORD = os.environ.get("DASHBOARD_PASSWORD", "")       # 비우면 잠금 없음
PAGES_ORIGIN = os.environ.get("DASHBOARD_PAGES_ORIGIN", "https://uyeonim.github.io")  # 고정 주소 입구 (GitHub Pages)

BASE = os.path.dirname(os.path.abspath(__file__))
REC_DIR = os.path.join(BASE, "recordings")   # 녹화(영상 + 센서 CSV)
CAP_DIR = os.path.join(BASE, "captures")     # 사진
FRAME_W, JPEG_Q = 960, 75                    # 전송용 영상 크기/화질 (공개 주소에서도 가볍게)
TRAIL_MAX = 3000                             # 경로 점 최대 개수
LEGS = ["FR", "FL", "RR", "RL"]              # Go2 모터 순서: 다리별 (hip, thigh, calf)

state = {
    "sport": None, "sport_t": 0.0,
    "low": None, "low_t": 0.0,
    "lidar": [], "lidar_t": 0.0,      # 위에서 본 [x, y] 점 목록 (m, 월드 좌표)
    "obstacle_lidar": None,           # LiDAR로 계산한 [앞, 왼쪽, 뒤, 오른쪽] 거리 (m)
    "trail": [],                      # 지나온 경로 [[x, y], ...]
    "status": "연결 대기 중...",
    "connected": False,
    # 화면 표시용 상태: connecting(연결 시도) / retrying(실패 후 대기) / reconnecting(끊겨서 재연결) / connected
    "phase": "connecting",
    "attempt": 0,
    "error": None,                    # 마지막 실패 이유 (사람이 읽는 안내)
    "demo": False,
}

# 라이브러리 오류 → 무엇을 확인하면 되는지 안내
ERROR_HINTS = {
    "RobotBusyError": "다른 앱이나 프로그램이 로봇에 연결돼 있어요. 폰 앱을 꺼 주세요.",
    "AesKeyRequiredError": "AES 키가 없어요. .env의 UNITREE_AES_128_KEY를 확인하세요.",
    "AesKeyRejectedError": "AES 키가 틀려요. .env의 UNITREE_AES_128_KEY를 확인하세요.",
    "LocalSignalingPortError": "로봇에 닿지 않아요. 로봇 전원, Wi-Fi, IP를 확인하세요.",
    "NoSdpAnswerError": "로봇이 응답하지 않아요. 잠시 후 다시 시도해요.",
    "DataChannelTimeoutError": "연결이 끝까지 안 돼요. PC와 로봇이 같은 Wi-Fi인지 확인하세요.",
    "TimeoutError": "연결이 끝까지 안 돼요. PC와 로봇이 같은 Wi-Fi인지 확인하세요.",
}

_frame_lock = threading.Lock()
_latest_jpeg = [None]
_latest_t = [0.0]  # 마지막 프레임 받은 시각

logging.getLogger("werkzeug").setLevel(logging.ERROR)  # /data 요청 로그 숨김
app = Flask(__name__, static_folder="web", static_url_path="")
# 로그인 쿠키 서명용 (비밀번호가 바뀌면 기존 로그인은 풀림), 로그인은 30일 유지
app.secret_key = hashlib.sha256(f"go2-dashboard:{PASSWORD}:{DEFAULT_KEY}".encode()).digest()
app.permanent_session_lifetime = timedelta(days=30)
OPEN_PATHS = {"/health", "/login", "/config.js", "/favicon.ico"}   # 잠금 없이 열리는 주소

LOGIN_PAGE = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Go2 대시보드 로그인</title>
<style>body{{margin:0;min-height:100vh;display:flex;align-items:center;justify-content:center;background:#0e1116;color:#e6ebf2;
font-family:'Segoe UI','Malgun Gothic',system-ui,sans-serif}}form{{background:#161b24;border:1px solid #2a3340;border-radius:14px;
padding:28px;width:min(340px,90vw)}}h1{{font-size:18px;margin:0 0 6px}}p{{color:#8b97a7;font-size:13px;margin:0 0 18px}}
input{{width:100%;box-sizing:border-box;padding:11px 12px;border-radius:8px;border:1px solid #2a3340;background:#0e1116;color:#e6ebf2;
font-size:15px}}button{{width:100%;margin-top:12px;padding:11px;border:0;border-radius:8px;background:#4c8dff;color:#fff;
font-size:15px;font-weight:700;cursor:pointer}}.err{{color:#ff8fa3;font-size:13px;margin-top:10px}}</style></head>
<body><form method="post"><h1>Unitree Go2 · 실시간 대시보드</h1><p>실제 로봇 화면은 비밀번호가 필요해요.</p>
<input type="password" name="pw" placeholder="비밀번호" autofocus required><button>들어가기</button>{err}</form></body></html>"""


@app.before_request
def require_login():
    """DASHBOARD_PASSWORD가 있으면 로그인한 사람만 실제 화면/데이터를 볼 수 있음."""
    if not PASSWORD or request.path in OPEN_PATHS or session.get("ok"):
        return None
    if request.path == "/":
        return redirect("/login")
    return jsonify({"error": "login required"}), 401


@app.route("/login", methods=["GET", "POST"])
def login():
    if not PASSWORD:
        return redirect("/")
    if request.method == "POST":
        if hmac.compare_digest(request.form.get("pw", ""), PASSWORD):
            session.permanent = True
            session["ok"] = True
            return redirect("/")
        return LOGIN_PAGE.format(err='<div class="err">비밀번호가 달라요.</div>'), 403
    return LOGIN_PAGE.format(err="")


@app.route("/config.js")
def config_js():
    """노트북 서버로 열면 항상 실제(live) 모드 — web/config.js(고정 주소용)를 덮어씀."""
    r = Response("window.GO2_MODE = 'live';\n", mimetype="application/javascript")
    r.headers["Cache-Control"] = "no-store"
    return r


@app.route("/health", methods=["GET", "OPTIONS"])
def health():
    """고정 주소 입구(GitHub Pages)가 '노트북 대시보드가 켜져 있나'를 확인하는 곳. 켜짐 여부만 알려줌.
    이 노트북에서 열면 ts.net 주소가 Tailscale 내부 IP로 풀려서, Chrome이 '공개 사이트 → 내부망' 요청에
    허용 헤더(Access-Control-Allow-Private-Network)를 요구할 수 있음 → 미리 확인 요청(OPTIONS)에도 응답."""
    r = Response(status=204) if request.method == "OPTIONS" else jsonify({"ok": True, "robot_connected": state["connected"]})
    r.headers["Access-Control-Allow-Origin"] = PAGES_ORIGIN
    r.headers["Access-Control-Allow-Methods"] = "GET"
    r.headers["Access-Control-Allow-Private-Network"] = "true"
    r.headers["Cache-Control"] = "no-store"
    return r


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def _placeholder(text):
    img = np.full((540, 960, 3), (35, 30, 30), dtype=np.uint8)
    cv2.putText(img, text, (60, 280), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (180, 180, 190), 2)
    return cv2.imencode(".jpg", img)[1].tobytes()


PLACEHOLDER = _placeholder("waiting for camera...")


# ───────────────────────── 데이터 갱신 (실제 로봇 / 데모 공용) ─────────────────────────
def set_frame(img_bgr):
    h, w = img_bgr.shape[:2]
    if w > FRAME_W:
        img_bgr = cv2.resize(img_bgr, (FRAME_W, int(h * FRAME_W / w)), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", img_bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
    if ok:
        with _frame_lock:
            _latest_jpeg[0] = buf.tobytes()
            _latest_t[0] = time.time()


def update_sport(d):
    state["sport"] = d
    state["sport_t"] = time.time()
    x, y = d["position"][0], d["position"][1]
    tr = state["trail"]
    if not tr or math.hypot(x - tr[-1][0], y - tr[-1][1]) > 0.03:  # 3cm 이상 움직이면 기록
        tr.append([round(x, 3), round(y, 3)])
        if len(tr) > TRAIL_MAX:
            del tr[: len(tr) - TRAIL_MAX]


def update_low(d):
    state["low"] = d
    state["low_t"] = time.time()


def lidar_obstacles(xy, rx, ry, yaw, reach=4.0):
    """LiDAR 점(월드 좌표)을 로봇 기준으로 돌려서 앞/왼쪽/뒤/오른쪽 가장 가까운 거리 계산."""
    if len(xy) == 0:
        return None
    dx, dy = xy[:, 0] - rx, xy[:, 1] - ry
    c, s = math.cos(yaw), math.sin(yaw)
    fx, fy = c * dx + s * dy, -s * dx + c * dy   # fx: 앞(+)/뒤(-), fy: 왼쪽(+)/오른쪽(-)
    half_w, half_l = 0.25, 0.40                   # 로봇 몸 폭/길이 절반 + 여유

    def nearest(mask, val):
        v = val[mask]
        v = v[v < reach]
        return round(float(v.min()), 2) if len(v) else None

    return [
        nearest((fx > 0.30) & (np.abs(fy) < half_w), fx),      # 앞
        nearest((fy > 0.15) & (np.abs(fx) < half_l), fy),      # 왼쪽
        nearest((fx < -0.30) & (np.abs(fy) < half_w), -fx),    # 뒤
        nearest((fy < -0.15) & (np.abs(fx) < half_l), -fy),    # 오른쪽
    ]


def update_lidar(pts):
    """pts: N x 3 (m, 월드 좌표)."""
    if len(pts) == 0:
        return
    s = state["sport"]
    z0 = s["position"][2] if s else 0.0
    band = pts[(pts[:, 2] > z0 - 0.15) & (pts[:, 2] < z0 + 0.6)]  # 몸 높이 근처(바닥/천장 제외)
    if s is not None and len(band):
        state["obstacle_lidar"] = lidar_obstacles(band[:, :2], s["position"][0], s["position"][1],
                                                  s["imu_state"]["rpy"][2])
    if len(band) > 4000:
        band = band[np.random.choice(len(band), 4000, replace=False)]
    state["lidar"] = np.round(band[:, :2], 2).tolist()
    state["lidar_t"] = time.time()


# ───────────────────────── 실제 로봇 연결 ─────────────────────────
def robot_thread(args):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    async def on_track(track):
        while True:
            try:
                frame = await track.recv()
            except Exception:  # 트랙 종료(연결 끊김) → 감시 루프가 재연결
                return
            try:  # 프레임 하나가 깨져도 수신은 계속
                set_frame(frame.to_ndarray(format="bgr24"))
            except Exception:
                continue

    def on_lidar(m):
        try:
            update_lidar(m["data"]["data"]["points"])
        except Exception as e:
            log(f"LiDAR 처리 오류: {e}")

    async def setup():
        while True:
            try:
                state["attempt"] += 1
                if state["phase"] != "reconnecting":
                    state["phase"] = "connecting"
                state["status"] = f"로봇({args.ip})에 연결 중..."
                log(f"로봇 {args.ip} 연결 시도... ({state['attempt']}번째)")
                conn = make_conn(args)
                try:
                    await asyncio.wait_for(conn.connect(), timeout=20)
                except BaseException:
                    try:  # 반쯤 열린 연결을 닫아 로봇 자리를 비움
                        await asyncio.wait_for(conn.disconnect(), timeout=5)
                    except BaseException:
                        pass
                    raise
                sub = conn.datachannel.pub_sub.subscribe
                sub(RTC_TOPIC["LF_SPORT_MOD_STATE"], lambda m: update_sport(m["data"]))
                sub(RTC_TOPIC["LOW_STATE"], lambda m: update_low(m["data"]))
                conn.video.switchVideoChannel(True)
                conn.video.add_track_callback(on_track)
                try:  # LiDAR 켜기 + 구독 (실패해도 영상/센서는 계속)
                    conn.datachannel.set_decoder(decoder_type="native")
                    await conn.datachannel.disableTrafficSaving(True)
                    conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")
                    sub("rt/utlidar/voxel_map_compressed", on_lidar)
                except Exception as e:
                    log(f"LiDAR 시작 실패(영상/센서는 계속): {e}")
                state.update(status="연결됨", connected=True, phase="connected", attempt=0, error=None)
                log("✅ 연결됨 — 센서/영상 수신 시작")

                # ── 감시: 연결이 닫히거나 데이터가 끊기면 정리 후 재연결 ──
                t_conn = time.time()
                while True:
                    await asyncio.sleep(2)
                    now = time.time()
                    pc_state = conn.pc.connectionState if conn.pc else "closed"
                    video_age = now - max(_latest_t[0], t_conn)
                    sport_age = now - max(state["sport_t"], t_conn)
                    if pc_state in ("closed", "failed"):
                        reason = f"WebRTC 연결 끊김({pc_state})"
                    elif video_age > 15 or sport_age > 15:
                        reason = f"데이터 끊김(영상 {video_age:.0f}초, 센서 {sport_age:.0f}초)"
                    else:
                        continue
                    break
                state.update(connected=False, phase="reconnecting", error=reason,
                             status=f"{reason} — 재연결 중...")
                log(f"⚠ {reason} → 재연결")
                try:
                    await asyncio.wait_for(conn.disconnect(), timeout=5)
                except BaseException:
                    pass
                await asyncio.sleep(3)
            except Exception as e:
                state.update(connected=False, phase="retrying",
                             error=ERROR_HINTS.get(type(e).__name__, f"{type(e).__name__}: {e}"),
                             status=f"연결 실패: {type(e).__name__} — 5초 후 재시도")
                log(f"❌ 연결 실패: {type(e).__name__}: {e} — 5초 후 재시도")
                await asyncio.sleep(5)

    loop.run_until_complete(setup())


# ───────────────────────── 데모 모드 (로봇 없이) ─────────────────────────
def _demo_room():
    """가상의 방: 6m x 5m 벽 + 상자 두 개 (월드 좌표, m)."""
    pts = []
    for z in np.arange(0.2, 0.8, 0.1):
        for x in np.arange(-3, 3, 0.03):
            pts += [(x, -2.5, z), (x, 2.5, z)]
        for y in np.arange(-2.5, 2.5, 0.03):
            pts += [(-3, y, z), (3, y, z)]
        for cxy, r in (((1.6, 1.1), 0.25), ((-1.2, -1.3), 0.3)):
            for t in np.arange(-r, r, 0.03):
                pts += [(cxy[0] + t, cxy[1] - r, z), (cxy[0] + t, cxy[1] + r, z),
                        (cxy[0] - r, cxy[1] + t, z), (cxy[0] + r, cxy[1] + t, z)]
    return np.array(pts)


def demo_thread():
    state.update(demo=True, phase="connecting", attempt=1, status="데모: 가상 로봇에 연결 중...")
    time.sleep(2)  # 연결 중 화면 확인용
    state.update(connected=True, phase="connected", attempt=0, status="연결됨 (데모)")
    log("✅ 데모 모드 — 가짜 데이터 생성 중")
    room = _demo_room()
    t0 = time.time()
    last_cam = last_lidar = 0.0
    while True:
        t = time.time() - t0
        w = 0.15                                        # 타원 경로를 천천히 돎
        x, y = 1.6 * math.cos(w * t), 1.0 * math.sin(w * t)
        vx, vy = -1.6 * w * math.sin(w * t), 1.0 * w * math.cos(w * t)
        yaw = math.atan2(vy, vx)
        update_sport({
            "position": [x, y, 0.30], "body_height": 0.32,
            "velocity": [vx, vy, 0.0], "yaw_speed": w,
            "imu_state": {"rpy": [0.01 * math.sin(t * 3), 0.02 * math.sin(t * 2), yaw]},
            "foot_force": [0, 0, 0, 0], "range_obstacle": [0, 0, 0, 0],  # Pro처럼 안 들어오는 값
            "mode": 0, "gait_type": 0,
        })
        battery = 24 - int(t / 15) % 20                 # 24%에서 15초마다 1%씩 → 20% 아래 경고 확인용
        temps = []
        for i in range(12):
            base = 36 + (i % 3) * 3 + 2 * math.sin(t / 20 + i)
            if i == 11:                                  # RL calf: 천천히 과열됐다 식음 (경고 확인용)
                base = 45 + 28 * max(0.0, math.sin(t / 40))
            temps.append(round(base))
        update_low({"bms_state": {"soc": battery}, "power_v": 27 + battery / 25,
                    "temperature_ntc1": 38, "motor_state": [{"temperature": v} for v in temps]})
        if t - last_cam > 1 / 15:
            last_cam = t
            img = np.full((540, 960, 3), (45, 38, 32), dtype=np.uint8)
            for k in range(12):                          # 고개 방향에 따라 흘러가는 기둥 (가짜 풍경)
                px = int((k * 160 - yaw * 300) % 1920) - 480
                cv2.rectangle(img, (px, 120), (px + 50, 460), (90 + k * 10, 120, 150), -1)
            cv2.rectangle(img, (0, 460), (960, 540), (40, 70, 50), -1)
            cv2.putText(img, "DEMO", (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.8, (255, 255, 255), 3)
            cv2.putText(img, f"{time.strftime('%H:%M:%S')}  x={x:+.2f} y={y:+.2f}", (30, 515),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (230, 230, 230), 2)
            set_frame(img)
        if t - last_lidar > 0.5:
            last_lidar = t
            update_lidar(room + np.random.normal(0, 0.01, room.shape))
        time.sleep(0.02)


# ───────────────────────── 녹화 (영상 mp4 + 센서 CSV) ─────────────────────────
class Recorder:
    FPS = 15

    def __init__(self):
        self.lock = threading.Lock()
        self.cur = None

    def status(self):
        c = self.cur
        return {"active": True, "name": c["name"], "elapsed": round(time.time() - c["t0"], 1)} if c else {"active": False}

    def start(self):
        with self.lock:
            if self.cur:
                return self.cur["name"]
            name = time.strftime("%Y%m%d_%H%M%S")
            d = os.path.join(REC_DIR, name)
            os.makedirs(d, exist_ok=True)
            self.cur = {"name": name, "dir": d, "t0": time.time(), "stop": threading.Event(), "done": threading.Event()}
            threading.Thread(target=self._run, args=(self.cur,), daemon=True).start()
            log(f"⏺ 녹화 시작: recordings/{name}")
            return name

    def stop(self):
        with self.lock:
            c, self.cur = self.cur, None
        if c:
            c["stop"].set()
            c["done"].wait(5)
            log(f"⏹ 녹화 종료: recordings/{c['name']}")
            return c["name"]

    def _run(self, c):
        vw = None
        f = open(os.path.join(c["dir"], "sensors.csv"), "w", newline="", encoding="utf-8-sig")
        w = csv.writer(f)
        w.writerow(["t_s", "x", "y", "yaw", "vx", "vy", "roll", "pitch", "height", "battery", "voltage",
                    "obst_front", "obst_left", "obst_back", "obst_right"]
                   + [f"temp_{leg}_{j}" for leg in LEGS for j in ("hip", "thigh", "calf")])
        try:
            n = 0
            while not c["stop"].is_set():
                tick = c["t0"] + n / self.FPS
                time.sleep(max(0.0, tick - time.time()))
                n += 1
                with _frame_lock:
                    jpg = _latest_jpeg[0]
                if jpg is not None:  # 매 틱마다 최신 프레임을 써서 재생 속도가 실제 시간과 같게
                    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                    if vw is None:
                        h, wd = img.shape[:2]
                        vw = cv2.VideoWriter(os.path.join(c["dir"], "video.mp4"),
                                             cv2.VideoWriter_fourcc(*"mp4v"), self.FPS, (wd, h))
                        size = (wd, h)
                    if (img.shape[1], img.shape[0]) != size:
                        img = cv2.resize(img, size)
                    vw.write(img)
                if n % 3 == 0:  # 센서는 5Hz
                    s, low = state["sport"], state["low"]
                    if s:
                        ob = state["obstacle_lidar"] or [None] * 4
                        temps = [m.get("temperature") for m in low.get("motor_state", [])[:12]] if low else []
                        w.writerow([round(time.time() - c["t0"], 2), *[round(v, 3) for v in s["position"][:2]],
                                    round(s["imu_state"]["rpy"][2], 3), *[round(v, 3) for v in s["velocity"][:2]],
                                    round(s["imu_state"]["rpy"][0], 3), round(s["imu_state"]["rpy"][1], 3),
                                    round(s["body_height"], 3),
                                    low["bms_state"]["soc"] if low else "", round(low["power_v"], 2) if low else "",
                                    *ob, *temps])
        finally:
            if vw is not None:
                vw.release()
            f.close()
            c["done"].set()


recorder = Recorder()


# ───────────────────────── 웹 API ─────────────────────────
@app.route("/")
def index():
    return send_from_directory("web", "index.html")


@app.route("/video")
def video():
    """새 프레임이 생기면 바로 밀어 보내는 스트림 (멀티파트 JPEG)."""
    def part(jpg):
        return b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"

    def gen():
        last_t, last_sent = -1.0, 0.0
        while True:
            with _frame_lock:
                jpg, t = _latest_jpeg[0], _latest_t[0]
            now = time.time()
            if jpg is not None and t != last_t:
                last_t, last_sent = t, now
                yield part(jpg)
            elif now - last_sent > 1.0:  # 새 프레임이 없어도 1초마다 보내서 연결이 살아 있음을 알림
                last_sent = now
                yield part(jpg if jpg is not None else PLACEHOLDER)
            time.sleep(0.01)

    r = Response(gen(), mimetype="multipart/x-mixed-replace; boundary=f")
    r.headers["Cache-Control"] = "no-store"
    r.headers["X-Accel-Buffering"] = "no"
    return r


@app.route("/frame")
def frame():
    """최신 카메라 사진 한 장."""
    with _frame_lock:
        jpg, t = _latest_jpeg[0], _latest_t[0]
    r = Response(jpg or PLACEHOLDER, mimetype="image/jpeg")
    r.headers["Cache-Control"] = "no-store"
    r.headers["X-Frame-Age"] = f"{time.time() - t:.2f}" if t else "-1"
    return r


@app.route("/snapshot", methods=["POST"])
def snapshot():
    """지금 카메라 화면을 captures/에 저장하고 내려받기 주소를 돌려줌."""
    with _frame_lock:
        jpg = _latest_jpeg[0]
    if jpg is None:
        return jsonify({"ok": False, "error": "아직 카메라 영상이 없어요"}), 409
    os.makedirs(CAP_DIR, exist_ok=True)
    name = f"photo_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
    with open(os.path.join(CAP_DIR, name), "wb") as f:
        f.write(jpg)
    return jsonify({"ok": True, "url": f"/files/captures/{name}", "name": name})


@app.route("/record/start", methods=["POST"])
def record_start():
    return jsonify({"ok": True, "name": recorder.start()})


@app.route("/record/stop", methods=["POST"])
def record_stop():
    return jsonify({"ok": True, "name": recorder.stop()})


@app.route("/recordings")
def recordings():
    out = []
    if os.path.isdir(REC_DIR):
        for name in sorted(os.listdir(REC_DIR), reverse=True)[:10]:
            d = os.path.join(REC_DIR, name)
            files = [{"name": fn, "url": f"/files/recordings/{name}/{fn}",
                      "kb": round(os.path.getsize(os.path.join(d, fn)) / 1024)}
                     for fn in ("video.mp4", "sensors.csv") if os.path.exists(os.path.join(d, fn))]
            out.append({"name": name, "files": files})
    return jsonify(out)


@app.route("/files/<kind>/<path:sub>")
def files(kind, sub):
    root = {"recordings": REC_DIR, "captures": CAP_DIR}.get(kind)
    if root is None:
        return "not found", 404
    return send_from_directory(root, sub, as_attachment=True)  # send_from_directory가 폴더 밖 경로를 막아 줌


@app.route("/trail/clear", methods=["POST"])
def trail_clear():
    state["trail"].clear()
    return jsonify({"ok": True})


@app.route("/data")
def data():
    s, low = state["sport"], state["low"]
    now = time.time()
    age = lambda t: round(now - t, 1) if t else None
    out = {
        "status": state["status"], "connected": state["connected"], "demo": state["demo"],
        "phase": state["phase"], "attempt": state["attempt"], "error": state["error"],
        "video_age": age(_latest_t[0]), "sport_age": age(state["sport_t"]),
        "lidar_age": age(state["lidar_t"]), "recording": recorder.status(),
        "obstacle_lidar": state["obstacle_lidar"],
    }
    if s is not None:
        out.update({
            "pos": s["position"][:2],
            "height": round(s["body_height"], 3),
            "vel": [round(v, 3) for v in s["velocity"]],
            "yaw_speed": round(s["yaw_speed"], 3),
            "rpy": [round(v, 3) for v in s["imu_state"]["rpy"]],
            "foot_force": s["foot_force"],
            "obstacle": s.get("range_obstacle"),
        })
    if low is not None:
        out["battery"] = low["bms_state"]["soc"]
        out["voltage"] = round(low["power_v"], 1)
        out["motors"] = [m.get("temperature") for m in low.get("motor_state", [])[:12]]
    return jsonify(out)


@app.route("/lidar")
def lidar():
    s = state["sport"]
    tr = state["trail"]
    step = max(1, len(tr) // 1000)  # 화면용으로 최대 ~1000점
    return jsonify({
        "points": state["lidar"],
        "age": round(time.time() - state["lidar_t"], 1) if state["lidar_t"] else None,
        "robot": s["position"][:2] if s else None,
        "yaw": s["imu_state"]["rpy"][2] if s else None,
        "trail": tr[::step] + (tr[-1:] if tr and (len(tr) - 1) % step else []),
        "trail_len": round(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(tr, tr[1:])), 2),
    })


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--ip", default=DEFAULT_IP)
    p.add_argument("--ap", action="store_true")
    p.add_argument("--key", default=DEFAULT_KEY)
    p.add_argument("--demo", action="store_true", help="로봇 없이 가짜 데이터로 실행")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--no-browser", action="store_true")
    args = p.parse_args()
    if not args.demo and not args.ap and not args.ip:
        p.error(".env에 UNITREE_ROBOT_IP를 적거나 --ip 192.168.x.x 를 지정하세요 (로봇 없이 보려면 --demo)")

    threading.Thread(target=demo_thread if args.demo else robot_thread,
                     args=() if args.demo else (args,), daemon=True).start()

    # Windows에서 'localhost'는 IPv6(::1)를 먼저 시도해서 요청마다 ~2초 지연됨
    # → IPv6에서도 바로 응답하도록 ::1 서버를 하나 더 띄움
    def serve_ipv6():
        try:
            from werkzeug.serving import make_server
            make_server("::1", args.port, app, threaded=True).serve_forever()
        except Exception as e:
            log(f"IPv6(::1) 서버 시작 실패 — 127.0.0.1 주소로 접속하세요: {e}")
    threading.Thread(target=serve_ipv6, daemon=True).start()

    url = f"http://127.0.0.1:{args.port}/"
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    print(f"대시보드: {url}  {'(데모 모드) ' if args.demo else ''}(종료: Ctrl+C)")
    # .env는 sensors.load_env()가 이미 읽음 → Flask가 python-dotenv로 또 읽지 않게 끔
    app.run(host="0.0.0.0", port=args.port, threaded=True, load_dotenv=False)
