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
import json
import logging
import re
import math
import os
import threading
import time
import webbrowser
from datetime import timedelta

import cv2
import numpy as np
from flask import Flask, Response, jsonify, redirect, request, send_from_directory, session

from follow import Follower
from mapstack import MapStack
from agent import RobotAgent
from robotslam import RobotSlam
import speech
from navigator import Navigator
from sensors import make_conn  # import 시 .env도 읽음
from unitree_webrtc_connect import RTC_TOPIC, SPORT_CMD
from unitree_webrtc_connect import webrtc_datachannel as _wdc

# 로봇 내장 SLAM의 점구름(rt/uslam/...)은 LiDAR 바이너리 경로로 오는데 형식이 달라서 라이브러리 디코더가
# KeyError('src_size')로 버림 → uslam 주제는 원본 바이트를 그대로 넘기게 고침 (점 하나 = uint16 x3, 범위 xmin~zmax)
_orig_lidar = _wdc.WebRTCDataChannel.deal_array_buffer_for_lidar


def _deal_lidar_keep_uslam(self, buffer):
    import struct
    hl, = struct.unpack_from("<I", buffer, 0)
    head = json.loads(bytes(buffer[8:8 + hl]).decode("utf-8"))
    if "uslam" in head.get("topic", ""):
        head["data"]["data"] = bytes(buffer[8 + hl:])
        return head
    return _orig_lidar(self, buffer)


_wdc.WebRTCDataChannel.deal_array_buffer_for_lidar = _deal_lidar_keep_uslam

# 라이브러리 버그: 파일 내려받기(request_static_file)가 첫 조각만 받고 끝남 → 마지막 조각까지 모은 뒤에 넘기게 고침
from unitree_webrtc_connect.msgs import future_resolver as _fr  # noqa: E402
from unitree_webrtc_connect.util import get_nested_field as _gnf  # noqa: E402


def _resolve_file_all_chunks(self, message):
    key = self.generate_message_key(message["type"], message.get("topic", ""),
                                    _gnf(message, "info", "req_uuid") or _gnf(message, "info", "uuid"))
    fi = _gnf(message, "info", "file")
    if fi and fi.get("enable_chunking"):
        part = fi.get("data") or ""
        self.chunk_data_storage.setdefault(key, []).append(part.encode() if isinstance(part, str) else part)
        if (fi.get("chunk_index") or 0) < (fi.get("total_chunk_num") or 0):
            return                                            # 아직 조각이 더 옴
        message["info"]["file"]["data"] = b"".join(self.chunk_data_storage.pop(key, []))
    for fut in self.pending_callbacks.pop(key, []):
        if fut and not fut.done():
            fut.set_result(message)


_fr.FutureResolver.run_resolve_for_topic_for_file = _resolve_file_all_chunks

# 접속 정보는 .env에서 읽음 (sensors.py의 load_env). 코드에 키를 적지 않음.
DEFAULT_IP = os.environ.get("UNITREE_ROBOT_IP")
DEFAULT_KEY = os.environ.get("UNITREE_AES_128_KEY")
PASSWORD = os.environ.get("DASHBOARD_PASSWORD", "")       # 비우면 잠금 없음
PAGES_ORIGIN = os.environ.get("DASHBOARD_PAGES_ORIGIN", "https://uyeonim.github.io")  # 고정 주소 입구 (GitHub Pages)

BASE = os.path.dirname(os.path.abspath(__file__))
REC_DIR = os.path.join(BASE, "recordings")   # 녹화(영상 + 센서 CSV)
CAP_DIR = os.path.join(BASE, "captures")     # 사진
ROUTE_DIR = os.path.join(BASE, "routes")     # 저장한 경로 (route_drive.py와 같은 형식)
DMAP_DIR = os.path.join(BASE, "maps", "dashboard")   # 대시보드에서 쌓은 지도
FRAME_W, JPEG_Q = 960, 75                    # 전송용 영상 크기/화질 (공개 주소에서도 가볍게)
TRAIL_MAX = 3000                             # 경로 점 최대 개수
LEGS = ["FR", "FL", "RR", "RL"]              # Go2 모터 순서: 다리별 (hip, thigh, calf)

state = {
    "sport": None, "sport_t": 0.0,
    "low": None, "low_t": 0.0,
    "lidar": [], "lidar_t": 0.0,      # 위에서 본 [x, y] 점 목록 (m, 월드 좌표)
    "obs": np.zeros((0, 2)),          # 길 찾기용 LiDAR 점 전체 (화면용보다 촘촘)
    "conn": None, "loop": None,       # 이동 명령을 보낼 로봇 연결
    "uwb": None, "uwb_t": 0.0,        # 따라가기 리모컨(UWB) 상태
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

mapst = MapStack()   # 지나간 곳의 LiDAR 점을 계속 쌓는 지도
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
    # GO2_HOME_URL: 공개 주소로 보던 사람이 대시보드가 꺼지면 돌아갈 고정 주소(데모)
    r = Response(f"window.GO2_MODE = 'live';\nwindow.GO2_HOME_URL = '{PAGES_ORIGIN}/go2-pro-dashboard/';\n",
                 mimetype="application/javascript")
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
    x, y, _ = mapst.correct((d["position"][0], d["position"][1], d["imu_state"]["rpy"][2]))  # 지도 좌표
    tr = state["trail"]
    if not tr or math.hypot(x - tr[-1][0], y - tr[-1][1]) > 0.03:  # 3cm 이상 움직이면 기록
        tr.append([round(x, 3), round(y, 3)])
        if len(tr) > TRAIL_MAX:
            del tr[: len(tr) - TRAIL_MAX]


def update_low(d):
    state["low"] = d
    state["low_t"] = time.time()


PROBE_TOPICS = [
    "rt/uslam/frontend/odom", "rt/uslam/frontend/cloud_world_ds", "rt/uslam/cloud_map", "rt/uslam/server_log",
    "rt/uslam/localization/odom", "rt/uslam/localization/cloud_world", "rt/uslam/navigation/global_path",
    "rt/utlidar/robot_pose", "rt/mapping/grid_map", "rt/lio_sam_ros2/mapping/odometry",
    "rt/qt_notice", "rt/pctoimage_local", "rt/utlidar/lidar_state",
]
probe = {}


def _shape(v, depth=0):
    """메시지 구조 요약 (값이 크면 길이만)"""
    if isinstance(v, dict):
        return {k: _shape(x, depth + 1) for k, x in list(v.items())[:25]} if depth < 4 else "{...}"
    if isinstance(v, (list, tuple)):
        return f"list[{len(v)}]" + (f" e.g. {_shape(v[0], depth + 1)}" if v and depth < 4 else "")
    if isinstance(v, (bytes, bytearray)):
        return f"bytes[{len(v)}]"
    if hasattr(v, "shape"):
        return f"array{tuple(v.shape)}"
    if isinstance(v, str):
        return v[:120]
    return v


uslam_log = []   # 로봇 내장 SLAM(uslam_server)의 응답·상태 문자열 (rt/uslam/server_log)


def probe_msg(tp, m):
    d = m.get("data") if isinstance(m, dict) else m
    if tp == "rt/uslam/server_log":
        txt = d.get("data") if isinstance(d, dict) else d
        if not uslam_log or uslam_log[-1][1] != str(txt)[:300]:      # 같은 줄 반복은 한 번만
            uslam_log.append([time.strftime("%H:%M:%S"), str(txt)[:300]])
        del uslam_log[:-500]
        rslam.on_log(txt)
    elif tp == "rt/uslam/localization/odom" and isinstance(d, dict):
        rslam.on_loc_odom(d)
    elif tp == "rt/uslam/frontend/odom" and isinstance(d, dict):
        rslam.on_map_odom(d)
    elif tp == "rt/uslam/navigation/global_path" and isinstance(d, dict):
        rslam.on_path(d)
    if tp == "rt/uslam/frontend/cloud_world_ds" and probe.get(tp, {}).get("n", 0) < 6:   # 분석용 샘플 몇 개 저장
        try:
            d = m["data"]
            os.makedirs(os.path.join(BASE, "logs", "uslam"), exist_ok=True)
            k = probe.get(tp, {}).get("n", 0)
            with open(os.path.join(BASE, "logs", "uslam", f"cloud_{k}.bin"), "wb") as f:
                f.write(d["data"])
            with open(os.path.join(BASE, "logs", "uslam", f"cloud_{k}.json"), "w") as f:
                json.dump({kk: vv for kk, vv in d.items() if kk != "data"}, f)
        except Exception as e:
            log(f"uslam 샘플 저장 실패: {e}")
    p = probe.setdefault(tp, {"n": 0, "first": time.time()})
    p["n"] += 1
    p["last"] = time.time()
    if p["n"] <= 3 or p["n"] % 20 == 0:
        try:
            p["sample"] = _shape(m.get("data") if isinstance(m, dict) else m)
        except Exception as e:
            p["sample"] = f"(요약 실패 {e})"


def update_uwb(d):
    """따라가기 리모컨(UWB 태그) 상태: 로봇 기준 리모컨 거리·방향 등"""
    state["uwb"] = d
    state["uwb_t"] = time.time()
    state["uwb_n"] = state.get("uwb_n", 0) + 1
    follower.feed_uwb(d)


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
    if not len(band):
        return
    raw = (s["position"][0], s["position"][1], s["imu_state"]["rpy"][2]) if s is not None else None
    # 스캔 정합으로 odom 오차를 보정한 "지도 좌표" 점 (+ 확률 지도 갱신)
    xy = mapst.process(band[:, :2], raw) if raw else mapst.correct_pts(band[:, :2].astype(np.float32))
    pose = mapst.correct(raw)
    if pose is not None:
        state["obstacle_lidar"] = lidar_obstacles(xy, pose[0], pose[1], pose[2])
    obs = xy
    if len(obs) > 30000:
        obs = obs[np.random.choice(len(obs), 30000, replace=False)]
    state["obs"] = obs.astype(np.float32)
    if len(xy) > 4000:
        xy = xy[np.random.choice(len(xy), 4000, replace=False)]
    state["lidar"] = np.round(xy, 2).tolist()
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
                sub(RTC_TOPIC["UWB_STATE"], lambda m: update_uwb(m["data"]))   # 따라가기 리모컨(UWB 태그)
                for tp in PROBE_TOPICS:   # 로봇 내장 SLAM 등: 어떤 데이터가 오는지 확인만 (읽기 전용)
                    sub(tp, lambda m, tp=tp: probe_msg(tp, m))
                conn.video.switchVideoChannel(True)
                conn.video.add_track_callback(on_track)
                try:  # LiDAR 켜기 + 구독 (실패해도 영상/센서는 계속)
                    conn.datachannel.set_decoder(decoder_type="native")
                    await conn.datachannel.disableTrafficSaving(True)
                    conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")
                    sub("rt/utlidar/voxel_map_compressed", on_lidar)
                except Exception as e:
                    log(f"LiDAR 시작 실패(영상/센서는 계속): {e}")
                state.update(status="연결됨", connected=True, phase="connected", attempt=0, error=None,
                             conn=conn, loop=loop)
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
                nav.stop("로봇 연결 끊김")
                follower.stop("로봇 연결 끊김")
                state.update(connected=False, phase="reconnecting", error=reason,
                             status=f"{reason} — 재연결 중...", conn=None)
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


DEMO_DRIFT = False   # --drift
sim = {"manual": False, "cmd": (0.0, 0.0, 0.0), "cmd_t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0,
       "vx": 0.0, "vy": 0.0, "vyaw": 0.0}


def demo_thread():
    state.update(demo=True, phase="connecting", attempt=1, status="데모: 가상 로봇에 연결 중...")
    time.sleep(2)  # 연결 중 화면 확인용
    state.update(connected=True, phase="connected", attempt=0, status="연결됨 (데모)")
    log("✅ 데모 모드 — 가짜 데이터 생성 중")
    room = _demo_room()
    t0 = time.time()
    last_cam = last_lidar = last_uwb = 0.0
    t_prev = time.time()
    while True:
        t = time.time() - t0
        dt, t_prev = time.time() - t_prev, time.time()
        w = 0.15
        if not sim["manual"]:                           # 평소: 타원 경로를 천천히 돎
            x, y = 1.6 * math.cos(w * t), 1.0 * math.sin(w * t)
            vx, vy = -1.6 * w * math.sin(w * t), 1.0 * w * math.cos(w * t)
            yaw = math.atan2(vy, vx)
            sim.update(x=x, y=y, yaw=yaw)
        else:                                           # 이동 명령을 받은 뒤: 명령대로 걷는 가짜 로봇
            cvx, cvy, cvyaw = sim["cmd"] if time.time() - sim["cmd_t"] < 0.5 else (0.0, 0.0, 0.0)
            a = min(1.0, dt / 0.3)                      # 실제 로봇처럼 0.3초쯤 늦게 반응
            sim["vx"] += (cvx - sim["vx"]) * a
            sim["vy"] += (cvy - sim["vy"]) * a
            sim["vyaw"] += (cvyaw - sim["vyaw"]) * a
            sim["yaw"] = (sim["yaw"] + sim["vyaw"] * dt + math.pi) % (2 * math.pi) - math.pi
            c, s_ = math.cos(sim["yaw"]), math.sin(sim["yaw"])
            sim["x"] += (sim["vx"] * c - sim["vy"] * s_) * dt
            sim["y"] += (sim["vx"] * s_ + sim["vy"] * c) * dt
            x, y, yaw = sim["x"], sim["y"], sim["yaw"]
            vx, vy, w = sim["vx"] * c - sim["vy"] * s_, sim["vx"] * s_ + sim["vy"] * c, sim["vyaw"]
        # --drift: 실제 로봇처럼 odom이 조금씩 틀어지게 (위치·LiDAR 둘 다 같은 오차) → 스캔 정합 시험용
        dth, dtx, dty = (0.0015 * t, 0.003 * t, -0.002 * t) if DEMO_DRIFT else (0.0, 0.0, 0.0)
        cd, sd = math.cos(dth), math.sin(dth)
        ox, oy = cd * x - sd * y + dtx, sd * x + cd * y + dty
        update_sport({
            "position": [ox, oy, 0.30], "body_height": 0.32,
            "velocity": [vx, vy, 0.0], "yaw_speed": w,
            "imu_state": {"rpy": [0.01 * math.sin(t * 3), 0.02 * math.sin(t * 2), yaw + dth]},
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
        if t - last_uwb > 0.1:                          # 가짜 리모컨: 방 안을 천천히 걷는 사람
            last_uwb = t
            hx, hy = 1.8 * math.cos(0.08 * t + 2.0), 1.4 * math.sin(0.16 * t + 2.0)
            bearing = math.atan2(hy - y, hx - x) - yaw + np.random.normal(0, 0.08)
            if np.random.rand() < 0.05:                 # 실제처럼 가끔 앞뒤가 뒤집혀 튀는 값
                bearing += math.pi
            update_uwb({"distance_est": math.hypot(hx - x, hy - y) + np.random.normal(0, 0.05),
                        "orientation_est": (bearing + math.pi) % (2 * math.pi) - math.pi,   # 로봇 기준 리모컨 방향
                        "tag_yaw": np.random.uniform(-3, 3), "base_yaw": yaw, "buttons": 0, "joystick": [0, 0]})
        if t - last_lidar > 0.5:
            last_lidar = t
            pts = room + np.random.normal(0, 0.01, room.shape)
            if DEMO_DRIFT:   # 잡음 시험: 걸어다니는 사람 + 아무 데나 튀는 점
                px_, py_ = -2.0 + 4.0 * ((t / 20) % 1), -1.8
                ang = np.random.uniform(0, 2 * np.pi, 120)
                person = np.stack([px_ + 0.2 * np.cos(ang), py_ + 0.2 * np.sin(ang), np.full(120, 0.5)], 1)
                specks = np.stack([np.random.uniform(-3, 3, 25), np.random.uniform(-2.5, 2.5, 25), np.full(25, 0.4)], 1)
                pts = np.vstack([pts, person, specks])
            pts[:, :2] = pts[:, :2] @ np.array([[cd, sd], [-sd, cd]]) + [dtx, dty]   # odom 좌표로 (드리프트)
            update_lidar(pts)
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


# ───────────────────────── 자율 이동 (지도 클릭 / 경로) ─────────────────────────
def _pose():
    """지도 좌표 로봇 자세 (스캔 정합 보정 반영) + 받은 시각"""
    s = state["sport"]
    if s is None:
        return None
    return (*mapst.correct((s["position"][0], s["position"][1], s["imu_state"]["rpy"][2])), state["sport_t"])


def _sport_cmd(api, param=None):
    """로봇에 스포츠 명령 보내기 (응답은 기다리지 않음). 데모에서는 가짜 로봇으로."""
    if state["demo"]:
        if api == "Move":
            sim.update(manual=True, cmd=(param["x"], param.get("y", 0), param["z"]), cmd_t=time.time())
        else:
            sim.update(cmd=(0.0, 0.0, 0.0), cmd_t=time.time())
        return
    conn, loop = state["conn"], state["loop"]
    if conn is None or loop is None:
        return
    opts = {"api_id": SPORT_CMD[api]}
    if param is not None:
        opts["parameter"] = param

    async def send():
        try:
            await asyncio.wait_for(conn.datachannel.pub_sub.publish_request_new(RTC_TOPIC["SPORT_MOD"], opts), 1.0)
        except Exception:
            pass
    asyncio.run_coroutine_threadsafe(send(), loop)


def _all_obstacles():
    """길 찾기용: 지금 보이는 점 + 쌓은 지도"""
    stack = mapst.occupied_xy()
    obs = np.vstack([state["obs"], stack]) if len(stack) else state["obs"]
    return obs, state["lidar_t"]


nav = Navigator(
    get_pose=_pose,
    get_obstacles=_all_obstacles,
    get_live_obstacles=lambda: (state["obs"], state["lidar_t"]),
    send_move=lambda vx, vyaw: _sport_cmd("Move", {"x": round(vx, 3), "y": 0, "z": round(vyaw, 3)}),
    send_stop=lambda: _sport_cmd("StopMove"),
    log=log,
)


follower = Follower(
    get_pose=_pose,
    get_live_obstacles=lambda: (state["obs"], state["lidar_t"]),
    get_jpeg=lambda: _latest_jpeg[0],
    send_move=lambda vx, vyaw: _sport_cmd("Move", {"x": round(vx, 3), "y": 0, "z": round(vyaw, 3)}),
    send_stop=lambda: _sport_cmd("StopMove"),
    send_action=lambda name: _sport_cmd(name),
    log=log,
    get_obstacles=_all_obstacles,
)


def can_drive():
    """로봇을 움직이는 기능은 이 노트북 화면(127.0.0.1)에서만. 공개 주소(Tailscale/Cloudflare)로 온 요청은 막음."""
    if request.remote_addr not in ("127.0.0.1", "::1", "::ffff:127.0.0.1"):
        return False
    proxied = ("X-Forwarded-For", "X-Forwarded-Host", "X-Forwarded-Proto", "Forwarded",
               "Tailscale-Funnel-Request", "Tailscale-User-Login", "Cf-Connecting-Ip", "Cf-Ray")
    if any(h in request.headers for h in proxied):
        return False
    host = request.host.rsplit(":", 1)[0] if not request.host.startswith("[") else request.host.split("]")[0] + "]"
    return host in ("127.0.0.1", "localhost", "[::1]")


def _drive_guard():
    if not can_drive():
        return jsonify({"ok": False, "error": "로봇 이동은 노트북 화면(127.0.0.1)에서만 할 수 있어요"}), 403
    if not state["connected"] or _pose() is None:
        return jsonify({"ok": False, "error": "로봇이 연결돼 있지 않아요"}), 409
    return None


def _route_path(name):
    if not re.fullmatch(r"[\w\-가-힣]{1,40}", name or ""):
        return None
    return os.path.join(ROUTE_DIR, name + ".json")


def _load_route(name):
    p = _route_path(name)
    if not p or not os.path.exists(p):
        return None
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def _local_to_world(pts, pose):
    x0, y0, yaw0 = pose[0], pose[1], pose[2]
    c, s = math.cos(yaw0), math.sin(yaw0)
    return [(x0 + c * lx - s * ly, y0 + s * lx + c * ly) for lx, ly in pts]


@app.route("/nav/goto", methods=["POST"])
def nav_goto():
    bad = _drive_guard()
    if bad:
        return bad
    j = request.get_json(force=True)
    yaw = j.get("yaw")
    follower.stop("클릭 이동")
    if rslam.busy():
        rslam.stop()
    nav.start([(float(j["x"]), float(j["y"]))], name="클릭 이동", speed=j.get("speed"), final_yaw=yaw)
    log(f"🎯 클릭 이동: ({j['x']:.2f}, {j['y']:.2f})" + (f" 방향 {math.degrees(yaw):.0f}°" if yaw is not None else ""))
    return jsonify({"ok": True})


# 방향키 조종: 브라우저가 키를 누르는 동안 0.1초마다 보냄. 0.35초 넘게 안 오면 자동 정지.
teleop = {"t": 0.0, "active": False}


@app.route("/nav/teleop", methods=["POST"])
def nav_teleop():
    bad = _drive_guard()
    if bad:
        return bad
    j = request.get_json(force=True)
    vx = max(-0.5, min(0.6, float(j.get("vx", 0))))
    vy = max(-0.4, min(0.4, float(j.get("vy", 0))))
    vyaw = max(-1.0, min(1.0, float(j.get("vyaw", 0))))
    if nav.active:
        nav.stop("방향키 조종", quiet=True)
    if follower.active:
        follower.stop("방향키 조종")
    if rslam.busy():
        rslam.stop()
    if vx == vy == vyaw == 0:
        teleop["active"] = False
        _sport_cmd("StopMove")
    else:
        teleop.update(t=time.time(), active=True)
        _sport_cmd("Move", {"x": round(vx, 3), "y": round(vy, 3), "z": round(vyaw, 3)})
    return jsonify({"ok": True})


def teleop_watchdog():
    while True:
        time.sleep(0.1)
        if teleop["active"] and time.time() - teleop["t"] > 0.35:   # 키 신호가 끊김(창 닫힘/네트워크) → 정지
            teleop["active"] = False
            _sport_cmd("StopMove")


@app.route("/nav/run", methods=["POST"])
def nav_run():
    bad = _drive_guard()
    if bad:
        return bad
    j = request.get_json(force=True)
    r = _load_route(j.get("name"))
    if not r:
        return jsonify({"ok": False, "error": "경로를 찾을 수 없어요"}), 404
    goals = _local_to_world(r["points"], _pose())     # 지금 로봇 자리·방향을 경로의 출발점으로
    follower.stop("경로 주행")
    if rslam.busy():
        rslam.stop()
    nav.start(goals, loops=j.get("loops", 1), name=r["name"], speed=j.get("speed"))
    log(f"▶ 경로 주행: {r['name']} ({len(goals)}개 지점, {j.get('loops', 1)}바퀴)")
    return jsonify({"ok": True})


@app.route("/nav/stop", methods=["POST"])
def nav_stop():
    agent.stop()              # 말로 시키기 중이면 다음 단계도 멈춤
    nav.stop("정지 버튼")      # 정지는 누구든 (공개 주소에서도) 누를 수 있게
    follower.stop("정지 버튼")
    if rslam.busy():
        rslam.stop()
    return jsonify({"ok": True})


@app.route("/follow/<action>", methods=["POST"])
def follow_action(action):
    """1번 기능: 리모컨 든 사람 따라가기 (+ 바라보면 하트)"""
    if action == "stop":
        follower.stop("정지 버튼")
        return jsonify({"ok": True})
    bad = _drive_guard()
    if bad:
        return bad
    j = request.get_json(silent=True) or {}
    if action == "start":
        nav.stop("따라가기 시작", quiet=True)
        if rslam.busy():
            rslam.stop()
        follower.start(dist_goal=j.get("dist"), heart=j.get("heart"), speed=j.get("speed"))
    elif action == "set":
        follower.set(dist_goal=j.get("dist"), heart=j.get("heart"), speed=j.get("speed"))
    else:
        return jsonify({"ok": False, "error": "unknown"}), 404
    return jsonify({"ok": True})


@app.route("/routes")
def routes_list():
    out = []
    if os.path.isdir(ROUTE_DIR):
        for fn in sorted(os.listdir(ROUTE_DIR)):
            if fn.endswith(".json"):
                try:
                    r = _load_route(fn[:-5])
                    pts = r["points"]
                    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip([(0, 0)] + pts, pts))
                    out.append({"name": r["name"], "points": pts, "length": round(length, 1),
                                "created": r.get("created", "")})
                except Exception:
                    continue
    return jsonify(out)


@app.route("/routes/save", methods=["POST"])
def routes_save():
    if not can_drive():
        return jsonify({"ok": False, "error": "경로 저장은 노트북 화면에서만 할 수 있어요"}), 403
    j = request.get_json(force=True)
    p = _route_path(j.get("name"))
    if not p:
        return jsonify({"ok": False, "error": "이름은 한글·영어·숫자·_·- 로 40자까지"}), 400
    ax, ay, ayaw = j["anchor"]
    c, s = math.cos(ayaw), math.sin(ayaw)
    pts = []
    for wx, wy in j["points"]:                         # 월드 → 출발 기준(앞 x, 왼쪽 y)
        dx, dy = wx - ax, wy - ay
        pts.append([round(c * dx + s * dy, 3), round(-s * dx + c * dy, 3)])
    os.makedirs(ROUTE_DIR, exist_ok=True)
    with open(p, "w", encoding="utf-8") as f:
        json.dump({"name": j["name"], "frame": "start: x=forward, y=left (m)",
                   "created": time.strftime("%Y-%m-%dT%H:%M:%S"), "points": pts}, f, ensure_ascii=False, indent=1)
    log(f"💾 경로 저장: routes/{j['name']}.json ({len(pts)}개 지점)")
    return jsonify({"ok": True})


@app.route("/map/cells")
def map_cells():
    return jsonify(mapst.cells())


def _dmap_path(name):
    if not re.fullmatch(r"[\w\-가-힣]{1,40}", name or ""):
        return None
    return os.path.join(DMAP_DIR, name)


@app.route("/map/list")
def map_list():
    out = []
    if os.path.isdir(DMAP_DIR):
        for fn in sorted(os.listdir(DMAP_DIR)):
            if fn.endswith(".json"):
                n = fn[:-5]
                out.append({"name": n, "png": f"/files/dmaps/{n}.png" if os.path.exists(os.path.join(DMAP_DIR, n + ".png")) else None})
    return jsonify(out)


@app.route("/map/<action>", methods=["POST"])
def map_action(action):
    if not can_drive():
        return jsonify({"ok": False, "error": "노트북 화면에서만 할 수 있어요"}), 403
    j = request.get_json(silent=True) or {}
    if action == "clear":
        mapst.clear(home=_pose()[:3] if _pose() else None)
        return jsonify({"ok": True})
    if action == "toggle":
        mapst.enabled = not mapst.enabled
        return jsonify({"ok": True, "enabled": mapst.enabled})
    p = _dmap_path(j.get("name"))
    if not p:
        return jsonify({"ok": False, "error": "이름은 한글·영어·숫자·_·- 로 40자까지"}), 400
    if action == "save":
        ok, msg = mapst.save(p, j["name"])
        if ok:
            log(f"💾 지도 저장: maps/dashboard/{j['name']} ({msg})")
        return jsonify({"ok": ok, "error": None if ok else msg})
    if action == "load":
        if not os.path.exists(p + ".json"):
            return jsonify({"ok": False, "error": "지도를 찾을 수 없어요"}), 404
        if _pose() is None:
            return jsonify({"ok": False, "error": "로봇 위치를 아직 몰라요"}), 409
        n = mapst.load(p, _pose()[:3])
        log(f"📂 지도 불러옴: {j['name']} ({n}칸, 지금 자리를 출발점으로)")
        return jsonify({"ok": True})
    if action == "delete":
        for ext in (".json", ".png"):
            if os.path.exists(p + ext):
                os.remove(p + ext)
        return jsonify({"ok": True})
    return jsonify({"ok": False, "error": "unknown"}), 404


@app.route("/routes/delete", methods=["POST"])
def routes_delete():
    if not can_drive():
        return jsonify({"ok": False, "error": "노트북 화면에서만 할 수 있어요"}), 403
    p = _route_path(request.get_json(force=True).get("name"))
    if p and os.path.exists(p):
        os.remove(p)
    return jsonify({"ok": True})


# ───────────────────────── 웹 API ─────────────────────────
@app.route("/")
def index():
    return send_from_directory("web", "index.html")


def is_remote():
    """공개 주소(Tailscale Funnel/Cloudflare)로 들어온 요청인지. 공개 경로는 대역폭이 작아서(초당 ~50KB) 가볍게 보냄."""
    if request.remote_addr not in ("127.0.0.1", "::1", "::ffff:127.0.0.1"):
        return True
    return any(h in request.headers for h in ("X-Forwarded-For", "Forwarded", "Cf-Connecting-Ip"))


REMOTE_W, REMOTE_Q, REMOTE_FPS = 480, 45, 4      # 원격 화면용: 작게·낮은 화질·초당 4장 (~15KB/장)
_low = {"t": None, "jpg": None}
_low_lock = threading.Lock()


def low_jpeg():
    """최신 프레임의 가벼운 버전 (프레임마다 한 번만 다시 인코딩)."""
    with _frame_lock:
        jpg, t = _latest_jpeg[0], _latest_t[0]
    if jpg is None:
        return None, t
    with _low_lock:
        if _low["t"] != t:
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            h, w = img.shape[:2]
            img = cv2.resize(img, (REMOTE_W, int(h * REMOTE_W / w)), interpolation=cv2.INTER_AREA)
            _low.update(t=t, jpg=cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, REMOTE_Q])[1].tobytes())
        return _low["jpg"], t


@app.route("/video")
def video():
    """새 프레임이 생기면 바로 밀어 보내는 스트림 (멀티파트 JPEG). 원격 화면은 작고 느리게."""
    def part(jpg):
        return b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"

    remote = is_remote()

    def gen():
        last_t, last_sent = -1.0, 0.0
        while True:
            if remote:
                jpg, t = low_jpeg()
            else:
                with _frame_lock:
                    jpg, t = _latest_jpeg[0], _latest_t[0]
            now = time.time()
            if remote and now - last_sent < 1 / REMOTE_FPS:
                time.sleep(0.02)
                continue
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
    if is_remote():
        jpg, t = low_jpeg()
    else:
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
    root = {"recordings": REC_DIR, "captures": CAP_DIR, "dmaps": DMAP_DIR}.get(kind)
    if root is None:
        return "not found", 404
    return send_from_directory(root, sub, as_attachment=True)  # send_from_directory가 폴더 밖 경로를 막아 줌


@app.route("/probe")
def probe_view():
    now = time.time()
    return jsonify({tp: {"n": p["n"], "hz": round(p["n"] / max(1e-3, p["last"] - p["first"]), 1) if p["n"] > 1 else None,
                         "age": round(now - p["last"], 1), "sample": p.get("sample")}
                    for tp, p in probe.items()} | {"_not_received": [t for t in PROBE_TOPICS if t not in probe]})


USLAM_SAFE = ("mapping/", "localization/", "common/", "autocharge/stop")   # 로봇이 스스로 걷지 않는 명령만 (navigation/patrol은 나중에)


@app.route("/uslam/cmd", methods=["POST"])
def uslam_cmd():
    """로봇 내장 SLAM에 문자열 명령 (예: mapping/get_status, mapping/start, mapping/stop). docs: unitree_ui/docs/slam.md"""
    if not can_drive():
        return jsonify({"ok": False, "error": "노트북 화면에서만 할 수 있어요"}), 403
    cmd = str((request.get_json(silent=True) or {}).get("cmd", "")).strip()
    if not cmd.startswith(USLAM_SAFE) and not cmd.endswith("/get_status"):
        return jsonify({"ok": False, "error": "아직 허용하지 않은 명령이에요"}), 400
    if not _uslam_send(cmd):
        return jsonify({"ok": False, "error": "로봇이 연결돼 있지 않아요"}), 409
    return jsonify({"ok": True})


def _uslam_send(cmd):
    conn, loop = state["conn"], state["loop"]
    if conn is None or loop is None:
        return False
    loop.call_soon_threadsafe(conn.datachannel.pub_sub.publish_without_callback, "rt/uslam/client_command", cmd)
    uslam_log.append([time.strftime("%H:%M:%S"), f">> {cmd}"])
    log(f"🗺 내장 SLAM 명령: {cmd}")
    return True


ROBOT_MAP_DIR = os.path.join(BASE, "maps", "robot")   # 로봇 내장 SLAM 지도(앱에서 딴 지도 등)를 내려받는 곳
rslam = RobotSlam(send_cmd=_uslam_send, map_dir=ROBOT_MAP_DIR, log=log)
follower.rnav = rslam      # 로봇 지도 위치를 알면 따라가기도 로봇 내장 길찾기로


def _rslam_guard():
    bad = _drive_guard()
    if bad:
        return bad
    if not rslam.meta:
        return jsonify({"ok": False, "error": "로봇 지도를 먼저 받아 주세요"}), 409
    return None


def _xyyaw():
    j = request.get_json(force=True)
    return float(j["x"]), float(j["y"]), float(j.get("yaw") or 0.0), j


@app.route("/rslam/map.png")
def rslam_map_png():
    if not rslam.png:
        return "no map", 404
    r = Response(rslam.png, mimetype="image/png")
    r.headers["Cache-Control"] = "no-store"
    return r


@app.route("/rslam/refresh_map", methods=["POST"])
def rslam_refresh_map():
    """로봇에서 2D 지도(map.pgm, map.txt)를 새로 받아 화면용으로 준비"""
    if not can_drive():
        return jsonify({"ok": False, "error": "노트북 화면에서만 할 수 있어요"}), 403
    for name in ("map.txt", "map.pgm"):
        ok, res = _fetch_robot_file(name)
        if not ok:
            return jsonify({"ok": False, "error": res}), 502
    rslam.on_fetched_pgm()
    return jsonify({"ok": rslam.load_map()})


def _push_robot_file(name, data):
    """로봇 지도 칸에 파일 올리기 (unitree_ui pushFile과 같은 방식: 30KB 조각마다 'ok' 확인)"""
    import base64
    conn, loop = state["conn"], state["loop"]
    if conn is None or loop is None:
        return False, "로봇이 연결돼 있지 않아요"
    b64 = base64.b64encode(data).decode()
    size = 30 * 1024
    chunks = [b64[i:i + size] for i in range(0, len(b64), size)]

    async def push():
        ps = conn.datachannel.pub_sub
        for g, ch in enumerate(chunks):
            if g and g % 5 == 0:
                await asyncio.sleep(0.5)
            msg = {"req_type": "push_static_file", "req_uuid": f"upload_req_{int(time.time() * 1000) % 2**31}_{g}",
                   "related_bussiness": "uslam_final_pcd", "file_md5": "null", "file_path": name,
                   "file_size_after_b64": len(b64),
                   "file": {"chunk_index": g + 1, "total_chunk_num": len(chunks), "chunk_data": ch,
                            "chunk_data_size": len(ch)}}
            ack = await asyncio.wait_for(ps.publish("", msg, "rtc_inner_req"), 10)
            st = (ack.get("info") or {}).get("file_status")
            if st != "ok":
                raise RuntimeError(f"조각 {g + 1}/{len(chunks)} 실패 ({st})")
        return True
    try:
        asyncio.run_coroutine_threadsafe(push(), loop).result(30 + 12 * len(chunks))
    except Exception as e:
        return False, f"올리기 실패: {e}"
    log(f"📤 로봇에 올림: {name} ({len(data)} bytes, {len(chunks)}조각)")
    return True, len(data)


@app.route("/rslam/nogo", methods=["GET", "POST"])
def rslam_nogo():
    """가지 말 곳 목록 (지도 좌표 사각형). POST로 저장만 (로봇 적용은 /rslam/nogo/apply)"""
    if request.method == "POST":
        if not can_drive():
            return jsonify({"ok": False, "error": "노트북 화면에서만 할 수 있어요"}), 403
        rslam.save_nogo(zones=(request.get_json(force=True) or {}).get("zones", []))
    return jsonify({"ok": True, "zones": rslam.nogo, "applied": rslam.nogo_applied})


@app.route("/rslam/nogo/apply", methods=["POST"])
def rslam_nogo_apply():
    """가지 말 곳을 칠한 2D 지도를 로봇에 올리고 다시 불러오게 함"""
    bad = _rslam_guard()
    if bad:
        return bad
    if rslam.busy():
        return jsonify({"ok": False, "error": "로봇이 이동 중이에요. 멈춘 뒤에 적용하세요"}), 409
    data = rslam.edited_pgm()
    ok, res = _push_robot_file("map.pgm", data)
    if not ok:
        return jsonify({"ok": False, "error": res}), 502
    rslam.after_upload(data)
    # 지도 다시 불러오기: 같은 id로 set_map_id → 다음 navigation/start 때 새 2D 지도로 길 찾음
    _uslam_send("common/get_map_id")
    for _ in range(20):
        time.sleep(0.1)
        if rslam.map_id:
            break
    if rslam.map_id and rslam.map_id != "none":
        _uslam_send(f"common/set_map_id/{rslam.map_id}")
    _uslam_send("navigation/stop")
    return jsonify({"ok": True, "zones": len(rslam.nogo)})


@app.route("/rslam/localize", methods=["POST"])
def rslam_localize():
    """지도 위 로봇 위치 알려주기 → 로봇 내장 위치 찾기 시작 (로봇은 안 움직임)"""
    bad = _rslam_guard()
    if bad:
        return bad
    x, y, yaw, _ = _xyyaw()
    threading.Thread(target=rslam.localize, args=(x, y, yaw), daemon=True).start()
    return jsonify({"ok": True})


@app.route("/rslam/goto", methods=["POST"])
def rslam_goto():
    """로봇 내장 내비게이션으로 목표 지점 이동"""
    bad = _rslam_guard()
    if bad:
        return bad
    if not rslam.localized:
        return jsonify({"ok": False, "error": "먼저 📍 위치 지정으로 로봇 위치를 찾아 주세요"}), 409
    x, y, yaw, _ = _xyyaw()
    nav.stop("로봇 내비게이션", quiet=True)
    follower.stop("로봇 내비게이션")
    threading.Thread(target=rslam.goto, args=(x, y, yaw), daemon=True).start()
    return jsonify({"ok": True})


@app.route("/rslam/patrol", methods=["POST"])
def rslam_patrol():
    """로봇 내장 순찰: 지점들을 차례로 (loops 바퀴, -1 = 계속)"""
    bad = _rslam_guard()
    if bad:
        return bad
    if not rslam.localized:
        return jsonify({"ok": False, "error": "먼저 📍 위치 지정으로 로봇 위치를 찾아 주세요"}), 409
    j = request.get_json(force=True)
    pts = [(float(p[0]), float(p[1]), float(p[2])) for p in j.get("points", [])]
    if len(pts) < 2:
        return jsonify({"ok": False, "error": "순찰 지점을 2개 이상 찍어 주세요"}), 400
    nav.stop("로봇 순찰", quiet=True)
    follower.stop("로봇 순찰")
    threading.Thread(target=rslam.patrol, args=(pts, int(j.get("loops", 1))), daemon=True).start()
    return jsonify({"ok": True})


@app.route("/rslam/stop", methods=["POST"])
def rslam_stop():
    agent.stop()
    rslam.stop()          # 정지는 누구든
    return jsonify({"ok": True})


# ───────────────────────── 말로 시키기 (LLM 에이전트) ─────────────────────────
def _stop_all():
    nav.stop("정지", quiet=True)
    follower.stop("정지")
    if rslam.busy():
        rslam.stop()


def _agent_status():
    low, s = state["low"], rslam.status()
    return {"battery_pct": low["bms_state"]["soc"] if low else None, "robot_connected": state["connected"],
            "map_localized": s["localized"], "map_pose": [round(v, 2) for v in s["pose"]] if s["pose"] else None,
            "robot_navigating": s["nav_active"], "nav_state": s["nav_state"], "patrolling": s["patrol_active"],
            "dashboard_nav": nav.status()["msg"] if nav.active else None,
            "following": follower.active, "remote_distance_m": (follower.status()["remote"] or [None])[0]}


def _agent_goto_map(x, y, yaw):
    if not state["connected"]:
        return "로봇이 연결돼 있지 않아요"
    if not rslam.localized:
        return "로봇 지도 위 위치를 몰라요 (로봇 지도 화면에서 📍 위치 지정 필요)"
    nav.stop("에이전트", quiet=True)
    follower.stop("에이전트")
    rslam.goto(x, y, yaw)
    return None


def _agent_nav_relative(fwd, left, turn, cancel):
    """지금 자리 기준 상대 이동: 로봇 지도 위치를 알면 내장 내비게이션, 아니면 대시보드 길 찾기"""
    if rslam.localized and rslam.pose:
        x, y, yaw = rslam.pose[:3]
        gx, gy = x + fwd * math.cos(yaw) - left * math.sin(yaw), y + fwd * math.sin(yaw) + left * math.cos(yaw)
        err = _agent_goto_map(gx, gy, yaw + turn)
        return f"실패: {err}" if err else agent._wait_nav(timeout=60)
    p = _pose()
    if p is None or not state["connected"]:
        return "실패: 로봇 위치를 몰라요"
    x, y, yaw = p[:3]
    gx, gy = x + fwd * math.cos(yaw) - left * math.sin(yaw), y + fwd * math.sin(yaw) + left * math.cos(yaw)
    follower.stop("에이전트")
    nav.start([(gx, gy)], name="말로 이동", final_yaw=yaw + turn)
    t0 = time.time()
    while nav.active and time.time() - t0 < 60:
        if cancel.is_set():
            nav.stop("에이전트 취소")
            return "취소됨"
        time.sleep(0.3)
    return nav.status()["msg"]


def _agent_follow(on):
    if not on:
        follower.stop("에이전트")
        return "따라가기를 껐어요."
    r = follower.remote()
    if r is None or time.time() - r[2] > 1.0:
        return "실패: 리모컨 신호가 없어요 (리모컨을 켜고 로봇 근처에서 들어 주세요)."
    nav.stop("따라가기", quiet=True)
    if rslam.busy():
        rslam.stop()
    follower.start()
    return f"따라가기 시작 (리모컨까지 {r[0]:.1f}m)."


def _agent_patrol(pts, loops):
    if not rslam.localized:
        return "로봇 지도 위 위치를 몰라요"
    if len(pts) < 2:
        return "순찰 장소가 2개 이상 필요해요"
    nav.stop("순찰", quiet=True)
    follower.stop("순찰")
    threading.Thread(target=rslam.patrol, args=([tuple(p) for p in pts], loops), daemon=True).start()
    return None


_say_lock = threading.Lock()


def robot_say(text):
    """로봇 스피커로 말하기 (끝까지 말할 때까지 기다림). 성공하면 None, 아니면 오류 문자열."""
    if not _say_lock.acquire(blocking=False):
        return "아직 이전 말을 하는 중이에요"
    try:
        try:
            wav, sec = speech.tts_wav(text)
        except Exception as e:
            return f"음성 만들기 실패: {e}"
        if state["demo"]:
            log(f"🗣 (데모) 로봇이 말함: {text}")
            time.sleep(sec)
            return None
        conn, loop = state["conn"], state["loop"]
        if conn is None or loop is None or not state["connected"]:
            return "로봇이 연결돼 있지 않아요"
        t0 = time.time()
        try:
            n = asyncio.run_coroutine_threadsafe(
                speech.play_on_robot(conn.datachannel.pub_sub, wav, sec, log=log), loop).result(60 + sec)
        except Exception as e:
            return f"로봇 스피커로 보내기 실패: {e!r}"
        log(f"🗣 로봇이 말함 ({sec:.1f}초, {n}조각, {time.time() - t0:.1f}초 걸림): {text}")
        return None
    finally:
        _say_lock.release()


@app.route("/speak", methods=["POST"])
def speak():
    if not can_drive():
        return jsonify({"ok": False, "error": "노트북 화면에서만 할 수 있어요"}), 403
    text = str((request.get_json(silent=True) or {}).get("text", "")).strip()[:speech.MAX_CHARS]
    if not text:
        return jsonify({"ok": False, "error": "말할 내용을 입력해 주세요"}), 400
    if _say_lock.locked():
        return jsonify({"ok": False, "error": "아직 이전 말을 하는 중이에요"}), 409
    threading.Thread(target=robot_say, args=(text,), daemon=True).start()
    return jsonify({"ok": True})


agent = RobotAgent(robot={
    "say": robot_say,
    "status": _agent_status, "pose_map": lambda: rslam.pose[:3] if rslam.localized and rslam.pose else None,
    "goto_map": _agent_goto_map, "rslam_state": rslam.status, "nav_relative": _agent_nav_relative,
    "action": lambda name: _sport_cmd(name), "jpeg": lambda: _latest_jpeg[0], "follow": _agent_follow,
    "patrol": _agent_patrol, "stop": _stop_all,
}, places_file=os.path.join(ROBOT_MAP_DIR, "places.json"), log=log)


KEYWORD_ACTIONS = {"hello": "Hello", "heart": "FingerHeart", "lie_down": "StandDown", "stand_up": "StandUp"}


@app.route("/action", methods=["POST"])
def direct_action():
    """👂 단어 듣기용: 정해진 동작만 바로 실행 (Claude 안 거침). stop은 모든 이동 정지."""
    if not can_drive():
        return jsonify({"ok": False, "error": "노트북 화면에서만 할 수 있어요"}), 403
    name = str((request.get_json(silent=True) or {}).get("name", ""))
    if name == "stop":
        agent.stop()
        _stop_all()
        _sport_cmd("StopMove")
        log("👂 멈춰")
        return jsonify({"ok": True})
    if name not in KEYWORD_ACTIONS:
        return jsonify({"ok": False, "error": "허용하지 않은 동작이에요"}), 400
    if not state["connected"]:
        return jsonify({"ok": False, "error": "로봇이 연결돼 있지 않아요"}), 409
    if follower.active:      # 따라가는 중: 멈추고 → 동작 → 다시 따라가기 (엎드려는 '일어나' 할 때까지 쉼)
        kind = {"lie_down": "rest", "stand_up": "wake"}.get(name, "act")
        threading.Thread(target=follower.perform, args=(KEYWORD_ACTIONS[name], kind), daemon=True).start()
        log(f"👂 따라가는 중 단어 동작: {name}")
        return jsonify({"ok": True, "during_follow": True})
    if nav.active or rslam.busy() or agent.busy:
        return jsonify({"ok": False, "error": "로봇이 다른 일을 하는 중이에요"}), 409
    _sport_cmd(KEYWORD_ACTIONS[name])
    if name == "stand_up":   # 일어선 뒤 걸을 수 있는 상태로
        threading.Timer(1.5, lambda: _sport_cmd("BalanceStand")).start()
    log(f"👂 단어 동작: {name}")
    return jsonify({"ok": True})


@app.route("/agent/ask", methods=["POST"])
def agent_ask():
    if not can_drive():
        return jsonify({"ok": False, "error": "노트북 화면에서만 할 수 있어요"}), 403
    text = str((request.get_json(silent=True) or {}).get("text", "")).strip()[:500]
    if not text:
        return jsonify({"ok": False, "error": "할 말을 입력해 주세요"}), 400
    if not os.environ.get("ANTHROPIC_API_KEY"):
        return jsonify({"ok": False, "error": ".env에 ANTHROPIC_API_KEY가 없어요"}), 409
    if not agent.ask(text):
        return jsonify({"ok": False, "error": "아직 이전 명령을 하는 중이에요"}), 409
    log(f"🗣 말로 시키기: {text}")
    return jsonify({"ok": True})


@app.route("/agent/state")
def agent_state():
    return jsonify({"busy": agent.busy, "chat": agent.chat[-30:], "places": agent.places()})


@app.route("/agent/reset", methods=["POST"])
def agent_reset():
    if not can_drive():
        return jsonify({"ok": False}), 403
    agent.reset()
    return jsonify({"ok": True})


@app.route("/uslam/fetch", methods=["POST"])
def uslam_fetch():
    """로봇의 지도 칸에서 파일 내려받기: map.pgm(2D) / map.txt(원점·해상도) / map.pcd(3D)"""
    if not can_drive():
        return jsonify({"ok": False, "error": "노트북 화면에서만 할 수 있어요"}), 403
    name = str((request.get_json(silent=True) or {}).get("file", ""))
    if name not in ("map.pgm", "map.txt", "map.pcd"):
        return jsonify({"ok": False, "error": "map.pgm / map.txt / map.pcd 중 하나"}), 400
    ok, res = _fetch_robot_file(name)
    if not ok:
        return jsonify({"ok": False, "error": res}), 502
    return jsonify({"ok": True, "bytes": res})


def _fetch_robot_file(name):
    """로봇 지도 칸의 파일을 maps/robot/ 에 저장. (성공여부, 바이트수 또는 오류)"""
    conn, loop = state["conn"], state["loop"]
    if conn is None or loop is None:
        return False, "로봇이 연결돼 있지 않아요"
    from unitree_webrtc_connect.msgs.rtc_inner_req import WebRTCDataChannelFileDownloader
    dl = WebRTCDataChannelFileDownloader(conn.datachannel.channel, conn.datachannel.pub_sub)
    fut = asyncio.run_coroutine_threadsafe(asyncio.wait_for(dl.download_file(name), 60), loop)
    try:
        data = fut.result(70)
    except Exception as e:
        return False, f"내려받기 실패: {type(e).__name__}"
    if not isinstance(data, (bytes, bytearray)):
        return False, f"내려받기 실패: {data}"
    os.makedirs(ROBOT_MAP_DIR, exist_ok=True)
    with open(os.path.join(ROBOT_MAP_DIR, name), "wb") as f:
        f.write(data)
    log(f"📥 로봇 지도 파일 받음: maps/robot/{name} ({len(data)} bytes)")
    return True, len(data)


@app.route("/lidar/restart", methods=["POST"])
def lidar_restart():
    """LiDAR 끄고 다시 켜기 (앱에서 SLAM 지도를 딴 뒤 실시간 LiDAR가 안 올 때)"""
    if not can_drive():
        return jsonify({"ok": False, "error": "노트북 화면에서만 할 수 있어요"}), 403
    conn, loop = state["conn"], state["loop"]
    if conn is None or loop is None:
        return jsonify({"ok": False, "error": "로봇이 연결돼 있지 않아요"}), 409

    async def cycle():
        ps = conn.datachannel.pub_sub
        ps.publish_without_callback("rt/utlidar/switch", "off")
        await asyncio.sleep(1.5)
        ps.publish_without_callback("rt/utlidar/switch", "on")
        await conn.datachannel.disableTrafficSaving(True)
        ps.unsubscribe("rt/utlidar/voxel_map_compressed")
        await asyncio.sleep(0.3)
        ps.subscribe("rt/utlidar/voxel_map_compressed", lambda m: update_lidar(m["data"]["data"]["points"]))
    asyncio.run_coroutine_threadsafe(cycle(), loop)
    log("🔄 LiDAR 끄고 다시 켬")
    return jsonify({"ok": True})


@app.route("/uslam/log")
def uslam_log_view():
    return jsonify(uslam_log[-60:])


@app.route("/uwb")
def uwb_raw():
    """리모컨(UWB) 원본 값 확인용"""
    t = state.get("uwb_t")
    return jsonify({"age": round(time.time() - t, 2) if t else None, "count": state.get("uwb_n", 0),
                    "data": state.get("uwb")})


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
        "can_drive": can_drive(), "nav": nav.status(), "follow": follower.status(), "rslam": rslam.status(),
        "uwb_buttons": (state["uwb"] or {}).get("buttons") if time.time() - state["uwb_t"] < 1 else None,
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
    p = _pose()                     # 지도 좌표 (스캔 정합 보정)
    tr = state["trail"]
    remote = is_remote()
    step = max(1, len(tr) // (300 if remote else 1000))  # 화면용으로 최대 ~1000점 (원격 300)
    pts = state["lidar"]
    if remote and len(pts) > 1200:  # 원격은 점을 줄여서 보냄
        pts = pts[:: len(pts) // 1200 + 1]
    return jsonify({
        "points": pts,
        "age": round(time.time() - state["lidar_t"], 1) if state["lidar_t"] else None,
        "robot": [p[0], p[1]] if p else None,
        "yaw": p[2] if p else None,
        "trail": tr[::step] + (tr[-1:] if tr and (len(tr) - 1) % step else []),
        "trail_len": round(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(tr, tr[1:])), 2),
        "stack": mapst.info(),
        "remote": _remote_world(p),
    })


def _remote_world(p):
    """리모컨(UWB) 위치를 지도 좌표로 — 지도에 분홍 점으로 표시"""
    r = follower.remote()
    if p is None or r is None or time.time() - r[2] > 1.0:
        return None
    d, b = r[0], r[1]
    return [round(p[0] + d * math.cos(p[2] + b), 3), round(p[1] + d * math.sin(p[2] + b), 3)]


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--ip", default=DEFAULT_IP)
    p.add_argument("--ap", action="store_true")
    p.add_argument("--key", default=DEFAULT_KEY)
    p.add_argument("--demo", action="store_true", help="로봇 없이 가짜 데이터로 실행")
    p.add_argument("--drift", action="store_true", help="데모에서 odom 오차·잡음을 일부러 넣음 (지도 보정 시험)")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--no-browser", action="store_true")
    args = p.parse_args()
    DEMO_DRIFT = args.drift
    if not args.demo and not args.ap and not args.ip:
        p.error(".env에 UNITREE_ROBOT_IP를 적거나 --ip 192.168.x.x 를 지정하세요 (로봇 없이 보려면 --demo)")

    threading.Thread(target=demo_thread if args.demo else robot_thread,
                     args=() if args.demo else (args,), daemon=True).start()
    threading.Thread(target=teleop_watchdog, daemon=True).start()

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
