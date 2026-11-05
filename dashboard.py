"""Go2 실시간 대시보드 (왼쪽 카메라 영상 + 오른쪽 센서값).

실행:
  .venv\\Scripts\\python dashboard.py            # IP와 키는 .env에서 읽음
  .venv\\Scripts\\python dashboard.py --ip 192.168.x.x --key <AES키>

실행하면 브라우저가 자동으로 열리고, 로봇에 자동 연결돼요.
로봇은 WebRTC를 한 번에 하나만 받으니, 폰 앱은 종료해 두세요.
"""
import argparse
import asyncio
import logging
import os
import threading
import time
import webbrowser

import cv2
import numpy as np
from flask import Flask, Response, jsonify, send_from_directory

from sensors import make_conn
from unitree_webrtc_connect import RTC_TOPIC

# 접속 정보는 .env에서 읽음 (sensors.py의 load_env). 코드에 키를 적지 않음.
DEFAULT_IP = os.environ.get("UNITREE_ROBOT_IP")
DEFAULT_KEY = os.environ.get("UNITREE_AES_128_KEY")

state = {
    "sport": None,
    "low": None,
    "lidar": [],      # 위에서 본 [x, y] 점 목록 (m, 월드 좌표)
    "lidar_t": 0.0,
    "status": "연결 대기 중...",
    "connected": False,
}
_frame_lock = threading.Lock()
_latest_jpeg = [None]
_latest_t = [0.0]  # 마지막 프레임 받은 시각

logging.getLogger("werkzeug").setLevel(logging.ERROR)  # /data 요청 로그 숨김

app = Flask(__name__, static_folder="web", static_url_path="")


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def _placeholder(text):
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:] = (30, 30, 35)
    cv2.putText(img, text, (40, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 180, 190), 2)
    ok, buf = cv2.imencode(".jpg", img)
    return buf.tobytes()


def robot_thread(args):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    async def on_track(track):
        while True:
            img = (await track.recv()).to_ndarray(format="bgr24")
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with _frame_lock:
                    _latest_jpeg[0] = buf.tobytes()
                    _latest_t[0] = time.time()

    def on_lidar(m):
        try:
            pts = m["data"]["data"]["points"]  # N x 3 (m)
            if len(pts) == 0:
                return
            s = state["sport"]
            z0 = s["position"][2] if s else 0.0
            # 몸 높이 근처(바닥/천장 제외)만 남겨 장애물처럼 보이게
            band = pts[(pts[:, 2] > z0 - 0.15) & (pts[:, 2] < z0 + 0.6)]
            if len(band) > 4000:
                band = band[np.random.choice(len(band), 4000, replace=False)]
            state["lidar"] = np.round(band[:, :2], 2).tolist()
            state["lidar_t"] = time.time()
        except Exception as e:
            log(f"LiDAR 처리 오류: {e}")

    async def setup():
        while True:
            try:
                state["status"] = f"로봇({args.ip})에 연결 중..."
                log(f"로봇 {args.ip} 연결 시도...")
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
                sub(RTC_TOPIC["LF_SPORT_MOD_STATE"], lambda m: state.__setitem__("sport", m["data"]))
                sub(RTC_TOPIC["LOW_STATE"], lambda m: state.__setitem__("low", m["data"]))
                conn.video.switchVideoChannel(True)
                conn.video.add_track_callback(on_track)
                try:  # LiDAR 켜기 + 구독 (실패해도 영상/센서는 계속)
                    conn.datachannel.set_decoder(decoder_type="native")
                    await conn.datachannel.disableTrafficSaving(True)
                    conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")
                    sub("rt/utlidar/voxel_map_compressed", on_lidar)
                except Exception as e:
                    log(f"LiDAR 시작 실패(영상/센서는 계속): {e}")
                state["status"] = "연결됨"
                state["connected"] = True
                log("✅ 연결됨 — 센서/영상 수신 시작")
                return
            except Exception as e:
                state["connected"] = False
                state["status"] = f"연결 실패: {type(e).__name__} — 5초 후 재시도"
                log(f"❌ 연결 실패: {type(e).__name__}: {e} — 5초 후 재시도")
                await asyncio.sleep(5)

    loop.run_until_complete(setup())
    loop.run_forever()


@app.route("/")
def index():
    return send_from_directory("web", "index.html")


@app.route("/video")
def video():
    def gen():
        while True:
            with _frame_lock:
                frame = _latest_jpeg[0]
            if frame is None:
                frame = _placeholder("카메라 영상 대기 중...")
            yield (b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(1 / 15)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=f")


@app.route("/frame")
def frame():
    """최신 카메라 사진 한 장 (페이지가 1초에 여러 번 받아 감)."""
    with _frame_lock:
        jpg, t = _latest_jpeg[0], _latest_t[0]
    if jpg is None:
        jpg = _placeholder("카메라 영상 대기 중...")
    r = Response(jpg, mimetype="image/jpeg")
    r.headers["Cache-Control"] = "no-store"
    r.headers["X-Frame-Age"] = f"{time.time() - t:.2f}" if t else "-1"
    return r


@app.route("/data")
def data():
    s, low = state["sport"], state["low"]
    out = {"status": state["status"], "connected": state["connected"]}
    if s is not None:
        out.update({
            "pos": s["position"][:2],
            "height": round(s["body_height"], 3),
            "vel": [round(v, 3) for v in s["velocity"]],
            "yaw_speed": round(s["yaw_speed"], 3),
            "rpy": [round(v, 3) for v in s["imu_state"]["rpy"]],
            "foot_force": s["foot_force"],
            "mode": s["mode"],
            "gait": s["gait_type"],
            "obstacle": s.get("range_obstacle"),
        })
    if low is not None:
        out["battery"] = low["bms_state"]["soc"]
        out["voltage"] = round(low["power_v"], 1)
        out["temp"] = low.get("temperature_ntc1")
    return jsonify(out)


@app.route("/lidar")
def lidar():
    s = state["sport"]
    return jsonify({
        "points": state["lidar"],
        "age": round(time.time() - state["lidar_t"], 1) if state["lidar_t"] else None,
        "robot": s["position"][:2] if s else None,
        "yaw": s["imu_state"]["rpy"][2] if s else None,
    })


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--ip", default=DEFAULT_IP)
    p.add_argument("--ap", action="store_true")
    p.add_argument("--key", default=DEFAULT_KEY)
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--no-browser", action="store_true")
    args = p.parse_args()
    if not args.ap and not args.ip:
        p.error(".env에 UNITREE_ROBOT_IP를 적거나 --ip 192.168.x.x 를 지정하세요")

    threading.Thread(target=robot_thread, args=(args,), daemon=True).start()

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
    print(f"대시보드: {url}  (종료: Ctrl+C)")
    app.run(host="0.0.0.0", port=args.port, threaded=True)
