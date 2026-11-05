"""Go2 센서값 받아오기 (Wi-Fi / WebRTC).

사용법:
  .venv\\Scripts\\python sensors.py --ip 192.168.x.x        # 로봇과 같은 Wi-Fi(STA)
  .venv\\Scripts\\python sensors.py --ap                     # 로봇 핫스팟(AP)에 연결한 경우
  펌웨어 1.1.15 이상이면 --key <32자리 AES 키> 추가

받은 값은 화면에 출력하고 logs/sensors_<시각>.csv 에 저장한다. Ctrl+C로 종료.
"""
import argparse
import asyncio
import csv
import os
import time
from datetime import datetime

from unitree_webrtc_connect import RTC_TOPIC, UnitreeWebRTCConnection, WebRTCConnectionMethod


def load_env(path=os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")):
    """프로젝트 폴더의 .env (KEY=VALUE 줄)를 환경변수로 읽음. 이미 설정된 값은 덮지 않음."""
    if not os.path.exists(path):
        return
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))


load_env()  # sensors.py / camera.py / dashboard.py 모두 이걸 거쳐서 .env를 읽음

latest = {"sport": None, "low": None}


def make_conn(args):
    kw = {"aes_128_key": args.key} if args.key else {}
    if args.ap:
        return UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP, **kw)
    return UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=args.ip, **kw)


async def main(args):
    conn = make_conn(args)
    print("로봇에 연결 중...")
    await conn.connect()
    print("연결됨!")

    sub = conn.datachannel.pub_sub.subscribe
    sub(RTC_TOPIC["LF_SPORT_MOD_STATE"], lambda m: latest.__setitem__("sport", m["data"]))
    sub(RTC_TOPIC["LOW_STATE"], lambda m: latest.__setitem__("low", m["data"]))

    os.makedirs("logs", exist_ok=True)
    path = f"logs/sensors_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z", "vx", "vy", "vz", "yaw_speed",
                    "roll", "pitch", "yaw", "foot_force", "battery_pct", "voltage"])
        t0 = time.time()
        while True:
            await asyncio.sleep(1 / args.hz)
            s, low = latest["sport"], latest["low"]
            if s is None:
                continue
            pos, vel, rpy = s["position"], s["velocity"], s["imu_state"]["rpy"]
            soc = low["bms_state"]["soc"] if low else ""
            volt = low["power_v"] if low else ""
            w.writerow([round(time.time() - t0, 3), *pos, *vel, s["yaw_speed"], *rpy,
                        s["foot_force"], soc, volt])
            f.flush()
            print(f"pos=({pos[0]:+.2f},{pos[1]:+.2f})m  yaw={rpy[2]:+.2f}rad  "
                  f"vel=({vel[0]:+.2f},{vel[1]:+.2f})  배터리={soc}%  -> {path}", end="\r")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--ip", default=os.environ.get("UNITREE_ROBOT_IP"))
    p.add_argument("--ap", action="store_true", help="로봇 핫스팟에 직접 연결한 경우")
    p.add_argument("--key", default=os.environ.get("UNITREE_AES_128_KEY"), help="펌웨어 1.1.15+ AES 키 (.env에 두는 걸 권장)")
    p.add_argument("--hz", type=float, default=10, help="저장 주기")
    a = p.parse_args()
    if not a.ap and not a.ip:
        p.error("--ip 192.168.x.x 또는 --ap 를 지정하세요")
    try:
        asyncio.run(main(a))
    except KeyboardInterrupt:
        print("\n종료")
