"""Go2 전면 카메라 영상 보기/저장 (Wi-Fi / WebRTC).

사용법 (sensors.py 와 같은 옵션):
  .venv\\Scripts\\python camera.py --ap --key <AES키>
  .venv\\Scripts\\python camera.py --ip 192.168.x.x --key <AES키>

창에서 키:  q 종료 | s 사진 저장 | r 녹화 시작/정지   (저장 위치: captures/)
"""
import argparse
import asyncio
import os
import threading
from datetime import datetime
from queue import Empty, Queue

import cv2

from sensors import make_conn

frames: Queue = Queue(maxsize=1)  # 최신 프레임 1장만 유지 (지연 방지)
errors: list = []


def start_robot_thread(args):
    loop = asyncio.new_event_loop()

    async def on_track(track):
        while True:
            img = (await track.recv()).to_ndarray(format="bgr24")
            if frames.full():
                try:
                    frames.get_nowait()
                except Empty:
                    pass
            frames.put(img)

    async def setup():
        conn = make_conn(args)
        await conn.connect()
        conn.video.switchVideoChannel(True)
        conn.video.add_track_callback(on_track)

    def run():
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(setup())
            loop.run_forever()
        except Exception as e:  # 연결 실패 등은 메인 스레드에서 출력
            errors.append(e)

    threading.Thread(target=run, daemon=True).start()


def stamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def main(args):
    os.makedirs("captures", exist_ok=True)
    print("로봇에 연결 중... (영상 창이 뜨면 q 종료 / s 사진 / r 녹화)")
    start_robot_thread(args)

    writer = None
    while True:
        if errors:
            print(f"\n연결 실패: {errors[0]!r}")
            break
        try:
            img = frames.get(timeout=0.1)
        except Empty:
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            continue

        if writer is not None:
            writer.write(img)
            cv2.circle(img, (30, 30), 10, (0, 0, 255), -1)  # 녹화 중 표시(저장본에는 안 들어감)
        cv2.imshow("Go2 camera", img)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            path = f"captures/photo_{stamp()}.jpg"
            cv2.imwrite(path, img)
            print("사진 저장:", path)
        if key == ord("r"):
            if writer is None:
                path = f"captures/video_{stamp()}.mp4"
                h, w = img.shape[:2]
                writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), 15, (w, h))
                print("녹화 시작:", path)
            else:
                writer.release()
                writer = None
                print("녹화 정지")

    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--ip", default=os.environ.get("UNITREE_ROBOT_IP"))
    p.add_argument("--ap", action="store_true", help="로봇 핫스팟에 직접 연결한 경우")
    p.add_argument("--key", default=os.environ.get("UNITREE_AES_128_KEY"), help="펌웨어 1.1.15+ AES 키")
    a = p.parse_args()
    if not a.ap and not a.ip:
        p.error("--ip 192.168.x.x 또는 --ap 를 지정하세요")
    main(a)
