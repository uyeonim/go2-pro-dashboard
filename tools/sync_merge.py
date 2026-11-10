"""화면 녹화 + 폰 영상을 '삐-삐-삐' 싱크 신호로 자동 맞춰서 한 영상으로 합치기.

찍는 법:
  1) 화면 녹화 시작 (Win+Alt+R 게임 바, 또는 OBS) — 소리도 같이 녹음되게
  2) 폰 녹화 시작 (로봇 쪽) — 노트북 스피커 소리가 들리는 거리에서
  3) 대시보드 위 🎬 싱크 버튼 → 삐-삐-삐 (두 영상 모두에 소리가 들어감)
  4) 시연 → 끝나면 둘 다 녹화 끝

합치기:
  .venv\\Scripts\\python tools\\sync_merge.py 화면.mp4 폰.mp4
  옵션: -o 결과.mp4   --layout side(나란히, 기본) | pip(화면 위에 폰 작게)   --audio phone(기본)|screen
결과: 두 영상이 싱크 신호 기준으로 맞춰지고, 위에 "싱크 후 경과 시간"이 0.1초 단위로 찍힘.
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys

import numpy as np

SR = 16000
BEEP_HZ, BEEPS, BEEP_LEN = 2000, (0.0, 0.4, 0.8), 0.15     # 대시보드 🎬 싱크와 같아야 함


def find_ffmpeg():
    exe = shutil.which("ffmpeg")
    if exe:
        return exe, shutil.which("ffprobe")
    hits = glob.glob(os.path.expandvars(r"%LOCALAPPDATA%\Microsoft\WinGet\Packages\Gyan.FFmpeg*\*\bin\ffmpeg.exe"))
    if not hits:
        sys.exit("❌ ffmpeg를 못 찾았어요 (winget install Gyan.FFmpeg)")
    return hits[0], hits[0].replace("ffmpeg.exe", "ffprobe.exe")


def audio(ff, path):
    raw = subprocess.run([ff, "-v", "error", "-i", path, "-vn", "-ac", "1", "-ar", str(SR), "-f", "s16le", "-"],
                         capture_output=True, check=True).stdout
    return np.frombuffer(raw, np.int16).astype(np.float32) / 32768.0


def duration(fp, path):
    out = subprocess.run([fp, "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", path],
                         capture_output=True, text=True).stdout
    return float(out.strip())


def is_portrait(fp, path):
    """세로 영상인지 (폰 회전 정보까지 반영)"""
    out = subprocess.run([fp, "-v", "error", "-select_streams", "v:0", "-show_entries",
                          "stream=width,height:stream_side_data=rotation", "-of", "default=nw=1", path],
                         capture_output=True, text=True).stdout
    kv = dict(line.split("=", 1) for line in out.split() if "=" in line)
    w, h = int(kv.get("width", 0)), int(kv.get("height", 0))
    if abs(int(float(kv.get("rotation", 0) or 0))) % 180 == 90:
        w, h = h, w
    return h > w


def find_sync(x):
    """삐-삐-삐가 시작하는 시각(초)과 확실한 정도(배)."""
    hop, win = 160, 512                                        # 10ms 간격
    n = (len(x) - win) // hop
    if n <= 0:
        return None, 0
    frames = np.lib.stride_tricks.as_strided(x, (n, win), (x.strides[0] * hop, x.strides[0]))
    spec = np.abs(np.fft.rfft(frames * np.hanning(win), axis=1))
    f = np.fft.rfftfreq(win, 1 / SR)
    band = spec[:, (f > BEEP_HZ - 120) & (f < BEEP_HZ + 120)].sum(1)
    total = spec.sum(1) + 1e-9
    score = band / total                                       # 2kHz가 전체 소리에서 차지하는 비율 (음량과 무관)
    tmpl = np.zeros(int((BEEPS[-1] + BEEP_LEN + 0.2) * 100))
    for b in BEEPS:
        tmpl[int(b * 100):int((b + BEEP_LEN) * 100)] = 1
    tmpl -= tmpl.mean()
    corr = np.correlate(score - np.median(score), tmpl, mode="valid")
    i = int(np.argmax(corr))
    conf = corr[i] / (np.median(np.abs(corr)) + 1e-9)
    return i * hop / SR, conf


def main():
    ap = argparse.ArgumentParser(description="화면 녹화 + 폰 영상 싱크 맞춰 합치기")
    ap.add_argument("screen")
    ap.add_argument("phone")
    ap.add_argument("-o", "--out")
    ap.add_argument("--layout", choices=["side", "pip"], default="side")
    ap.add_argument("--audio", choices=["phone", "screen"], default="phone")
    ap.add_argument("--height", type=int, default=720)
    a = ap.parse_args()
    ff, fp = find_ffmpeg()
    # 순서를 바꿔 넣어도 되게: 세로 영상이 하나뿐이면 그게 폰 영상
    if is_portrait(fp, a.screen) and not is_portrait(fp, a.phone):
        a.screen, a.phone = a.phone, a.screen
        print("↔ 세로 영상을 폰 영상으로 보고 순서를 바꿨어요")
    out = a.out or os.path.splitext(a.screen)[0] + "_합본.mp4"

    print("🔎 싱크 신호(삐-삐-삐) 찾는 중…")
    ts, cs = find_sync(audio(ff, a.screen))
    tp, cp = find_sync(audio(ff, a.phone))
    print(f"   화면 영상: {ts:.2f}초 (확실도 {cs:.0f}배)   폰 영상: {tp:.2f}초 (확실도 {cp:.0f}배)")
    if ts is None or tp is None or min(cs, cp) < 8:
        print("⚠ 신호가 약해요. 폰이 노트북 소리를 들을 수 있었는지, 화면 녹화에 소리가 들어갔는지 확인하세요.")
    pre = min(ts, tp, 3.0)                                     # 싱크 신호 3초 전부터
    ss_s, ss_p = ts - pre, tp - pre
    dur = min(duration(fp, a.screen) - ss_s, duration(fp, a.phone) - ss_p)
    print(f"   폰 영상이 화면보다 {tp - ts:+.2f}초 늦게 시작함 → 맞춰서 {dur:.1f}초 합침")

    H = a.height - a.height % 2
    timer = (f"drawtext=fontfile='C\\:/Windows/Fonts/consola.ttf':fontsize={H // 18}:fontcolor=white:"
             f"box=1:boxcolor=black@0.55:boxborderw=8:x=(w-tw)/2:y=12:"
             f"text='SYNC +%{{eif\\:floor(max(t-{pre:.3f},0))\\:d}}.%{{eif\\:floor(mod(max(t-{pre:.3f},0)*10,10))\\:d}}s'")
    if a.layout == "side":
        graph = (f"[1:v]scale=-2:{H},setsar=1[p];[0:v]scale=-2:{H},setsar=1[s];"
                 f"[p][s]hstack=inputs=2[v0];[v0]{timer}[v]")
    else:
        graph = (f"[0:v]scale=-2:{H},setsar=1[s];[1:v]scale=-2:{H // 2},setsar=1[p];"
                 f"[s][p]overlay=W-w-16:H-h-16[v0];[v0]{timer}[v]")
    gfile = out + ".filter.txt"
    with open(gfile, "w", encoding="utf-8") as f:
        f.write(graph)
    aud = "1:a?" if a.audio == "phone" else "0:a?"
    cmd = [ff, "-y", "-hide_banner", "-loglevel", "error", "-stats", "-ss", f"{ss_s:.3f}", "-i", a.screen, "-ss", f"{ss_p:.3f}", "-i", a.phone,
           "-t", f"{dur:.3f}", "-/filter_complex", gfile, "-map", "[v]", "-map", aud,
           "-c:v", "libx264", "-preset", "veryfast", "-crf", "21", "-pix_fmt", "yuv420p", "-c:a", "aac", "-b:a", "128k",
           "-movflags", "+faststart", out]
    print("🎬 합치는 중…")
    try:
        subprocess.run(cmd, check=True)
    finally:
        os.remove(gfile)
    print(f"✅ 완성: {out}")


if __name__ == "__main__":
    main()
