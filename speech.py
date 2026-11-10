"""로봇이 말하기: 글자 → 한국어 음성 wav (윈도우 내장 TTS, 무료) → 로봇 스피커(메가폰)로 재생.

윈도우 '설정 > 시간 및 언어 > 음성'에 한국어 음성(Microsoft Heami)이 있어야 해요.
로봇 쪽은 unitree 앱의 메가폰과 같은 방식: ENTER_MEGAPHONE → wav 조각 올리기(UPLOAD_MEGAPHONE) → 다 재생되면 EXIT.
"""
import asyncio
import base64
import json
import os
import subprocess
import tempfile
import wave

RATE = 16000             # 작게 만들어야 빨리 올라감 (16kHz 16bit 모노 ≈ 32KB/초)
CHUNK = 16384            # base64 글자 수 (라이브러리는 4096 — 크게 보내면 조각 수가 줄어 빨라짐)
MAX_CHARS = 200

_PS = r"""
Add-Type -AssemblyName System.Speech
$s = New-Object System.Speech.Synthesis.SpeechSynthesizer
$v = $s.GetInstalledVoices() | Where-Object { $_.VoiceInfo.Culture.Name -eq 'ko-KR' } | Select-Object -First 1
if ($v) { $s.SelectVoice($v.VoiceInfo.Name) }
$s.Rate = 1
$fmt = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(%d, [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen, [System.Speech.AudioFormat.AudioChannel]::Mono)
$s.SetOutputToWaveFile($env:TTS_OUT, $fmt)
$s.Speak([IO.File]::ReadAllText($env:TTS_IN, [Text.Encoding]::UTF8))
$s.Dispose()
""" % RATE


def tts_wav(text):
    """글자 → wav 바이트, 길이(초)"""
    text = " ".join(str(text).split())[:MAX_CHARS]
    if not text:
        raise ValueError("말할 내용이 없어요")
    d = tempfile.mkdtemp(prefix="tts_")
    src, out = os.path.join(d, "in.txt"), os.path.join(d, "out.wav")
    try:
        with open(src, "w", encoding="utf-8") as f:
            f.write(text)
        env = dict(os.environ, TTS_IN=src, TTS_OUT=out)
        subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command", _PS],
                       env=env, check=True, timeout=30, capture_output=True,
                       creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        with open(out, "rb") as f:
            data = f.read()
        with wave.open(out) as w:
            sec = w.getnframes() / w.getframerate()
        return data, sec
    finally:
        for p in (src, out):
            try:
                os.remove(p)
            except OSError:
                pass
        try:
            os.rmdir(d)
        except OSError:
            pass


async def play_on_robot(pub_sub, wav, seconds, log=print, chunk=None):
    """메가폰으로 wav 재생하고, 다 말할 때까지 기다렸다가 메가폰 끄기"""
    topic = "rt/api/audiohub/request"

    async def req(api_id, param):
        return await asyncio.wait_for(pub_sub.publish_request_new(
            topic, {"api_id": api_id, "parameter": json.dumps(param, ensure_ascii=True)}), 5)

    b64 = base64.b64encode(wav).decode()
    chunk = chunk or CHUNK
    chunks = [b64[i:i + chunk] for i in range(0, len(b64), chunk)]
    t0 = asyncio.get_running_loop().time()
    await req(4001, {})                          # ENTER_MEGAPHONE
    try:
        for i, ch in enumerate(chunks, 1):
            await req(4003, {"current_block_size": len(ch), "block_content": ch,
                             "current_block_index": i, "total_block_number": len(chunks)})
        log(f"🗣 로봇에 음성 올림: {len(chunks)}조각, {asyncio.get_running_loop().time() - t0:.1f}초")
        await asyncio.sleep(seconds + 0.8)       # 재생 끝날 때까지
    finally:
        try:
            await req(4002, {})                  # EXIT_MEGAPHONE
        except Exception:
            pass
    return len(chunks)
