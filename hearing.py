"""로봇 귀: 로봇 마이크 소리(WebRTC 오디오) → 말소리 구간 자르기 → Whisper(노트북에서, 무료)로 글자로.

- feed(frame): aiortc 오디오 프레임을 16kHz 모노로 바꿔 쌓음 (로봇 연결 스레드에서 호출).
- 말소리 감지: 주변 소음 크기를 계속 재서, 그보다 충분히 큰 소리가 이어지면 말 시작, 0.8초 조용하면 끝.
- 로봇이 말하는 동안(mute_until)은 안 들음 — 자기 목소리를 알아듣지 않게.
- 알아들은 글자는 on_text(text) 로 넘김 (대시보드가 단어 동작/대화로 처리).
"""
import collections
import io
import threading
import time
import wave

import numpy as np

RATE = 16000
FRAME = 320                  # 20ms
START_RATIO = 3.0            # 소음보다 이만큼 크면 말소리
MIN_RMS = 250                # 이보다 작으면 말소리 아님 (16bit)
END_SIL = 0.8                # 이만큼 조용하면 말 끝
MIN_SPEECH = 0.35            # 너무 짧은 소리는 버림
MAX_SPEECH = 8.0
MODEL = "small"              # faster-whisper 모델 (small: 한국어 괜찮음, CPU에서도 1~2초)


class RobotEar:
    def __init__(self, on_text, log=print):
        self.on_text, self.log = on_text, log
        self.enabled = False
        self.mute_until = 0.0
        self.level = 0.0               # 지금 소리 크기 (화면 표시용)
        self.noise = 300.0             # 주변 소음 크기 추정
        self.frames_t = 0.0            # 마지막으로 소리 받은 시각
        self.heard = collections.deque(maxlen=20)   # [(t, text)]
        self.state = "꺼짐"
        self._resampler = None
        self._pend = np.zeros(0, np.int16)
        self._ring = collections.deque(maxlen=int(30 * RATE / FRAME))   # 최근 30초 (확인용 녹음)
        self._pre = collections.deque(maxlen=15)    # 말 시작 직전 0.3초도 같이
        self._speech, self._loud, self._quiet = [], 0, 0
        self._q = collections.deque(maxlen=3)
        self._model = None
        self._cv = threading.Condition()
        threading.Thread(target=self._worker, daemon=True).start()

    # ── 소리 받기 (로봇 연결 스레드) ──
    async def feed(self, frame):
        import av
        if self._resampler is None:
            self._resampler = av.AudioResampler(format="s16", layout="mono", rate=RATE)
        for f in self._resampler.resample(frame):
            pcm = f.to_ndarray().reshape(-1).astype(np.int16)
            self._pend = np.concatenate([self._pend, pcm])
        self.frames_t = time.time()
        while len(self._pend) >= FRAME:
            fr, self._pend = self._pend[:FRAME], self._pend[FRAME:]
            self._ring.append(fr)
            self._frame(fr)

    def _frame(self, fr):
        rms = float(np.sqrt(np.mean(fr.astype(np.float32) ** 2)))
        self.level = 0.8 * self.level + 0.2 * rms
        muted = time.time() < self.mute_until
        if not self.enabled or muted:
            self._speech, self._loud, self._quiet = [], 0, 0
            self.state = "꺼짐" if not self.enabled else "로봇이 말하는 중 (안 들음)"
            return
        loud = rms > max(MIN_RMS, self.noise * START_RATIO)
        if not self._speech:
            if not loud:                       # 조용할 때만 소음 크기 갱신
                self.noise = 0.98 * self.noise + 0.02 * rms
            self._pre.append(fr)
            self._loud = self._loud + 1 if loud else 0
            if self._loud >= 5:                # 0.1초 넘게 큰 소리 → 말 시작
                self._speech, self._quiet = list(self._pre), 0
                self.state = "말소리 듣는 중…"
            elif self.state != "알아듣는 중…":
                self.state = "듣는 중"
            return
        self._speech.append(fr)
        self._quiet = 0 if loud else self._quiet + 1
        dur = len(self._speech) * FRAME / RATE
        if self._quiet * FRAME / RATE >= END_SIL or dur >= MAX_SPEECH:
            speech = np.concatenate(self._speech)
            self._speech, self._loud, self._pre = [], 0, collections.deque(maxlen=15)
            if dur - self._quiet * FRAME / RATE >= MIN_SPEECH:
                with self._cv:
                    self._q.append(speech)
                    self._cv.notify()
                self.state = "알아듣는 중…"

    # ── 글자로 바꾸기 (별도 스레드) ──
    def _load(self):
        if self._model is None:
            from faster_whisper import WhisperModel
            t0 = time.time()
            try:
                self._model = WhisperModel(MODEL, device="cuda", compute_type="float16")
                self._model.transcribe(np.zeros(RATE, np.float32), language="ko")   # GPU 실제로 되는지
                dev = "GPU"
            except Exception:
                self._model = WhisperModel(MODEL, device="cpu", compute_type="int8")
                dev = "CPU"
            self.log(f"👂 음성 인식 준비됨 (Whisper {MODEL}, {dev}, {time.time() - t0:.0f}초)")
        return self._model

    def _worker(self):
        while True:
            with self._cv:
                while not self._q:
                    self._cv.wait()
                speech = self._q.popleft()
            try:
                t0 = time.time()
                segs, _ = self._load().transcribe(
                    speech.astype(np.float32) / 32768.0, language="ko", beam_size=1, vad_filter=False,
                    condition_on_previous_text=False, no_speech_threshold=0.6)
                text = " ".join(s.text.strip() for s in segs if s.no_speech_prob < 0.6).strip()
            except Exception as e:
                self.log(f"👂 음성 인식 오류: {e!r}")
                continue
            if self.state == "알아듣는 중…":
                self.state = "듣는 중"
            if not text or _junk(text):
                continue
            self.heard.append((time.time(), text))
            self.log(f"👂 로봇이 들음 ({len(speech) / RATE:.1f}초 → {time.time() - t0:.1f}초): {text}")
            try:
                self.on_text(text)
            except Exception as e:
                self.log(f"👂 처리 오류: {e!r}")

    def warmup(self):
        threading.Thread(target=self._load, daemon=True).start()

    def wav(self, seconds=10):
        """최근 소리를 wav로 (마이크 확인용)"""
        n = int(seconds * RATE / FRAME)
        pcm = np.concatenate(list(self._ring)[-n:]) if self._ring else np.zeros(0, np.int16)
        b = io.BytesIO()
        with wave.open(b, "wb") as w:
            w.setnchannels(1), w.setsampwidth(2), w.setframerate(RATE)
            w.writeframes(pcm.tobytes())
        return b.getvalue()

    def status(self):
        return {"enabled": self.enabled, "state": self.state, "level": round(self.level),
                "noise": round(self.noise), "receiving": time.time() - self.frames_t < 1.0,
                "heard": [t for _, t in list(self.heard)[-5:]]}


# Whisper가 조용한 소리에서 자주 지어내는 말
_JUNK = ("시청해 주셔서", "구독", "좋아요", "MBC", "KBS", "자막", "감사합니다.", "다음 영상")


def _junk(text):
    t = text.strip()
    return len(t) < 2 or any(j in t for j in _JUNK) and len(t) < 20
