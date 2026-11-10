"""말로 시키기 (LLM 에이전트, 3번 과제): Claude API 도구 호출로 로봇을 조종.

"문 앞에 갔다가 하트하고 돌아와" → Claude가 아래 도구 중에서 골라 순서대로 부르고, 우리 코드가 실행.
허용한 도구만 실행되므로 안전장치(속도 제한·정지 버튼·리모컨 비상정지)는 그대로.

도구: get_status, time_weather, say, list_places, save_place, go_to_place, move_relative, do_action, look, follow_me, patrol_places, stop
장소는 로봇 내장 SLAM 지도 좌표로 maps/robot/places.json 에 저장 (go_to_place는 내장 내비게이션 사용).
API 키: .env 의 ANTHROPIC_API_KEY (sensors.load_env가 읽음).
"""
import base64
import json
import math
import os
import threading
import time
import urllib.request

import anthropic

MODEL = "claude-opus-5-5"
MAX_STEPS = 10           # 한 번 명령에 도구를 최대 몇 번까지

ACTIONS = {              # 허용하는 동작만 (점프·뒤집기 같은 위험한 건 뺌)
    "hello": "Hello", "heart": "FingerHeart", "sit": "Sit", "rise_sit": "RiseSit", "stand_up": "StandUp",
    "lie_down": "StandDown", "stretch": "Stretch", "dance1": "Dance1", "dance2": "Dance2",
    "wiggle_hips": "WiggleHips", "scrape": "Scrape", "content": "Content", "balance_stand": "BalanceStand",
}

SYSTEM = """너는 연구실의 Unitree Go2 로봇개를 조종하는 비서야. 사용자는 한국어로 짧게 말해.
- 사용자가 시킨 일을 도구로 차례대로 실행하고, 끝나면 한두 문장으로 무엇을 했는지 한국어로 답해.
- 장소 이름은 list_places로 확인하고, 없는 장소면 지어내지 말고 없다고 말해 (save_place로 지금 자리를 저장할 수 있다고 안내).
- 이동 결과가 실패면 다시 시도하지 말고 이유를 알려줘.
- 동작(do_action)은 로봇이 서 있을 때 해. 누워 있거나 앉아 있으면 먼저 stand_up.
- 사람이 위험하다고 하거나 "멈춰/그만"이라고 하면 바로 stop.
- 카메라로 봐야 답할 수 있는 질문이면 look을 써.
- "~라고 말해", "인사해 봐(말로)", "소개해 봐"처럼 로봇이 소리 내서 말해야 하면 say를 써. 짧고 자연스러운 한국어 한두 문장으로.
- 시간·날짜·날씨를 물으면 time_weather로 확인하고 답해 (지어내지 말기). 숫자는 말하기 좋게 (예: 오후 3시 20분, 영상 12도).
- 확실하지 않은 큰 이동은 하지 말고 한 번 물어봐."""

TOOLS = [
    {"name": "get_status", "description": "로봇 상태: 배터리, 위치(지도 좌표), 위치를 찾았는지, 지금 하는 일.",
     "input_schema": {"type": "object", "properties": {}, "additionalProperties": False}},
    {"name": "time_weather", "description": "지금 날짜·시각·요일과 현재 날씨(하늘, 기온, 체감, 오늘 최고/최저, 비 올 확률).",
     "input_schema": {"type": "object", "properties": {}, "additionalProperties": False}},
    {"name": "say", "description": "로봇 스피커로 소리 내서 말하기 (한국어, 200자 이하). 다 말할 때까지 기다림.",
     "input_schema": {"type": "object", "properties": {"text": {"type": "string"}},
                      "required": ["text"], "additionalProperties": False}},
    {"name": "list_places", "description": "저장된 장소 이름 목록과 좌표.",
     "input_schema": {"type": "object", "properties": {}, "additionalProperties": False}},
    {"name": "save_place", "description": "로봇이 지금 있는 자리(지도 좌표·방향)를 이름으로 저장.",
     "input_schema": {"type": "object", "properties": {"name": {"type": "string", "description": "장소 이름 (예: 문 앞)"}},
                      "required": ["name"], "additionalProperties": False}},
    {"name": "go_to_place", "description": "저장된 장소로 이동 (로봇이 스스로 길 찾음). 도착하거나 실패할 때까지 기다렸다가 결과를 돌려줌.",
     "input_schema": {"type": "object", "properties": {"name": {"type": "string"}},
                      "required": ["name"], "additionalProperties": False}},
    {"name": "move_relative", "description": "지금 자리에서 조금 이동: 앞으로 forward_m (음수면 뒤로), 왼쪽 left_m, 그다음 turn_deg 만큼 회전(왼쪽 +). 각각 최대 3m / 180도.",
     "input_schema": {"type": "object", "properties": {
         "forward_m": {"type": "number"}, "left_m": {"type": "number"}, "turn_deg": {"type": "number"}},
         "required": ["forward_m", "left_m", "turn_deg"], "additionalProperties": False}},
    {"name": "do_action", "description": "몸 동작 하기.",
     "input_schema": {"type": "object", "properties": {"action": {"type": "string", "enum": sorted(ACTIONS)}},
                      "required": ["action"], "additionalProperties": False}},
    {"name": "look", "description": "로봇 앞 카메라 사진 한 장을 봄.",
     "input_schema": {"type": "object", "properties": {}, "additionalProperties": False}},
    {"name": "follow_me", "description": "리모컨(UWB 태그) 든 사람 따라가기 켜기/끄기.",
     "input_schema": {"type": "object", "properties": {"on": {"type": "boolean"}},
                      "required": ["on"], "additionalProperties": False}},
    {"name": "patrol_places", "description": "여러 장소를 차례로 도는 순찰 시작 (시작만 하고 바로 돌아옴). loops=-1 이면 계속.",
     "input_schema": {"type": "object", "properties": {
         "names": {"type": "array", "items": {"type": "string"}}, "loops": {"type": "integer"}},
         "required": ["names", "loops"], "additionalProperties": False}},
    {"name": "stop", "description": "모든 이동을 즉시 멈춤.",
     "input_schema": {"type": "object", "properties": {}, "additionalProperties": False}},
]
for _t in TOOLS:
    _t["strict"] = True


# 날씨: Open-Meteo (무료, 키 없음). 위치는 .env 의 WEATHER_LAT / WEATHER_LON / WEATHER_PLACE (기본: 서울)
WMO = {0: "맑음", 1: "대체로 맑음", 2: "구름 조금", 3: "흐림", 45: "안개", 48: "안개", 51: "약한 이슬비", 53: "이슬비",
       55: "강한 이슬비", 61: "약한 비", 63: "비", 65: "강한 비", 66: "어는 비", 67: "어는 비", 71: "약한 눈", 73: "눈",
       75: "많은 눈", 77: "싸락눈", 80: "소나기", 81: "소나기", 82: "강한 소나기", 85: "눈 소나기", 86: "강한 눈 소나기",
       95: "뇌우", 96: "우박 동반 뇌우", 99: "우박 동반 뇌우"}
DAYS = "월화수목금토일"


def time_weather():
    now = time.localtime()
    out = {"now": time.strftime("%Y-%m-%d %H:%M", now) + f" ({DAYS[now.tm_wday]}요일)"}
    lat, lon = os.environ.get("WEATHER_LAT", "37.5665"), os.environ.get("WEATHER_LON", "126.9780")
    out["place"] = os.environ.get("WEATHER_PLACE", "서울")
    url = (f"https://api.open-meteo.com/v1/forecast?latitude={lat}&longitude={lon}&timezone=Asia%2FSeoul&forecast_days=1"
           "&current=temperature_2m,apparent_temperature,relative_humidity_2m,precipitation,weather_code,wind_speed_10m"
           "&daily=temperature_2m_max,temperature_2m_min,precipitation_probability_max")
    try:
        with urllib.request.urlopen(url, timeout=5) as r:
            w = json.load(r)
        c, d = w["current"], w["daily"]
        out["weather"] = {"sky": WMO.get(c["weather_code"], str(c["weather_code"])), "temp_c": c["temperature_2m"],
                          "feels_like_c": c["apparent_temperature"], "humidity_pct": c["relative_humidity_2m"],
                          "wind_kmh": c["wind_speed_10m"], "today_max_c": d["temperature_2m_max"][0],
                          "today_min_c": d["temperature_2m_min"][0], "rain_chance_pct": d["precipitation_probability_max"][0]}
    except Exception as e:
        out["weather"] = f"날씨를 못 가져왔어요 ({e.__class__.__name__})"
    return out


class RobotAgent:
    def __init__(self, robot, places_file, log=print):
        """robot: 대시보드가 넘겨주는 함수 모음 (dict) — status, pose_map, goto_map, rslam_state, nav_relative,
        action, jpeg, follow, patrol, stop, say"""
        self.robot, self.places_file, self.log = robot, places_file, log
        self.client = None
        self.lock = threading.Lock()
        self.history = []        # Claude와의 대화 (응답 content 그대로 덧붙임 — 생각 블록 보존)
        self.chat = []           # 화면에 보여줄 대화 [{role, text}]
        self.busy = False
        self.cancel = threading.Event()

    # ── 장소 ──
    def places(self):
        try:
            with open(self.places_file, encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return {}

    def _save_places(self, p):
        os.makedirs(os.path.dirname(self.places_file), exist_ok=True)
        with open(self.places_file, "w", encoding="utf-8") as f:
            json.dump(p, f, ensure_ascii=False, indent=1)

    # ── 도구 실행 ──
    def run_tool(self, name, a):
        R = self.robot
        if name == "get_status":
            return json.dumps(R["status"](), ensure_ascii=False)
        if name == "time_weather":
            return json.dumps(time_weather(), ensure_ascii=False)
        if name == "say":
            err = R["say"](a["text"])
            return f"실패: {err}" if err else "말했어요."
        if name == "list_places":
            p = self.places()
            return json.dumps(p, ensure_ascii=False) if p else "저장된 장소가 없어요."
        if name == "save_place":
            pose = R["pose_map"]()
            if not pose:
                return "실패: 로봇이 지도 위 위치를 아직 몰라요 (로봇 지도 화면에서 📍 위치 지정 필요)."
            p = self.places()
            p[a["name"]] = [round(pose[0], 3), round(pose[1], 3), round(pose[2], 3)]
            self._save_places(p)
            return f"'{a['name']}' 저장: x={pose[0]:.2f}, y={pose[1]:.2f}"
        if name == "go_to_place":
            p = self.places().get(a["name"])
            if not p:
                return f"실패: '{a['name']}' 라는 장소가 없어요. 있는 장소: {', '.join(self.places()) or '없음'}"
            err = R["goto_map"](*p)
            if err:
                return f"실패: {err}"
            return self._wait_nav(timeout=120)
        if name == "move_relative":
            fwd = max(-3.0, min(3.0, float(a["forward_m"])))
            left = max(-3.0, min(3.0, float(a["left_m"])))
            turn = math.radians(max(-180.0, min(180.0, float(a["turn_deg"]))))
            return R["nav_relative"](fwd, left, turn, self.cancel)
        if name == "do_action":
            R["action"](ACTIONS[a["action"]])
            time.sleep(4.0 if a["action"] not in ("stand_up", "balance_stand") else 2.0)
            return f"{a['action']} 했어요."
        if name == "look":
            jpg = R["jpeg"]()
            if not jpg:
                return "카메라 영상이 없어요."
            return [{"type": "image", "source": {"type": "base64", "media_type": "image/jpeg",
                                                 "data": base64.b64encode(jpg).decode()}},
                    {"type": "text", "text": "로봇 앞 카메라 사진이에요."}]
        if name == "follow_me":
            return R["follow"](bool(a["on"]))
        if name == "patrol_places":
            pl = self.places()
            missing = [n for n in a["names"] if n not in pl]
            if missing:
                return f"실패: 없는 장소 {missing}"
            err = R["patrol"]([pl[n] for n in a["names"]], int(a["loops"]))
            return f"실패: {err}" if err else f"순찰 시작: {' → '.join(a['names'])} ({'계속' if a['loops'] < 0 else str(a['loops']) + '바퀴'})"
        if name == "stop":
            R["stop"]()
            return "멈췄어요."
        return f"모르는 도구: {name}"

    def _wait_nav(self, timeout):
        t0 = time.time()
        time.sleep(1.5)
        while time.time() - t0 < timeout:
            if self.cancel.is_set():
                return "취소됨 (정지)"
            s = self.robot["rslam_state"]()
            st = s.get("nav_state", "")
            if st == "REACHED" and not s.get("goal"):
                return "도착했어요."
            if st in ("FAILURE", "NO_PATH", "GOAL_OCCUPIED", "TIMEOUT"):
                return f"실패: {st}"
            if not s.get("nav_active"):
                return "이동이 중간에 멈췄어요 (정지 버튼 등)."
            time.sleep(0.5)
        return "실패: 시간 초과"

    # ── 대화 ──
    def ask(self, text):
        """백그라운드에서 실행. 화면은 self.chat 을 계속 읽어감."""
        with self.lock:
            if self.busy:
                return False
            self.busy = True
        self.cancel.clear()
        self.chat.append({"role": "user", "text": text, "t": time.time()})
        threading.Thread(target=self._run, args=(text,), daemon=True).start()
        return True

    def reset(self):
        with self.lock:
            if not self.busy:
                self.history, self.chat = [], []

    def stop(self):
        self.cancel.set()

    def _run(self, text):
        try:
            if self.client is None:
                self.client = anthropic.Anthropic()          # .env의 ANTHROPIC_API_KEY
            if len(self.history) > 40:                      # 너무 길어지면 새 대화로 (중간을 고치지 않음)
                self.history = []
            self.history.append({"role": "user", "content": text})
            for _ in range(MAX_STEPS):
                if self.cancel.is_set():
                    self.chat.append({"role": "assistant", "text": "⏹ 멈췄어요.", "t": time.time()})
                    return
                resp = self.client.beta.messages.create(
                    model=MODEL, max_tokens=4096, system=SYSTEM, tools=TOOLS, messages=self.history,
                    output_config={"effort": "low"},          # 간단한 도구 고르기 → 빠르게
                    cache_control={"type": "ephemeral"},
                    betas=["server-side-fallback-2026-07-01"], fallbacks="default",
                )
                self.history.append({"role": "assistant", "content": resp.content})
                if resp.stop_reason == "refusal":
                    self.chat.append({"role": "assistant", "text": "그 요청은 할 수 없어요.", "t": time.time()})
                    return
                calls = [b for b in resp.content if b.type == "tool_use"]
                if resp.stop_reason != "tool_use" or not calls:
                    reply = " ".join(b.text for b in resp.content if b.type == "text").strip()
                    self.chat.append({"role": "assistant", "text": reply or "(답 없음)", "t": time.time()})
                    return
                results = []
                for c in calls:
                    self.chat.append({"role": "tool", "text": f"🔧 {c.name}({json.dumps(c.input, ensure_ascii=False)})",
                                      "t": time.time()})
                    self.log(f"🤖 도구: {c.name} {c.input}")
                    try:
                        out = self.run_tool(c.name, c.input)
                        results.append({"type": "tool_result", "tool_use_id": c.id, "content": out})
                        if isinstance(out, str):
                            self.chat.append({"role": "tool", "text": f"   → {out[:150]}", "t": time.time()})
                    except Exception as e:
                        results.append({"type": "tool_result", "tool_use_id": c.id, "is_error": True,
                                        "content": f"{type(e).__name__}: {e}"})
                self.history.append({"role": "user", "content": results})
            self.chat.append({"role": "assistant", "text": "단계가 너무 많아서 여기서 멈췄어요.", "t": time.time()})
        except anthropic.AuthenticationError:
            self.chat.append({"role": "assistant", "text": "⚠ API 키가 틀렸어요 (.env의 ANTHROPIC_API_KEY)", "t": time.time()})
        except anthropic.RateLimitError:
            self.chat.append({"role": "assistant", "text": "⚠ 요청이 너무 많아요. 잠시 뒤에 다시 해 주세요.", "t": time.time()})
        except anthropic.APIStatusError as e:
            self.chat.append({"role": "assistant", "text": f"⚠ API 오류 {e.status_code}: {e.message[:120]}", "t": time.time()})
        except anthropic.APIConnectionError:
            self.chat.append({"role": "assistant", "text": "⚠ 인터넷 연결을 확인해 주세요.", "t": time.time()})
        except Exception as e:
            self.chat.append({"role": "assistant", "text": f"⚠ 오류: {type(e).__name__}: {e}", "t": time.time()})
        finally:
            self.chat[:] = self.chat[-60:]
            self.busy = False
