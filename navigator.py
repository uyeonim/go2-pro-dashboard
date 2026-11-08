"""대시보드용 자율 이동: LiDAR 지도에서 길 찾기(A*) + 로봇 자체 위치(odom)로 따라 걷기.

좌표는 모두 로봇 odom(월드) 좌표 (m). 대시보드의 LiDAR 점·지나온 경로와 같은 좌표라서
지도에서 클릭한 곳이 곧 로봇이 갈 곳.

Navigator(...)에 넘기는 함수:
  get_pose()      -> (x, y, yaw, 받은시각) 또는 None
  get_obstacles() -> (N x 2 numpy 배열[월드 x,y], 받은시각)   몸 높이 근처 LiDAR 점
  send_move(vx, vyaw), send_stop()
"""
import heapq
import math
import threading
import time

import cv2
import numpy as np

RES = 0.08            # 길 찾기 격자 크기 (m)
SELF_R = 0.42         # 로봇 몸 반경: 이 안의 LiDAR 점은 자기 몸/다리로 보고 무시
LETHAL = 0.30         # 장애물에서 이만큼은 절대 안 지나감 (로봇 폭 절반 + 여유)
SOFT = 0.70           # 이 거리 안쪽은 지나갈 수 있지만 되도록 피함 (통로 가운데로)
REACH = 0.15          # 중간 지점 도착 판정
REACH_END = 0.10      # 마지막 지점 도착 판정
MAX_V, MAX_W = 0.6, 0.8
STALE = 0.6           # 위치가 이보다 오래되면 정지
FRONT_STOP = 0.45     # 앞 이 거리 안에 장애물이 있으면 전진 금지
GOAL_TIMEOUT = 120    # 지점 하나에 이보다 오래 걸리면 포기 (s)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clip(v, lo, hi):
    return max(lo, min(hi, v))


# ───────────────────────── 길 찾기 ─────────────────────────
class Grid:
    """LiDAR 점 → 격자. dist = 가장 가까운 장애물까지 거리(m)."""

    def __init__(self, obstacles, start, goal, margin=3.0):
        pts = obstacles if len(obstacles) else np.zeros((0, 2))
        xs = [start[0], goal[0]]
        ys = [start[1], goal[1]]
        self.x0 = min(xs) - margin
        self.y0 = min(ys) - margin
        self.nx = int((max(xs) + margin - self.x0) / RES) + 1
        self.ny = int((max(ys) + margin - self.y0) / RES) + 1
        occ = np.zeros((self.nx, self.ny), np.uint8)
        if len(pts):
            # 로봇 몸 근처 점은 버림 (다리/몸을 장애물로 보는 문제)
            far = np.hypot(pts[:, 0] - start[0], pts[:, 1] - start[1]) > SELF_R
            ij = ((pts[far] - [self.x0, self.y0]) / RES).astype(int)
            ok = (ij[:, 0] >= 0) & (ij[:, 0] < self.nx) & (ij[:, 1] >= 0) & (ij[:, 1] < self.ny)
            ij = ij[ok]
            occ[ij[:, 0], ij[:, 1]] = 1
        free = (1 - occ).astype(np.uint8)
        self.dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * RES if occ.any() else np.full(occ.shape, 99.0, np.float32)

    def cell(self, p):
        return int((p[0] - self.x0) / RES), int((p[1] - self.y0) / RES)

    def world(self, c):
        return self.x0 + (c[0] + 0.5) * RES, self.y0 + (c[1] + 0.5) * RES

    def inside(self, c):
        return 0 <= c[0] < self.nx and 0 <= c[1] < self.ny

    def d(self, c):
        return float(self.dist[c]) if self.inside(c) else 99.0

    def segment_free(self, a, b, need=LETHAL):
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (RES * 0.5)))
        for k in range(n + 1):
            t = k / n
            if self.d(self.cell((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))) < need:
                return False
        return True


def plan(obstacles, start, goal):
    """start→goal 경로 [(x,y), ...] (꺾이는 지점만). 못 찾으면 (None, 이유)."""
    g = Grid(obstacles, start, goal)
    s, t = g.cell(start), g.cell(goal)
    if not g.inside(t):
        return None, "목표가 지도 밖이에요"
    # 목표가 장애물에 너무 붙어 있으면 1m 안에서 가장 가까운 빈 곳으로
    if g.d(t) < LETHAL:
        best = None
        r = int(1.0 / RES)
        for i in range(t[0] - r, t[0] + r + 1):
            for j in range(t[1] - r, t[1] + r + 1):
                if g.inside((i, j)) and g.d((i, j)) >= LETHAL + 0.05:
                    dd = (i - t[0]) ** 2 + (j - t[1]) ** 2
                    if best is None or dd < best[0]:
                        best = (dd, (i, j))
        if best is None:
            return None, "목표가 장애물 안이에요. 빈 곳을 찍어 주세요"
        t = best[1]
    # 출발점이 벽에 붙어 있으면 근처 0.4m는 지나갈 수 있게 (안 그러면 못 빠져나옴)
    start_free = int(0.4 / RES)
    dist = g.dist
    cost_near = np.clip(SOFT - dist, 0, None) * 6.0                # 벽 가까울수록 비쌈
    blocked = dist < LETHAL
    i0, i1 = max(0, s[0] - start_free), min(g.nx, s[0] + start_free + 1)
    j0, j1 = max(0, s[1] - start_free), min(g.ny, s[1] + start_free + 1)
    blocked[i0:i1, j0:j1] = False
    blocked_l = blocked.tolist()
    cost_l = cost_near.tolist()
    # A* (8방향)
    nbrs = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    h = lambda c: math.hypot(c[0] - t[0], c[1] - t[1])
    openq = [(h(s), 0.0, s)]
    came = {s: None}
    gsc = {s: 0.0}
    found = False
    nx, ny = g.nx, g.ny
    while openq:
        _, gc, c = heapq.heappop(openq)
        if c == t:
            found = True
            break
        if gc > gsc.get(c, 1e18):
            continue
        for di, dj, w in nbrs:
            n = (c[0] + di, c[1] + dj)
            if not (0 <= n[0] < nx and 0 <= n[1] < ny) or blocked_l[n[0]][n[1]]:
                continue
            ng = gc + w * (1.0 + cost_l[n[0]][n[1]])
            if ng < gsc.get(n, 1e18):
                gsc[n] = ng
                came[n] = c
                heapq.heappush(openq, (ng + h(n), ng, n))
    if not found:
        return None, "갈 수 있는 길이 없어요 (막혀 있음)"
    cells = []
    c = t
    while c is not None:
        cells.append(c)
        c = came[c]
    cells.reverse()
    pts = [start] + [g.world(c) for c in cells[1:-1]] + [g.world(t)]
    # 직선으로 갈 수 있는 구간은 합쳐서 꺾이는 점만 남김 (벽에서 SOFT 절반 이상 떨어진 직선만)
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not g.segment_free(pts[i], pts[j], need=max(LETHAL, SOFT * 0.6)):
            j -= 1
        out.append(pts[j])
        i = j
    return [(round(x, 3), round(y, 3)) for x, y in out], None


# ───────────────────────── 따라 걷기 ─────────────────────────
class Navigator:
    def __init__(self, get_pose, get_obstacles, send_move, send_stop, log=print):
        self.get_pose, self.get_obstacles = get_pose, get_obstacles
        self.send_move, self.send_stop, self.log = send_move, send_stop, log
        self.lock = threading.Lock()
        self.thread = None
        self._stop = threading.Event()
        self.speed = 0.35
        self._reset()

    def _reset(self):
        self.active = False
        self.goals = []           # 월드 좌표 목표들
        self.loops = 1
        self.loop_i = 0
        self.goal_i = 0
        self.path = []            # 지금 따라가는 길 (월드)
        self.name = ""
        self.msg = ""
        self.result = None        # "done" / "stopped" / "failed"
        self.started = 0.0

    def status(self):
        with self.lock:
            return {"active": self.active, "name": self.name, "msg": self.msg, "result": self.result,
                    "goal_i": self.goal_i, "goals_n": len(self.goals), "loop_i": self.loop_i, "loops": self.loops,
                    "goals": [list(g) for g in self.goals], "path": [list(p) for p in self.path],
                    "speed": self.speed, "elapsed": round(time.time() - self.started, 1) if self.active else None}

    def start(self, goals, loops=1, name="", speed=None):
        self.stop("새 이동 시작", quiet=True)
        if self.thread:
            self.thread.join(2)
        with self.lock:
            self._reset()
            self.goals = [tuple(g) for g in goals]
            self.loops = max(1, int(loops))
            self.name = name
            if speed:
                self.speed = clip(float(speed), 0.1, MAX_V)
            self.active = True
            self.started = time.time()
            self.msg = "길 찾는 중…"
        self._stop.clear()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self, why="정지 버튼", quiet=False):
        was = self.active
        self._stop.set()
        self.send_stop()
        if was:
            with self.lock:
                self.active = False
                self.result = "stopped"
                self.msg = f"멈춤 ({why})"
            if not quiet:
                self.log(f"⏹ 이동 정지: {why}")

    def _finish(self, result, msg):
        self.send_stop()
        with self.lock:
            self.active = False
            self.result = result
            self.msg = msg
        self.log(("✅ " if result == "done" else "⚠ ") + msg)

    def _replan(self, pose, goal):
        obs, _ = self.get_obstacles()
        path, why = plan(obs, (pose[0], pose[1]), goal)
        with self.lock:
            self.path = path[1:] if path else []
        return path, why

    def _path_blocked(self, pose):
        """지금 길이 새 LiDAR 점으로 막혔는지"""
        obs, _ = self.get_obstacles()
        if not self.path or not len(obs):
            return False
        g = Grid(obs, (pose[0], pose[1]), self.path[-1])
        pts = [(pose[0], pose[1])] + self.path
        for a, b in zip(pts, pts[1:]):
            # 로봇 바로 앞 0.5m는 앞 장애물 검사가 따로 맡음 (자기 몸 오검출 방지)
            if math.hypot(b[0] - pose[0], b[1] - pose[1]) < 0.5:
                continue
            if not g.segment_free(a, b, need=LETHAL * 0.9):
                return True
        return False

    def _front_blocked(self, pose):
        obs, _ = self.get_obstacles()
        if not len(obs):
            return False
        dx, dy = obs[:, 0] - pose[0], obs[:, 1] - pose[1]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        fx, fy = c * dx + s * dy, -s * dx + c * dy
        m = (fx > 0.32) & (fx < 0.32 + FRONT_STOP) & (np.abs(fy) < 0.2)
        return bool(m.sum() >= 3)

    def _run(self):
        try:
            self._drive()
        except Exception as e:  # 어떤 오류든 로봇은 세움
            self._finish("failed", f"오류로 멈춤: {type(e).__name__}: {e}")

    def _drive(self):
        for loop_i in range(self.loops):
            for gi, goal in enumerate(self.goals):
                with self.lock:
                    self.loop_i, self.goal_i = loop_i, gi
                last_ok = time.time()
                t_goal = time.time()
                path = None
                t_plan = 0.0
                while not self._stop.is_set():
                    p = self.get_pose()
                    if not p or time.time() - p[3] > STALE:
                        self.send_stop()
                        with self.lock:
                            self.msg = "위치 데이터가 끊겨서 기다리는 중…"
                        if time.time() - last_ok > 15:
                            return self._finish("failed", "위치 데이터가 15초 넘게 끊겨서 멈췄어요")
                        time.sleep(0.2)
                        continue
                    last_ok = time.time()
                    x, y, yaw, _ = p
                    final = gi == len(self.goals) - 1 and loop_i == self.loops - 1
                    if math.hypot(goal[0] - x, goal[1] - y) < (REACH_END if final else REACH * 1.3):
                        break
                    if time.time() - t_goal > GOAL_TIMEOUT:
                        return self._finish("failed", f"지점 {gi + 1}에 {GOAL_TIMEOUT}초 안에 못 갔어요")
                    # 길 찾기: 처음 / 1.5초마다 막혔는지 확인
                    if path is None or (time.time() - t_plan > 1.5 and self._path_blocked(p)):
                        path, why = self._replan(p, goal)
                        t_plan = time.time()
                        if not path:
                            self.send_stop()
                            with self.lock:
                                self.msg = f"{why} — 다시 찾는 중"
                            if time.time() - t_goal > 15:
                                return self._finish("failed", why)
                            time.sleep(0.5)
                            path = None
                            continue
                    elif time.time() - t_plan > 1.5:
                        t_plan = time.time()
                    # 지금 향할 점: 남은 길의 첫 점 (가까우면 다음 점으로)
                    with self.lock:
                        while len(self.path) > 1 and math.hypot(self.path[0][0] - x, self.path[0][1] - y) < REACH:
                            self.path.pop(0)
                        tgt = self.path[0] if self.path else goal
                        remain = math.hypot(tgt[0] - x, tgt[1] - y) + sum(
                            math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.path, self.path[1:]))
                    dx, dy = tgt[0] - x, tgt[1] - y
                    err = wrap(math.atan2(dy, dx) - yaw)
                    vyaw = clip(1.5 * err, -MAX_W, MAX_W)
                    if abs(err) > 0.5:
                        vx = 0.0
                    else:
                        vx = clip(0.8 * remain, 0.12, self.speed) * max(0.0, math.cos(err))
                    blocked = vx > 0 and self._front_blocked(p)
                    if blocked:
                        vx = 0.0
                        path = None          # 다음 바퀴에 다시 길 찾기
                    self.send_move(vx, vyaw)
                    with self.lock:
                        lp = f" · {loop_i + 1}/{self.loops}바퀴" if self.loops > 1 else ""
                        self.msg = ("앞에 장애물 — 돌아갈 길 찾는 중" if blocked else
                                    f"이동 중 · 지점 {gi + 1}/{len(self.goals)}{lp} · 남은 거리 {remain:.1f}m")
                    time.sleep(0.1)
                if self._stop.is_set():
                    return
                self.send_stop()
                time.sleep(0.4)
        n = len(self.goals)
        self._finish("done", f"도착했어요 ({n}개 지점" + (f", {self.loops}바퀴" if self.loops > 1 else "")
                     + f", {time.time() - self.started:.0f}초)")
