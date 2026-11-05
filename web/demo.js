// 브라우저 데모: 로봇·서버 없이 가짜 카메라 / 센서 / LiDAR를 만들어 대시보드에 보여줌
// (dashboard.py --demo 의 가짜 데이터와 같은 동작을 자바스크립트로 옮긴 것)
(function () {
  const T0 = performance.now() / 1000;
  const now = () => performance.now() / 1000 - T0;
  const W_SPEED = 0.15;                         // 타원 경로를 천천히 돎
  const trail = [];

  // 가상의 방: 6m x 5m 벽 + 상자 두 개 (월드 좌표, m)
  const room = [];
  for (let x = -3; x < 3; x += 0.03) room.push([x, -2.5], [x, 2.5]);
  for (let y = -2.5; y < 2.5; y += 0.03) room.push([-3, y], [3, y]);
  for (const [[cx, cy], r] of [[[1.6, 1.1], 0.25], [[-1.2, -1.3], 0.3]])
    for (let t = -r; t < r; t += 0.03)
      room.push([cx + t, cy - r], [cx + t, cy + r], [cx - r, cy + t], [cx + r, cy + t]);
  const noisy = () => (Math.random() - 0.5) * 0.02;

  function pose(t) {
    const x = 1.6 * Math.cos(W_SPEED * t), y = 1.0 * Math.sin(W_SPEED * t);
    const vx = -1.6 * W_SPEED * Math.sin(W_SPEED * t), vy = 1.0 * W_SPEED * Math.cos(W_SPEED * t);
    return { x, y, vx, vy, yaw: Math.atan2(vy, vx) };
  }

  // LiDAR 점 → 앞/왼쪽/뒤/오른쪽 가장 가까운 거리 (dashboard.py의 lidar_obstacles와 같은 계산)
  function obstacles(p) {
    const c = Math.cos(p.yaw), s = Math.sin(p.yaw), reach = 4, hw = 0.25, hl = 0.40;
    let f = null, l = null, b = null, r = null;
    const min = (cur, v) => (v < reach && (cur === null || v < cur)) ? v : cur;
    for (const [wx, wy] of room) {
      const dx = wx - p.x, dy = wy - p.y, fx = c * dx + s * dy, fy = -s * dx + c * dy;
      if (fx > 0.30 && Math.abs(fy) < hw) f = min(f, fx);
      if (fx < -0.30 && Math.abs(fy) < hw) b = min(b, -fx);
      if (fy > 0.15 && Math.abs(fx) < hl) l = min(l, fy);
      if (fy < -0.15 && Math.abs(fx) < hl) r = min(r, -fy);
    }
    return [f, l, b, r].map(v => v === null ? null : Math.round(v * 100) / 100);
  }

  function updateTrail(p) {
    const last = trail[trail.length - 1];
    if (!last || Math.hypot(p.x - last[0], p.y - last[1]) > 0.03) trail.push([p.x, p.y]);
    if (trail.length > 3000) trail.splice(0, trail.length - 3000);
  }

  const rnd = v => Math.round(v * 1000) / 1000;

  function data() {
    const t = now();
    if (t < 2) {                                // 처음 2초는 "연결 중" 화면
      return { demo: true, connected: false, phase: 'connecting', attempt: 1, error: null,
               status: '데모: 가상 로봇에 연결 중...', recording: { active: false } };
    }
    const p = pose(t);
    updateTrail(p);
    const battery = 24 - Math.floor(t / 15) % 20;         // 15초마다 1%씩 → 20% 아래 경고 확인용
    const motors = [];
    for (let i = 0; i < 12; i++) {
      let v = 36 + (i % 3) * 3 + 2 * Math.sin(t / 20 + i);
      if (i === 11) v = 45 + 28 * Math.max(0, Math.sin(t / 40));   // RL calf: 과열됐다 식음
      motors.push(Math.round(v));
    }
    return {
      demo: true, connected: true, phase: 'connected', attempt: 0, error: null, status: '연결됨 (데모)',
      video_age: 0.0, sport_age: 0.0, lidar_age: 0.3, recording: { active: false },
      obstacle_lidar: obstacles(p),
      pos: [rnd(p.x), rnd(p.y)], height: 0.32, vel: [rnd(p.vx), rnd(p.vy), 0], yaw_speed: W_SPEED,
      rpy: [rnd(0.01 * Math.sin(t * 3)), rnd(0.02 * Math.sin(t * 2)), rnd(p.yaw)],
      foot_force: [0, 0, 0, 0], obstacle: [0, 0, 0, 0],          // Go2 Pro처럼 안 들어오는 값
      battery, voltage: Math.round((27 + battery / 25) * 10) / 10, motors,
    };
  }

  function lidar() {
    const t = now();
    if (t < 2) return { points: [], age: null, robot: null, yaw: null, trail: [], trail_len: 0 };
    const p = pose(t);
    let len = 0;
    for (let i = 1; i < trail.length; i++)
      len += Math.hypot(trail[i][0] - trail[i - 1][0], trail[i][1] - trail[i - 1][1]);
    return {
      points: room.map(([x, y]) => [x + noisy(), y + noisy()]), age: 0.3,
      robot: [p.x, p.y], yaw: p.yaw, trail: trail.slice(), trail_len: Math.round(len * 100) / 100,
    };
  }

  // 가짜 카메라: 고개 방향에 따라 흘러가는 기둥 + DEMO 글자 (15fps)
  const cv = document.createElement('canvas');
  cv.width = 960; cv.height = 540;
  const g = cv.getContext('2d');
  function drawFrame() {
    const t = now(), p = pose(Math.max(t, 2));
    g.fillStyle = 'rgb(32,38,45)'; g.fillRect(0, 0, 960, 540);
    for (let k = 0; k < 12; k++) {
      const px = ((((k * 160 - p.yaw * 300) % 1920) + 1920) % 1920) - 480;
      g.fillStyle = `rgb(150,120,${90 + k * 10})`; g.fillRect(px, 120, 50, 340);
    }
    g.fillStyle = 'rgb(50,70,40)'; g.fillRect(0, 460, 960, 80);
    g.fillStyle = '#fff'; g.font = 'bold 64px sans-serif'; g.fillText('DEMO', 30, 82);
    g.font = '26px sans-serif'; g.fillStyle = '#e6e6e6';
    const hhmmss = new Date().toTimeString().slice(0, 8);
    g.fillText(`${hhmmss}  x=${p.x >= 0 ? '+' : ''}${p.x.toFixed(2)} y=${p.y >= 0 ? '+' : ''}${p.y.toFixed(2)}`, 30, 515);
  }

  window.Go2Demo = {
    data: async () => data(),
    lidar: async () => lidar(),
    startCamera(show) {
      setInterval(() => { drawFrame(); cv.toBlob(b => b && show(b), 'image/jpeg', 0.8); }, 1000 / 15);
    },
    snapshot: async () => {
      drawFrame();
      const blob = await new Promise(r => cv.toBlob(r, 'image/jpeg', 0.9));
      const stamp = new Date().toISOString().replace(/[-:T]/g, '').slice(0, 14);
      return { ok: true, url: URL.createObjectURL(blob), name: `demo_photo_${stamp}.jpg` };
    },
    recStart: async () => ({ ok: false, error: '데모에서는 녹화할 수 없어요 (실제 대시보드에서 가능)' }),
    recStop: async () => ({ ok: false }),
    recordings: async () => null,                  // null = 데모라서 목록 없음
    trailClear: async () => { trail.length = 0; return { ok: true }; },
  };
})();
