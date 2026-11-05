# go2-pro-dashboard

Unitree **Go2 Pro**에서 PC로 **센서 · 카메라 · LiDAR** 데이터를 무선(Wi-Fi / WebRTC)으로 받아 웹 대시보드에 실시간으로 보여주는 프로젝트입니다. 연구실 인턴 과제로 만들었습니다.

- 왼쪽: 전면 카메라 영상 (약 12fps) + LiDAR를 위에서 본 지도 (로봇 중심, 반경 4m)
- 오른쪽: 배터리, 위치, 방향(yaw), 속도, 자세(roll/pitch), 발 접촉력, 장애물 거리
- 실행하면 로봇에 자동 연결, 끊기면 자동 재연결

Go2 Air/Pro는 공식 SDK가 열려 있지 않아서, 앱과 같은 WebRTC 방식을 쓰는 커뮤니티 라이브러리 [`unitree_webrtc_connect`](https://github.com/legion1581/unitree_webrtc_connect) (MIT)를 사용합니다.

## 파일

| 파일 | 내용 |
|---|---|
| `dashboard.py` | 웹 대시보드 서버 (Flask). 로봇 연결 1개로 영상·센서·LiDAR를 받아 중계 |
| `web/index.html` | 대시보드 화면 |
| `sensors.py` | 터미널에 센서값 출력 + `logs/`에 CSV 저장 |
| `camera.py` | OpenCV 창으로 카메라 보기, 사진(`s`)·녹화(`r`) → `captures/` |
| `대시보드_실행.bat` | 더블클릭으로 대시보드 실행 |

## 설치

Windows, Python 3.12 기준입니다.

```bash
py -3.12 -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
copy .env.example .env
```

`.env`에 로봇 IP와 AES 키를 채웁니다.

```
UNITREE_ROBOT_IP=<로봇 IP, Unitree Go 앱 네트워크 화면의 wlan0>
UNITREE_AES_128_KEY=<32자리 16진수>
```

## 실행

로봇은 WebRTC 연결을 **한 번에 하나만** 받습니다. 폰 앱과 다른 스크립트는 꺼 두고 하나만 실행하세요.

```bash
.venv\Scripts\python dashboard.py      # 브라우저가 http://127.0.0.1:8000 으로 열림
.venv\Scripts\python sensors.py        # 터미널 출력 + CSV
.venv\Scripts\python camera.py         # 카메라 창
```

## AES 키 (펌웨어 1.1.15 이상)

Go2 펌웨어 1.1.15부터 LAN 연결에 **로봇별 AES-128 키**가 필요합니다. 키는 로봇이 등록된 Unitree 계정의 기기 정보에 들어 있습니다.

- 라이브러리의 `unitree-fetch-aes-key` CLI는 Unitree 서버의 봇 차단(HTTP 567)에 막힐 수 있습니다.
- 브라우저 기반 [`unitree_ui`](https://github.com/legion1581/unitree_ui)의 Account Manager에서 로그인하면 기기 목록에서 키를 확인할 수 있습니다. 로그인한 계정에 로봇이 등록돼 있어야 합니다.
- 키는 로봇마다 고정이라 한 번만 받으면 됩니다. `.env`에만 두고 커밋하지 마세요.

## 자주 나는 문제

| 증상 | 원인 / 해결 |
|---|---|
| `AesKeyRequiredError` | `.env`의 키가 없거나 비어 있음 |
| `RobotBusyError` | 폰 앱 등 다른 클라이언트가 연결 중 → 종료 후 재시도 |
| `LocalSignalingPortError` | 로봇에 닿지 않음 → 같은 Wi-Fi인지, IP가 맞는지 확인 |
| ICE가 `checking`에서 멈춤 / `DataChannelTimeoutError` | PC와 로봇이 다른 Wi-Fi(예: 2.4G/5G)에 있거나 이전 연결이 남아 있음. 같은 Wi-Fi로 맞추고, 안 되면 로봇 재부팅 |
| 카메라가 느리거나 멈춤 | Windows에서 `localhost`가 IPv6를 먼저 시도해 요청마다 ~2초 지연 → 서버가 `::1`도 받도록 처리함. `127.0.0.1` 주소 권장 |
| 장애물 거리가 0.00 m | 로봇의 장애물 회피 기능이 꺼져 있으면 값이 채워지지 않는 것으로 보임 (미확인) |

## 안전

- 이 대시보드는 현재 **보기 전용**이며 로봇을 움직이는 명령을 보내지 않습니다.
- 로봇을 움직이는 테스트를 할 때는 넓은 곳에서, 리모컨 비상정지(**L2 + B**)를 손에 들고 하세요.
