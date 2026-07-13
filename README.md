# 좌판 경도 조절 시스템 - 실험/파일럿 인프라

본 시스템 개발 전, 파라미터 실측 및 매뉴얼 모드 하드웨어 UX를 검증하기 위한 파일럿 펌웨어 모음.

## 구성

```
firmware/experiment/experiment.ino       [파일럿 1] 모터/센서 실험값 측정용 (NUM_SECTORS 기반, 최소구성 1섹터)
firmware/manual_pilot/manual_pilot.ino   [파일럿 2] 매뉴얼 모드 하드웨어(엔코더+포켓선택) 검증용 (NUM_SECTORS=2)
server/bridge.py                         시리얼<->웹소켓 중계 + CSV 로깅 + 대시보드 정적 서빙 (파일럿 1용)
server/requirements.txt                  파이썬 의존성
web/index.html                           실시간 대시보드 (파일럿 1용)
```

개발 중에는 파일명/폴더명을 고정해서 사용 (아두이노 컴파일, 대시보드 기본 URL 등이 파일명에 의존하기 때문). 사용자가 체크포인트를 요청하는 시점에 `YYYYMMDD_HHMM` 이름의 상위 폴더로 firmware/server/web 전체를 복사해 보관 (내부 파일명은 그대로 유지).

---

## 파일럿 1: 모터/센서 실험 (`firmware/experiment`)

## 최소 배선 (1펌프 + 1밸브 + 1센서 + 가변저항 + 부저)

- 밸브(8채널 5V 릴레이 모듈 SZH-RLBG-009의 IN) → Arduino D2
- 펌프 (L9110S 경유) → Arduino D9 (A입력), D10 (B입력)
- 압력센서 아날로그 출력 → Arduino A0
- 가변저항(WH148) 가운데 단자 → Arduino A1 (펌프 속도 조절용)
- 부저 → Arduino D4

섹터를 늘릴 때는 `experiment.ino`의 `NUM_SECTORS`, `VALVE_PINS[]`, `SENSOR_PINS[]`, `alarmRawThreshold[]`만 확장하면 됨 (하드코딩 없음).

**릴레이 배선/극성 (SZH-RLBG-009, 2026-07-10 최종 확정)**: 밸브(+)는 릴레이 **NO** 단자에, 공급전원(+)은 **COM**에 연결. 릴레이는 **액티브하이**(IN=5V일 때 에너자이즈되어 COM-NO 연결/밸브 통전, IN=0V/GND일 때 비활성으로 COM-NC 연결/밸브 비통전 — 판매처 디바이스마트 확인). `setValve()`: state=1(열림/통전)일 때 **HIGH**, state=0(닫힘/비통전)일 때 **LOW** 출력, 부팅 기본값 LOW(안전).

(이 결선/극성은 디버깅 중 여러 번 정정을 거쳤음 — 배선 바꾸실 때는 반드시 이 문단 기준으로 맞춰볼 것.)

**펌프 속도 제어**: 가변저항이 PWM 듀티(0~255)를 직접 결정. 소프트웨어(대시보드)는 방향(정/역방향)만 지정. 대시보드는 현재 속도를 0~100%로 표시.

**경보 부저**: 섹터 raw 값이 임계값(`alarmRawThreshold`) 이상이면 즉시 440Hz 비프음 발생 (노이즈 필터링 없이 즉시 반응), 아래로 내려가면 즉시 정지. 기본값은 ADC 풀스케일(1023)을 40kPa로 임시 가정한 것이라, raw→kPa 변환식이 실측되기 전까지는 실제 40kPa 지점과 다를 수 있음 — `ALARM,<sector_idx>,<raw>` 명령으로 실시간 조정.

## 실행 방법

1. **랩탑에서 펌웨어 업로드** (arduino-cli)
   ```
   arduino-cli compile --fqbn arduino:avr:uno firmware/experiment
   arduino-cli upload -p COM3 --fqbn arduino:avr:uno firmware/experiment
   ```

2. **파이썬 서버 실행** (랩탑, 아두이노 USB 연결된 상태)
   ```
   cd server
   pip install -r requirements.txt
   python bridge.py --port COM3
   ```
   포트를 생략하면 자동 탐색 시도.

3. **브라우저에서 대시보드 접속**
   ```
   http://localhost:8000/
   ```
   (CSV 다운로드 버튼이 상대경로이므로 file://로 직접 열지 말고 위 주소로 접속할 것)

## 시리얼 프로토콜

- Arduino → PC: `T,<millis>,<sector0_raw>,<sector1_raw>,...` (50ms 주기, raw = analogRead 0~1023)
- Arduino → PC: `M,<pump_speed 0-255>,<pump_dir 0|1>` (50ms 주기, 가변저항으로 결정된 현재 펌프 속도)
- Arduino → PC: `READY,<NUM_SECTORS>` (부팅 시 / STATUS 요청 시)
- PC → Arduino: `VALVE,<sector_idx>,<0|1>` (0=닫힘, 1=열림)
- PC → Arduino: `PUMP,<dir 0|1>` (속도는 가변저항이 결정, 소프트웨어는 방향만 지정)
- PC → Arduino: `ALARM,<sector_idx>,<raw_threshold>` (경보 raw 임계값 조정)
- PC → Arduino: `STATUS`

## 로그

`server/logs/log_<타임스탬프>.csv` 로 매 실행마다 새 파일 생성. 대시보드의 "CSV 다운로드"는 현재 실행 중인 로그 파일을 받아옴.

## 현재까지 미확정 (실험으로 결정할 항목)

- 펌프 실제 회전 방향 (극성)
- 실사용 목표 압력 범위
- 압력센서 노이즈 특성 / raw→kPa 변환식
- 밸브 개폐 시 하중 상태에서의 자연 배기 속도
- 능동형 모드 오차범위, 각 포켓 최소 안전 압력 하한

이 값들이 실측되기 전까지 본 시스템(능동형/실시간/프리셋 모드)의 상태 머신 구현은 보류.

---

## 파일럿 2: 매뉴얼 모드 하드웨어 검증 (`firmware/manual_pilot`)

가변저항 1개 + 서브스위치로 4포켓을 선택/조절하면 전환 시 "포인트/밸류 점프"가 발생하는 설계상 결함을 발견하여, 엔코더(EC11, 상대회전값) 기반으로 대체한 것을 검증하는 파일럿. 대시보드 없이 하드웨어 단독으로 동작 (Serial Monitor로 임계값 튜닝만 가능).

### 배선 (2포켓 축소 구성)

- 엔코더(EC11) CLK → D2 (인터럽트), DT → D3, SW → D5
- 부저 → D4
- 밸브 0/1 (8채널 5V 릴레이 모듈 SZH-RLBG-009의 IN) → D6 / D7
- LED 0/1 (포켓 인디케이터) → D8 / D11
- 펌프 (L9110S) → D9 (A입력), D10 (B입력, 고정 방향만 사용)
- 포텐셔미터(펌프 속도) → A2
- 압력센서 0/1 → A0 / A1

**릴레이 배선/극성**: 파일럿 1과 동일한 SZH-RLBG-009. 밸브 전원은 릴레이 **NO** 단자에, 공급전원(+)은 **COM**에 연결 (자세한 극성 설명은 파일럿 1 섹션 참고).

### 동작 시나리오

1. 엔코더 스위치 롱프레스(≥`LONG_PRESS_MS`=1000ms) → 매뉴얼 모드 진입, 0번 포켓 선택 + LED0 점등
2. CW로 돌리는 동안(연속 회전, `HOLD_TIMEOUT_MS`=200ms 이내) → 선택 포켓 밸브 닫힘(펌프측 연결) + 펌프 작동(속도는 포텐셔미터가 결정)
3. 최대압력 도달 → 펌프 정지 + 440Hz 비프
4. CCW로 돌리는 동안에만 → 선택 포켓 밸브 열림(배기), 회전 멈추면 즉시 밸브 잠김
5. 최저압력 도달 → 밸브 강제 잠금 + 440Hz 비프 (CW의 최대압력 도달과 대칭 처리)
6. 스위치 숏프레스 → 다음 포켓으로 전환 (이전 포켓 밸브/펌프는 안전하게 정지 후 전환)

CW/CCW 모두 "돌리는 동안에만" 동작하는 홀드 방식으로 통일 (원 시나리오는 CCW만 명시했으나 대칭성/안전성을 위해 CW도 동일하게 적용 — 사용자 확인됨).

### 시리얼 명령 (튜닝용)

- `MAXTH,<idx>,<raw>` — 포켓별 최대압력 raw 임계값 조정 (기본값 1023 = 임시 가정치)
- `MINTH,<idx>,<raw>` — 포켓별 최저압력 raw 임계값 조정 (기본값 0 = 임시 가정치)

### 아직 미확정

- `PUMP_FIXED_DIR` (펌프 고정 회전 방향) — 파일럿 1 실험 결과로 확정 예정
- `alarmMaxRawThreshold` / `alarmMinRawThreshold` 실제 값 — raw→kPa 변환식 실측 후 조정
