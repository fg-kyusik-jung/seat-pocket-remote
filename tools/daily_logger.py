"""좌판 압력 데이터 백그라운드 로거.

pilot3.ino의 시리얼 디버그 스트림(TOP,...,M5,...,RAW,...)을 계속 받아서
착석/이석, 바른자세/불량자세(+패턴), 스트레칭모드, 전원 On/Off 이벤트를
tools/logs/seat_pocket.db(SQLite)에 누적 기록한다.

기록과 동시에 export_report.py의 로직을 재사용해서 오늘자 리포트 JSON
(tools/logs/report_YYYY-MM-DD.json)과 전체 기록(history.json)을 종료(Ctrl+C) 시
자동 생성한다 - export_report.py를 따로 실행할 필요 없음.

사용법:
    python tools/daily_logger.py [COM포트]   (기본 COM3, 115200 baud)
    종료: q 키를 누르거나 Ctrl+C (PowerShell에서 Ctrl+C가 바로 안 먹는 경우가 있어서
    q 키 방식을 기본으로 추가함 - 창에 포커스 있는 상태로 q 누르면 바로 종료됨).
"""

import json
import msvcrt
import re
import sqlite3
import sys
import time
from datetime import datetime
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).parent))
import export_report  # noqa: E402  (위 sys.path 조작 이후에 임포트해야 함)

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"
BAUD = 115200
DB_PATH = Path(__file__).parent / "logs" / "seat_pocket.db"
SAMPLE_COMMIT_INTERVAL_S = 2.0

# ---- 실측 후 튜닝 필요 (펌웨어 쪽 POSTURE/POSTURECONFIRM 기본값과 맞춰둠) ----
PRESENCE_RAW = 130          # 이 이상이면 "무언가 눌려있다"(착석 가능성)로 봄
PRESENCE_DEBOUNCE_S = 3.0   # 착석<->이석 전이로 확정하기까지 유지돼야 하는 시간
POSTURE_THRESHOLD = 40      # 기준압 대비 편차 임계값
POSTURE_CONFIRM_S = 2.0     # 편차가 이 시간 이상 지속돼야 불량자세로 확정

# pilot3.ino 실제 enum 순서와 맞춰야 함 (CalState/Mode3State/Mode4State/Mode5Sub)
CAL_ANALYZING = 1
M3_ADJUST = 2
M4_ACTIVE_STATES = {3, 4, 5}  # BOOSTING, HOLD, RETURNING
M5_IDLE = 0

LINE_RE = re.compile(
    r"TOP,(\d+),MODE,(-?\d+),CAL,(\d+),AS,(\d+),M2,(\d+),M3,(\d+),M4,(\d+),M5,(\d+),"
    r"RAW,(\d+),(\d+),(\d+),(\d+)"
)


def parse_line(line):
    m = LINE_RE.search(line)
    if not m:
        return None
    g = m.groups()
    return {
        "top": int(g[0]), "mode": int(g[1]), "cal": int(g[2]), "as": int(g[3]),
        "m2": int(g[4]), "m3": int(g[5]), "m4": int(g[6]), "m5": int(g[7]),
        "raw": [int(g[8]), int(g[9]), int(g[10]), int(g[11])],
    }


def is_active_phase(rec):
    """이 상태들에서는 압력 변화가 사람의 움직임이 아니라 펌웨어 자체 동작
    때문이므로, 착석/이석·자세 판정을 하지 않고 직전 상태를 그대로 유지한다."""
    return (
        rec["cal"] == CAL_ANALYZING
        or rec["m3"] == M3_ADJUST
        or rec["m4"] in M4_ACTIVE_STATES
        or rec["m5"] != M5_IDLE
    )


def classify_pattern(baseline, raw):
    """대시보드에서 쓰는 것과 동일한 분류 규칙.
    포켓 배치 가정: 1=앞왼쪽, 2=앞오른쪽, 3=뒤왼쪽, 4=뒤오른쪽 (2x2)."""
    dev = [raw[i] - baseline[i] for i in range(4)]
    front = dev[0] + dev[1]   # 포켓1+2
    back = dev[2] + dev[3]    # 포켓3+4
    left = dev[0] + dev[2]    # 포켓1+3
    right = dev[1] + dev[3]   # 포켓2+4
    candidates = {"forward": front, "back": back, "side": abs(left - right)}
    best = max(candidates, key=candidates.get)
    return best if candidates[best] > POSTURE_THRESHOLD else "other"


class Db:
    def __init__(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.conn = sqlite3.connect(path, check_same_thread=False)
        self.conn.execute("""
            CREATE TABLE IF NOT EXISTS raw_samples (
                ts REAL, r0 INT, r1 INT, r2 INT, r3 INT,
                top INT, mode INT, cal INT, as_state INT,
                m2 INT, m3 INT, m4 INT, m5 INT
            )
        """)
        self.conn.execute("""
            CREATE TABLE IF NOT EXISTS events (
                ts REAL, type TEXT, detail TEXT
            )
        """)
        self.conn.commit()
        self._last_commit = 0.0

    def insert_sample(self, ts, rec):
        self.conn.execute(
            "INSERT INTO raw_samples VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)",
            (ts, *rec["raw"], rec["top"], rec["mode"], rec["cal"], rec["as"],
             rec["m2"], rec["m3"], rec["m4"], rec["m5"]),
        )
        if ts - self._last_commit >= SAMPLE_COMMIT_INTERVAL_S:
            self.conn.commit()
            self._last_commit = ts

    def insert_event(self, ts, ev_type, detail=""):
        self.conn.execute("INSERT INTO events VALUES (?,?,?)", (ts, ev_type, detail))
        self.conn.commit()
        stamp = time.strftime("%H:%M:%S", time.localtime(ts))
        print(f"[{stamp}] {ev_type} {detail}".rstrip())


class SeatTracker:
    """RAW/모드 스트림을 받아 착석/이석/자세/스트레칭 이벤트를 판정해 DB에 기록."""

    def __init__(self, db):
        self.db = db
        self.occupied = False
        self.pending_state = None
        self.pending_since = None
        self.baseline = None
        self.posture_bad = False
        self.posture_pending_since = None
        self.stretching = False

    def update(self, ts, rec):
        self.db.insert_sample(ts, rec)

        stretching_now = rec["m5"] != M5_IDLE
        if stretching_now and not self.stretching:
            self.stretching = True
            self.db.insert_event(ts, "stretch_start")
        elif not stretching_now and self.stretching:
            self.stretching = False
            self.db.insert_event(ts, "stretch_end")
        if stretching_now:
            return  # 스트레칭 구간은 착석/자세 판정에서 완전히 제외

        if is_active_phase(rec):
            return  # 펌웨어가 능동으로 압력을 바꾸는 중 - 판정 보류

        self._update_occupancy(ts, rec)
        if not self.occupied or self.baseline is None:
            return
        self._update_posture(ts, rec)

    def _update_occupancy(self, ts, rec):
        occupied_now = max(rec["raw"]) > PRESENCE_RAW
        if occupied_now == self.occupied:
            self.pending_state = None
            return
        if self.pending_state != occupied_now:
            self.pending_state = occupied_now
            self.pending_since = ts
            return
        if ts - self.pending_since < PRESENCE_DEBOUNCE_S:
            return
        self.occupied = occupied_now
        self.pending_state = None
        if occupied_now:
            self.baseline = list(rec["raw"])
            self.db.insert_event(ts, "session_start")
        else:
            self.db.insert_event(ts, "session_end")
            if self.posture_bad:
                self.posture_bad = False
                self.posture_pending_since = None
                self.db.insert_event(ts, "posture_bad_end")

    def _update_posture(self, ts, rec):
        dev = max(abs(rec["raw"][i] - self.baseline[i]) for i in range(4))
        bad_now = dev > POSTURE_THRESHOLD
        if bad_now == self.posture_bad:
            self.posture_pending_since = None
            return
        if self.posture_pending_since is None:
            self.posture_pending_since = ts
            return
        if ts - self.posture_pending_since < POSTURE_CONFIRM_S:
            return
        self.posture_bad = bad_now
        self.posture_pending_since = None
        if bad_now:
            pattern = classify_pattern(self.baseline, rec["raw"])
            self.db.insert_event(ts, "posture_bad_start", pattern)
        else:
            self.db.insert_event(ts, "posture_bad_end")


def auto_export():
    """오늘자 리포트 JSON과, 지금까지 쌓인 전체 기록(history.json)을 새로 만들어 덮어쓴다.
    아직 세션이 하나도 없는 등 이유로 실패해도 로깅 자체를 막지 않도록 조용히 넘어감."""
    date_str = datetime.now().strftime("%Y-%m-%d")
    try:
        report = export_report.build_report(date_str)
        out_path = DB_PATH.parent / f"report_{date_str}.json"
        out_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"[자동 내보내기] {out_path.name} 갱신됨 "
              f"(세션 {len(report['sessions'])}개, 구간 {len(report['segments'])}개)")
    except Exception as e:  # noqa: BLE001 - 리포트 생성 실패가 로깅을 막으면 안 됨
        print(f"[자동 내보내기 실패] {e}")

    try:
        history = export_report.build_history()
        hist_path = DB_PATH.parent / "history.json"
        hist_path.write_text(json.dumps(history, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"[자동 내보내기] history.json 갱신됨 ({len(history['days'])}일치)")
    except Exception as e:  # noqa: BLE001
        print(f"[history 자동 내보내기 실패] {e}")


def stop_key_pressed():
    """PowerShell에서 Ctrl+C가 바로 안 먹힐 때를 대비한 대체 종료 수단.
    q 키가 눌려있으면 True (버퍼에 쌓인 다른 키 입력은 그냥 비움)."""
    pressed = False
    while msvcrt.kbhit():
        key = msvcrt.getch()
        if key in (b"q", b"Q"):
            pressed = True
    return pressed


def main():
    db = Db(DB_PATH)
    tracker = SeatTracker(db)
    print(f"로깅 시작 - 포트 {PORT} / DB {DB_PATH} (종료: q 키 또는 Ctrl+C)")

    try:
        while True:
            try:
                ser = serial.Serial(PORT, BAUD, timeout=1)
                db.insert_event(time.time(), "poweron")
                time.sleep(2.0)
                ser.reset_input_buffer()
                ser.write(b"DEBUGPRINT,1\n")
                while True:
                    if stop_key_pressed():
                        raise KeyboardInterrupt
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        rec = parse_line(line)
                        if rec:
                            tracker.update(time.time(), rec)
            except serial.SerialException:
                db.insert_event(time.time(), "poweroff")
                print("연결 끊김 - 5초 후 재시도...")
                time.sleep(5.0)
    except KeyboardInterrupt:
        print("\n종료합니다.")
        auto_export()


if __name__ == "__main__":
    main()
