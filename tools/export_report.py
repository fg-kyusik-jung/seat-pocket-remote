"""daily_logger.py가 쌓은 SQLite 기록에서 리포트 JSON을 만든다.

이 JSON을 seat_pocket_daily_report.html 대시보드의 "리포트 파일 불러오기"로 열면
샘플 데이터 대신 실측 데이터로 화면이 채워진다.

사용법:
    python tools/export_report.py [YYYY-MM-DD]   하루치만 (기본값: 오늘 날짜)
    python tools/export_report.py --history       DB에 쌓인 전체 날짜를 묶어서 내보내기
                                                   (tools/logs/history.json, 일/주/월 뷰용)
"""

import json
import sqlite3
import sys
from datetime import datetime, timedelta
from pathlib import Path

DB_PATH = Path(__file__).parent / "logs" / "seat_pocket.db"


def day_bounds(date_str):
    day = datetime.strptime(date_str, "%Y-%m-%d")
    return day.timestamp(), (day + timedelta(days=1)).timestamp()


def minute_of_day(ts):
    dt = datetime.fromtimestamp(ts)
    return dt.hour * 60 + dt.minute


def session_stats(conn, start_ts, end_ts):
    row = conn.execute(
        "SELECT AVG((r0+r1+r2+r3)/4.0) FROM raw_samples WHERE ts >= ? AND ts < ?",
        (start_ts, end_ts),
    ).fetchone()
    avg = round(row[0]) if row[0] is not None else 0

    bad_events = conn.execute(
        "SELECT ts, type FROM events WHERE ts >= ? AND ts < ?"
        " AND type IN ('posture_bad_start','posture_bad_end') ORDER BY ts",
        (start_ts, end_ts),
    ).fetchall()
    bad_min, open_ts = 0.0, None
    for ts, ev in bad_events:
        if ev == "posture_bad_start":
            open_ts = ts
        elif ev == "posture_bad_end" and open_ts is not None:
            bad_min += (ts - open_ts) / 60.0
            open_ts = None
    return avg, round(bad_min)


def build_report(date_str):
    conn = sqlite3.connect(DB_PATH)
    day = datetime.strptime(date_str, "%Y-%m-%d")
    start_ts, end_ts = day_bounds(date_str)

    events = conn.execute(
        "SELECT ts, type, detail FROM events WHERE ts >= ? AND ts < ? ORDER BY ts",
        (start_ts, end_ts),
    ).fetchall()

    segments = []
    sessions = []
    state = {"session_start_min": None, "bad_start_min": None, "bad_pattern": None,
             "poweroff_pending": False, "last_min": None}

    def close_gap(to_min, fill_state):
        if state["last_min"] is not None and to_min > state["last_min"]:
            segments.append({"state": fill_state, "s": state["last_min"], "e": to_min})
        state["last_min"] = to_min

    for ts, ev_type, detail in events:
        m = minute_of_day(ts)

        if ev_type == "session_start":
            close_gap(m, "poweroff" if state["poweroff_pending"] else "away")
            state["poweroff_pending"] = False
            state["session_start_min"] = m

        elif ev_type == "session_end":
            close_gap(m, "good")
            if state["session_start_min"] is not None:
                s_ts = (day + timedelta(minutes=state["session_start_min"])).timestamp()
                e_ts = (day + timedelta(minutes=m)).timestamp()
                avg, bad_min = session_stats(conn, s_ts, e_ts)
                sessions.append({"start": state["session_start_min"], "end": m,
                                  "avg": avg, "badMin": bad_min})
            state["session_start_min"] = None

        elif ev_type == "posture_bad_start":
            close_gap(m, "good")
            state["bad_start_min"] = m
            state["bad_pattern"] = detail or "other"

        elif ev_type == "posture_bad_end":
            if state["bad_start_min"] is not None:
                segments.append({"state": "bad", "s": state["bad_start_min"], "e": m,
                                  "pattern": state["bad_pattern"]})
                state["last_min"] = m
            state["bad_start_min"] = None

        elif ev_type == "stretch_start":
            close_gap(m, "good")

        elif ev_type == "stretch_end":
            close_gap(m, "stretch")

        elif ev_type == "poweroff":
            close_gap(m, "good")
            state["poweroff_pending"] = True

        elif ev_type == "poweron":
            if not state["poweroff_pending"]:
                state["last_min"] = m  # 로거 최초 시작 등 - 갭 없이 기준점만 갱신

    zone_row = conn.execute(
        "SELECT AVG(r0), AVG(r1), AVG(r2), AVG(r3) FROM raw_samples WHERE ts >= ? AND ts < ?",
        (start_ts, end_ts),
    ).fetchone()
    zone_pressure = [round(v) if v is not None else 0 for v in zone_row]

    return {
        "date": date_str,
        "segments": segments,
        "sessions": sessions,
        "zonePressure": zone_pressure,
    }


def build_history():
    """DB에 실제로 기록이 있는 첫날부터 마지막날까지, 하루도 안 빼고 날짜별로 묶어서 반환.
    (기록이 없는 날도 "그날은 0분 사용"이라는 의미가 있어서 건너뛰지 않고 포함시킴)"""
    conn = sqlite3.connect(DB_PATH)
    row = conn.execute("SELECT MIN(ts), MAX(ts) FROM raw_samples").fetchone()
    if row[0] is None:
        return {"generatedAt": datetime.now().isoformat(), "days": []}

    first_day = datetime.fromtimestamp(row[0]).date()
    last_day = datetime.fromtimestamp(row[1]).date()
    days = []
    d = first_day
    while d <= last_day:
        days.append(build_report(d.strftime("%Y-%m-%d")))
        d += timedelta(days=1)
    return {"generatedAt": datetime.now().isoformat(), "days": days}


if __name__ == "__main__":
    if not DB_PATH.exists():
        print(f"DB가 없습니다: {DB_PATH} - 먼저 daily_logger.py를 실행해서 기록을 쌓아주세요.")
        sys.exit(1)

    if len(sys.argv) > 1 and sys.argv[1] == "--history":
        history = build_history()
        out_path = DB_PATH.parent / "history.json"
        out_path.write_text(json.dumps(history, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"저장됨: {out_path} ({len(history['days'])}일치)")
    else:
        date_arg = sys.argv[1] if len(sys.argv) > 1 else datetime.now().strftime("%Y-%m-%d")
        report = build_report(date_arg)
        out_path = DB_PATH.parent / f"report_{date_arg}.json"
        out_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"저장됨: {out_path}")
        print(f"세션 {len(report['sessions'])}개, 구간 {len(report['segments'])}개")
