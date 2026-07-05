"""web_monitor.py — live dashboard for session JSONL logs (README §12).

Serves a self-contained HTML page that tails a SessionRecorder
`events.jsonl` and renders: safety-state timeline, shot log with det→fire
latency, cube temperature sparkline, and headline counters. Reads the log
file only — it never touches the hot targeting loop, so it costs the
session nothing.

    python tools/web_monitor.py                  # newest session, live-follow
    python tools/web_monitor.py user_data/sessions/live_fire_20260705T.../
    python tools/web_monitor.py --port 8768

Stdlib only (http.server) — no new dependencies. Binds
WEB_MONITOR_BIND_HOST (default 127.0.0.1); don't expose it wider without
adding auth.
"""
from __future__ import annotations

import argparse
import json
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from config import settings                                       # noqa: E402

_HTML_PATH = Path(__file__).with_name("web_monitor.html")


def _list_sessions(base: Path) -> list[dict]:
    """Session dirs under SESSIONS_DIR that have an events.jsonl,
    newest first."""
    out = []
    if not base.is_dir():
        return out
    for d in base.iterdir():
        ev = d / "events.jsonl"
        if d.is_dir() and ev.is_file():
            out.append({
                "name": d.name,
                "mtime": ev.stat().st_mtime,
                "closed": (d / "session_meta.json").is_file(),
            })
    out.sort(key=lambda s: -s["mtime"])
    return out


def _resolve_session(base: Path, name: Optional[str]) -> Optional[Path]:
    """Map a session name from the query string back to a directory.
    Names come from _list_sessions, so lookup-by-listing also prevents
    path traversal."""
    sessions = _list_sessions(base)
    if not sessions:
        return None
    if name:
        for s in sessions:
            if s["name"] == name:
                return base / name
        return None
    return base / sessions[0]["name"]


def _tail_events(session_dir: Path, offset: int) -> tuple[list, int]:
    """Read complete JSONL lines from `offset` (bytes). Returns
    (records, new_offset). A partial trailing line is left for next poll."""
    path = session_dir / "events.jsonl"
    if not path.is_file():
        return [], 0
    size = path.stat().st_size
    if offset > size:
        offset = 0                       # new/truncated file — restart
    with path.open("rb") as fp:
        fp.seek(offset)
        data = fp.read()
    if not data:
        return [], offset
    lines = data.split(b"\n")
    if not data.endswith(b"\n"):
        consumed = len(data) - len(lines[-1])
        lines = lines[:-1]
        new_offset = offset + consumed
    else:
        new_offset = offset + len(data)
    records = []
    for line in lines:
        line = line.strip()
        if not line:
            continue
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return records, new_offset


class _Handler(BaseHTTPRequestHandler):
    server_version = "NoMoWebMonitor/1.0"

    # Set by serve(): sessions base dir + pinned session (or None = latest).
    base_dir: Path
    pinned: Optional[Path]

    def log_message(self, fmt, *args) -> None:      # quiet by default
        pass

    def _send_json(self, obj, code: int = 200) -> None:
        body = json.dumps(obj, default=repr).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:                       # noqa: N802
        url = urlparse(self.path)
        q = parse_qs(url.query)
        if url.path == "/":
            try:
                body = _HTML_PATH.read_bytes()
            except OSError:
                self.send_error(500, "web_monitor.html missing")
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if url.path == "/api/sessions":
            self._send_json({"sessions": _list_sessions(self.base_dir)})
            return

        if url.path == "/api/events":
            name = (q.get("session", [None])[0]
                    if self.pinned is None else self.pinned.name)
            session = _resolve_session(self.base_dir, name)
            if session is None:
                self._send_json({"error": "no session found"}, code=404)
                return
            offset = int(q.get("offset", ["0"])[0])
            records, new_offset = _tail_events(session, offset)
            meta = None
            meta_path = session / "session_meta.json"
            if meta_path.is_file():
                try:
                    meta = json.loads(meta_path.read_text(encoding="utf-8"))
                except (OSError, json.JSONDecodeError):
                    meta = None
            self._send_json({
                "session": session.name,
                "records": records,
                "offset": new_offset,
                "meta": meta,
            })
            return

        self.send_error(404)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", nargs="?", default=None,
                    help="session dir to pin (default: follow the newest)")
    ap.add_argument("--host", default=settings.WEB_MONITOR_BIND_HOST)
    ap.add_argument("--port", type=int, default=settings.WEB_MONITOR_PORT)
    args = ap.parse_args(argv)

    base = Path(settings.SESSIONS_DIR)
    pinned: Optional[Path] = None
    if args.session is not None:
        pinned = Path(args.session)
        if pinned.name == "events.jsonl":
            pinned = pinned.parent
        if not (pinned / "events.jsonl").is_file():
            print(f"FAIL: no events.jsonl under {pinned}")
            return 1
        base = pinned.parent

    _Handler.base_dir = base
    _Handler.pinned = pinned

    srv = ThreadingHTTPServer((args.host, args.port), _Handler)
    mode = pinned.name if pinned is not None else "following newest session"
    print(f"web monitor: http://{args.host}:{args.port}/  ({mode})")
    print(f"  sessions dir: {base}")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
