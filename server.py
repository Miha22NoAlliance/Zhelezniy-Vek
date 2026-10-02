from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from osm import build_or_load_graph
from routing import find_route

ROOT = Path(__file__).resolve().parent
WEB = ROOT / "web"
CONFIG = ROOT / "config" / "weights.json"
DATA = ROOT / "data" / "lipetsk_graph.json"
GRAPH = None


class Handler(BaseHTTPRequestHandler):
    def _send(self, status: int, body: object, content_type: str = "application/json; charset=utf-8") -> None:
        raw = body if isinstance(body, bytes) else json.dumps(body, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self) -> None:
        global GRAPH
        parsed = urlparse(self.path)

        if parsed.path == "/api/graph":
            try:
                if GRAPH is None:
                    GRAPH = build_or_load_graph(DATA)
                self._send(200, GRAPH["meta"])
            except Exception as exc:
                self._send(500, {"error": str(exc)})
            return

        if parsed.path == "/api/route":
            try:
                if GRAPH is None:
                    GRAPH = build_or_load_graph(DATA)
                q = parse_qs(parsed.query)
                slat = float(q["slat"][0]); slon = float(q["slon"][0])
                glat = float(q["glat"][0]); glon = float(q["glon"][0])
                detour = float(q.get("detour", [1.35])[0])
                self._send(200, find_route(GRAPH, (slat, slon), (glat, glon), detour))
            except Exception as exc:
                self._send(400, {"error": str(exc)})
            return

        if parsed.path == "/api/reload":
            try:
                DATA.unlink(missing_ok=True)
                GRAPH = build_or_load_graph(DATA)
                self._send(200, {"ok": True, "meta": GRAPH["meta"]})
            except Exception as exc:
                self._send(500, {"error": str(exc)})
            return

        if parsed.path == "/api/config":
            try:
                self._send(200, json.loads(CONFIG.read_text(encoding="utf-8")))
            except Exception as exc:
                self._send(500, {"error": str(exc)})
            return

        if parsed.path in {"/", "/index.html"}:
            self._send(200, (WEB / "index.html").read_bytes(), "text/html; charset=utf-8")
            return

        if parsed.path.startswith("/web/"):
            path = ROOT / parsed.path.lstrip("/")
            if path.exists() and path.is_file():
                types = {".js": "application/javascript; charset=utf-8", ".css": "text/css; charset=utf-8"}
                self._send(200, path.read_bytes(), types.get(path.suffix, "application/octet-stream"))
                return

        self._send(404, {"error": "not found"})

    def log_message(self, fmt: str, *args) -> None:
        print(fmt % args)


def main() -> None:
    print("WalkRoute Demo")
    print("http://127.0.0.1:8765")
    ThreadingHTTPServer(("127.0.0.1", 8765), Handler).serve_forever()


if __name__ == "__main__":
    main()
