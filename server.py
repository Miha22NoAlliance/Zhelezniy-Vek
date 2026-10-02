from __future__ import annotations

import json
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from osm import PBF_FILENAME, build_or_load_graph
from routing import find_route

ROOT = Path(__file__).resolve().parent
WEB = ROOT / "web"
CONFIG = ROOT / "config" / "weights.json"
DATA_DIR = ROOT / "data"
GRAPH_PATH = DATA_DIR / "lipetsk_graph.json"
MAP_PATH = DATA_DIR / "lipetsk_map.json"
PBF_PATH = DATA_DIR / PBF_FILENAME

GRAPH = None
MAP_DATA = None
LOAD_ERROR = None


def ensure_data():
    global GRAPH, MAP_DATA, LOAD_ERROR
    if GRAPH is not None and MAP_DATA is not None:
        return GRAPH, MAP_DATA
    if LOAD_ERROR is not None:
        raise RuntimeError(LOAD_ERROR)
    try:
        GRAPH, MAP_DATA = build_or_load_graph(GRAPH_PATH, MAP_PATH, PBF_PATH)
        return GRAPH, MAP_DATA
    except Exception as exc:
        LOAD_ERROR = str(exc)
        raise


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
        global GRAPH, MAP_DATA, LOAD_ERROR
        parsed = urlparse(self.path)

        if parsed.path == "/api/status":
            self._send(200, {
                "ready": GRAPH is not None and MAP_DATA is not None,
                "error": LOAD_ERROR,
                "pbf": PBF_FILENAME,
                "offline": True,
            })
            return

        if parsed.path == "/api/graph":
            try:
                graph, _ = ensure_data()
                self._send(200, graph["meta"])
            except Exception as exc:
                self._send(500, {"error": str(exc)})
            return

        if parsed.path == "/api/map":
            try:
                _, map_data = ensure_data()
                self._send(200, map_data)
            except Exception as exc:
                self._send(500, {"error": str(exc)})
            return

        if parsed.path == "/api/route":
            try:
                graph, _ = ensure_data()
                q = parse_qs(parsed.query)
                slat = float(q["slat"][0]); slon = float(q["slon"][0])
                glat = float(q["glat"][0]); glon = float(q["glon"][0])
                detour = float(q.get("detour", [1.35])[0])
                mode = q.get("mode", ["quality"])[0]
                automatic = q.get("auto", ["0"])[0].lower() in {"1", "true", "yes"}
                self._send(200, find_route(
                    graph, (slat, slon), (glat, glon),
                    detour, mode, automatic
                ))
            except KeyError:
                self._send(400, {"error": "Не переданы координаты старта или финиша"})
            except ValueError as exc:
                self._send(400, {"error": str(exc)})
            except Exception as exc:
                print("INTERNAL ROUTE ERROR:")
                traceback.print_exc()
                self._send(500, {"error": f"Внутренняя ошибка сервера: {exc}"})
            return

        if parsed.path == "/api/reload":
            try:
                GRAPH = None
                MAP_DATA = None
                LOAD_ERROR = None
                GRAPH_PATH.unlink(missing_ok=True)
                MAP_PATH.unlink(missing_ok=True)
                graph, map_data = ensure_data()
                self._send(200, {"ok": True, "meta": graph["meta"], "map_roads": len(map_data["roads"])})
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
                types = {
                    ".js": "application/javascript; charset=utf-8",
                    ".css": "text/css; charset=utf-8",
                    ".html": "text/html; charset=utf-8",
                }
                self._send(200, path.read_bytes(), types.get(path.suffix, "application/octet-stream"))
                return

        self._send(404, {"error": "not found"})

    def log_message(self, fmt: str, *args) -> None:
        print(fmt % args)


def main() -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    print("WalkRoute Demo — полностью офлайн")
    print("Build: offline-pbf-v9")
    print(f"Server: {Path(__file__).resolve()}")
    print(f"OSM module: {Path(__import__('osm').__file__).resolve()}")
    print(f"PBF: data/{PBF_FILENAME}")
    print("http://127.0.0.1:8765")
    print("Граф и локальная карта будут построены из PBF при первом обращении.")
    ThreadingHTTPServer(("127.0.0.1", 8765), Handler).serve_forever()


if __name__ == "__main__":
    main()
