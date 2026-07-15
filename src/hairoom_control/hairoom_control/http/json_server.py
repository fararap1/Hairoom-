from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable


class ProcedureBusyError(RuntimeError):
    pass


class JsonHandler(BaseHTTPRequestHandler):
    service = None

    def do_GET(self) -> None:
        if self.path != "/health":
            self._send_json(404, {"error": "not_found"})
            return
        self._send_json(200, {"status": "ok"})

    def do_POST(self) -> None:
        if self.path != "/wash":
            self._send_json(404, {"error": "not_found"})
            return

        try:
            response = self.service.run_wash()
        except ProcedureBusyError as exc:
            self._send_json(409, {"status": "busy", "error": str(exc)})
            return
        except Exception as exc:
            self._send_json(500, {"status": "failed", "error": str(exc)})
            return

        self._send_json(200, response)

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _send_json(self, status_code: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def make_server(host: str, port: int, service) -> ThreadingHTTPServer:
    handler: Callable[..., JsonHandler] = type(
        "HairoomJsonHandler",
        (JsonHandler,),
        {"service": service},
    )
    return ThreadingHTTPServer((host, port), handler)
