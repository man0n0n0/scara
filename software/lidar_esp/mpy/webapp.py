"""Tiny asyncio HTTP server serving the tracker UI and its JSON API."""

import asyncio
import json
import gc

CT = {"html": "text/html", "js": "application/javascript", "css": "text/css",
      "json": "application/json", "bin": "application/octet-stream"}


async def _send(w, status, ctype, body=None, extra=None, length=None):
    head = "HTTP/1.1 %s\r\nContent-Type: %s\r\n" % (status, ctype)
    head += "Cache-Control: no-store\r\nConnection: close\r\n"
    if extra:
        for k, v in extra.items():
            head += "%s: %s\r\n" % (k, v)
    n = length if length is not None else (len(body) if body else 0)
    head += "Content-Length: %d\r\n\r\n" % n
    w.write(head.encode())
    if body:
        w.write(body)
    await w.drain()


async def _send_file(w, path, ctype):
    try:
        size = 0
        with open(path, "rb") as f:                 # two passes, no big RAM buffer
            while True:
                c = f.read(1024)
                if not c:
                    break
                size += len(c)
        await _send(w, "200 OK", ctype, None, length=size)
        with open(path, "rb") as f:
            while True:
                c = f.read(1024)
                if not c:
                    break
                w.write(c)
                await w.drain()
    except OSError:
        await _send(w, "404 Not Found", "text/plain", b"missing file")


class Server:
    def __init__(self, state, port=80):
        self.state = state
        self.port = port
        self.routes_get = {
            "/": self._index,
            "/raw": self._raw,
            "/state": self._state,
            "/profiles": self._profiles,
        }
        self.routes_post = {
            "/api/learn": self._learn,
            "/api/set": self._set,
            "/api/calib/add": self._calib_add,
            "/api/calib/solve": self._calib_solve,
            "/api/calib/clear": self._calib_clear,
            "/api/calib/undo": self._calib_undo,
        }

    async def start(self):
        await asyncio.start_server(self._handle, "0.0.0.0", self.port)

    async def _handle(self, r, w):
        try:
            line = await r.readline()
            if not line:
                return
            parts = line.decode().split()
            if len(parts) < 2:
                return
            method, path = parts[0], parts[1]
            qs = ""
            if "?" in path:
                path, qs = path.split("?", 1)

            clen = 0
            while True:
                h = await r.readline()
                if not h or h in (b"\r\n", b"\n"):
                    break
                hl = h.decode().lower()
                if hl.startswith("content-length:"):
                    clen = int(hl.split(":", 1)[1])
            body = await r.readexactly(clen) if clen else b""

            if method == "GET" and path in self.routes_get:
                await self.routes_get[path](w, qs)
            elif method == "POST" and path in self.routes_post:
                await self.routes_post[path](w, body)
            else:
                await _send(w, "404 Not Found", "text/plain", b"no route")
        except Exception as e:
            try:
                await _send(w, "500 Server Error", "text/plain", str(e).encode())
            except Exception:
                pass
        finally:
            try:
                await w.drain()
                w.close()
                await w.wait_closed()
            except Exception:
                pass
            gc.collect()

    # ---- routes
    async def _index(self, w, qs):
        await _send_file(w, "index.html", CT["html"])

    async def _raw(self, w, qs):
        st = self.state
        buf = st.snapshot()
        if buf is None:
            await _send(w, "503 Service Unavailable", "text/plain", b"no frame")
            return
        await _send(w, "200 OK", CT["bin"], buf,
                    extra={"X-Width": st.w, "X-Height": st.h})

    async def _state(self, w, qs):
        await _send(w, "200 OK", CT["json"], json.dumps(self.state.as_dict()).encode())

    async def _profiles(self, w, qs):
        st = self.state
        d = st.det
        step = max(1, st.w // 160)
        await _send(w, "200 OK", CT["json"], json.dumps({
            "step": step,
            "dpos": [d.dpos[i] for i in range(0, st.w, step)],
            "dneg": [d.dneg[i] for i in range(0, st.w, step)],
        }).encode())

    async def _learn(self, w, body):
        ok = await self.state.relearn()
        await _send(w, "200 OK", CT["json"],
                    json.dumps({"ok": ok, "floor": self.state.det.floor}).encode())

    async def _set(self, w, body):
        try:
            cfg = json.loads(body) if body else {}
        except ValueError:
            cfg = {}
        applied = self.state.apply_settings(cfg)
        await _send(w, "200 OK", CT["json"], json.dumps(applied).encode())

    async def _calib_add(self, w, body):
        p = json.loads(body)
        res = self.state.calib_add(float(p["X"]), float(p["Y"]),
                                   int(p.get("samples", 8)))
        await _send(w, "200 OK", CT["json"], json.dumps(res).encode())

    async def _calib_solve(self, w, body):
        await _send(w, "200 OK", CT["json"], json.dumps(self.state.calib_solve()).encode())

    async def _calib_clear(self, w, body):
        self.state.calib_clear()
        await _send(w, "200 OK", CT["json"], json.dumps({"ok": True}).encode())

    async def _calib_undo(self, w, body):
        self.state.calib_undo()
        await _send(w, "200 OK", CT["json"],
                    json.dumps({"ok": True, "n": len(self.state.calib["src"])}).encode())
