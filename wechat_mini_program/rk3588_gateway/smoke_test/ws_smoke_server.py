#!/usr/bin/env python3
"""RK3588 WebSocket 双向通信冒烟服务器；绝不访问电机、串口或控制 FIFO。"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import signal
import socket
import socketserver
import threading
import time
from typing import Dict

from websocket_wire import decode_available_frames, encode_frame


WEBSOCKET_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
STOP_EVENT = threading.Event()


def parse_http_headers(request: bytes) -> Dict[str, str]:
    lines = request.decode("latin-1").split("\r\n")
    headers: Dict[str, str] = {}
    for line in lines[1:]:
        if ":" in line:
            key, value = line.split(":", 1)
            headers[key.strip().lower()] = value.strip()
    return headers


class SmokeHandler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        self.request.settimeout(0.2)
        try:
            request = self._receive_handshake()
            headers = parse_http_headers(request)
            key = headers.get("sec-websocket-key")
            if not key or headers.get("upgrade", "").lower() != "websocket":
                self.request.sendall(b"HTTP/1.1 400 Bad Request\r\nConnection: close\r\n\r\n")
                return

            accept = base64.b64encode(
                hashlib.sha1((key + WEBSOCKET_GUID).encode("ascii")).digest()
            ).decode("ascii")
            response = (
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept}\r\n\r\n"
            )
            self.request.sendall(response.encode("ascii"))
            print(f"[连接] {self.client_address[0]}:{self.client_address[1]}", flush=True)
            self._serve_frames()
        except (ConnectionError, OSError, ValueError) as exc:
            if not STOP_EVENT.is_set():
                print(f"[断开] {self.client_address[0]}: {exc}", flush=True)
        finally:
            print(f"[离线] {self.client_address[0]}", flush=True)

    def _receive_handshake(self) -> bytes:
        data = bytearray()
        deadline = time.monotonic() + 3.0
        while b"\r\n\r\n" not in data:
            if time.monotonic() >= deadline or len(data) > 16384:
                raise ConnectionError("WebSocket 握手超时或请求过大")
            try:
                chunk = self.request.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionError("客户端在握手前断开")
            data.extend(chunk)
        return bytes(data)

    def _send_json(self, message: dict) -> None:
        payload = json.dumps(message, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.request.sendall(encode_frame(payload))

    def _serve_frames(self) -> None:
        receive_buffer = bytearray()
        next_status = time.monotonic()
        sequence = 0
        while not STOP_EVENT.is_set():
            now = time.monotonic()
            if now >= next_status:
                sequence += 1
                self._send_json(
                    {
                        "type": "transport_status_v0",
                        "seq": sequence,
                        "monotonic_ms": round(now * 1000),
                        "hostname": socket.gethostname(),
                        "message": "hello from RK3588",
                    }
                )
                next_status = now + 1.0

            try:
                chunk = self.request.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                return
            receive_buffer.extend(chunk)

            for opcode, payload in decode_available_frames(receive_buffer):
                if opcode == 0x8:  # Close
                    self.request.sendall(encode_frame(payload, opcode=0x8))
                    return
                if opcode == 0x9:  # Ping
                    self.request.sendall(encode_frame(payload, opcode=0xA))
                    continue
                if opcode != 0x1:
                    continue

                text = payload.decode("utf-8", errors="replace")
                print(f"[收到] {self.client_address[0]}: {text}", flush=True)
                self._send_json(
                    {
                        "type": "transport_ack_v0",
                        "received": text,
                        "server_monotonic_ms": round(time.monotonic() * 1000),
                    }
                )


class ThreadingWebSocketServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main() -> int:
    parser = argparse.ArgumentParser(description="RK3588 WebSocket 传输层冒烟服务器（不控制电机）")
    parser.add_argument("--host", default="0.0.0.0", help="监听地址，默认 0.0.0.0")
    parser.add_argument("--port", type=int, default=8765, help="监听端口，默认 8765")
    args = parser.parse_args()

    def request_stop(_signum: int, _frame: object) -> None:
        STOP_EVENT.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    with ThreadingWebSocketServer((args.host, args.port), SmokeHandler) as server:
        server.timeout = 0.2
        print(f"[启动] ws://{args.host}:{args.port}", flush=True)
        print("[安全] 本程序不访问电机、串口、GPIO 或 robot_control FIFO", flush=True)
        while not STOP_EVENT.is_set():
            server.handle_request()
    print("[退出] WebSocket 冒烟服务器已关闭", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
