#!/usr/bin/env python3
"""不依赖第三方库的 WebSocket 冒烟客户端，用于本机或另一台电脑验证。"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import socket
import time
from urllib.parse import urlparse

from websocket_wire import decode_available_frames, encode_frame


WEBSOCKET_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def receive_http_header(sock: socket.socket) -> tuple[bytes, bytes]:
    data = bytearray()
    while b"\r\n\r\n" not in data:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("服务器在握手期间断开")
        data.extend(chunk)
    header, _separator, remainder = data.partition(b"\r\n\r\n")
    return bytes(header), bytes(remainder)


def main() -> int:
    parser = argparse.ArgumentParser(description="WebSocket 传输层冒烟客户端")
    parser.add_argument("--url", default="ws://127.0.0.1:8765", help="服务器地址")
    parser.add_argument("--timeout", type=float, default=5.0, help="测试超时秒数")
    args = parser.parse_args()

    url = urlparse(args.url)
    if url.scheme != "ws" or not url.hostname:
        raise SystemExit("当前冒烟客户端仅接受 ws://host[:port][/path]")
    port = url.port or 80
    path = url.path or "/"

    key = base64.b64encode(os.urandom(16)).decode("ascii")
    expected_accept = base64.b64encode(
        hashlib.sha1((key + WEBSOCKET_GUID).encode("ascii")).digest()
    ).decode("ascii")

    with socket.create_connection((url.hostname, port), timeout=args.timeout) as sock:
        sock.settimeout(args.timeout)
        request = (
            f"GET {path} HTTP/1.1\r\n"
            f"Host: {url.hostname}:{port}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
        )
        sock.sendall(request.encode("ascii"))
        response_header, first_frame_bytes = receive_http_header(sock)
        response = response_header.decode("latin-1")
        if "101 Switching Protocols" not in response or expected_accept not in response:
            raise RuntimeError("WebSocket 握手响应无效")
        print("[OK] WebSocket 握手成功")

        probe = {
            "type": "transport_probe_v0",
            "client_monotonic_ms": round(time.monotonic() * 1000),
            "message": "hello from client",
        }
        probe_text = json.dumps(probe, ensure_ascii=False, separators=(",", ":"))
        sock.sendall(encode_frame(probe_text.encode("utf-8"), masked=True))

        # TCP 可能把握手响应和第一帧合并返回，保留多读到的字节。
        receive_buffer = bytearray(first_frame_bytes)
        got_status = False
        got_ack = False
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and not (got_status and got_ack):
            frames = decode_available_frames(receive_buffer)
            if not frames:
                receive_buffer.extend(sock.recv(4096))
                frames = decode_available_frames(receive_buffer)
            for opcode, payload in frames:
                if opcode == 0x9:
                    sock.sendall(encode_frame(payload, opcode=0xA, masked=True))
                    continue
                if opcode != 0x1:
                    continue
                message = json.loads(payload.decode("utf-8"))
                print("[收到] " + json.dumps(message, ensure_ascii=False))
                got_status |= message.get("type") == "transport_status_v0"
                got_ack |= message.get("type") == "transport_ack_v0"

        sock.sendall(encode_frame(b"", opcode=0x8, masked=True))
        if not got_status or not got_ack:
            raise RuntimeError(f"测试不完整: status={got_status}, ack={got_ack}")

    print("[PASS] RK3588 主动上报 + 客户端发送 + 服务器 ACK 均正常")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
