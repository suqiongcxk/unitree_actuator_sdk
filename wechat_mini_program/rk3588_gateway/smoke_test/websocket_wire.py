#!/usr/bin/env python3
"""WebSocket 冒烟测试使用的最小帧编解码工具（仅依赖 Python 标准库）。"""

from __future__ import annotations

import os
import struct
from typing import List, Tuple


def encode_frame(payload: bytes, opcode: int = 0x1, masked: bool = False) -> bytes:
    """编码单个 FIN WebSocket 帧；客户端发送时必须 masked=True。"""
    first = 0x80 | (opcode & 0x0F)
    length = len(payload)
    mask_bit = 0x80 if masked else 0
    if length < 126:
        header = struct.pack("!BB", first, mask_bit | length)
    elif length <= 0xFFFF:
        header = struct.pack("!BBH", first, mask_bit | 126, length)
    else:
        header = struct.pack("!BBQ", first, mask_bit | 127, length)

    if not masked:
        return header + payload

    mask = os.urandom(4)
    encoded = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
    return header + mask + encoded


def decode_available_frames(buffer: bytearray) -> List[Tuple[int, bytes]]:
    """从接收缓冲区解析所有完整帧，未收全的尾部留在 buffer 中。"""
    frames: List[Tuple[int, bytes]] = []
    while len(buffer) >= 2:
        first, second = buffer[0], buffer[1]
        if not (first & 0x80):
            raise ValueError("冒烟测试不支持分片 WebSocket 帧")

        opcode = first & 0x0F
        masked = bool(second & 0x80)
        length = second & 0x7F
        offset = 2

        if length == 126:
            if len(buffer) < offset + 2:
                break
            length = struct.unpack("!H", buffer[offset : offset + 2])[0]
            offset += 2
        elif length == 127:
            if len(buffer) < offset + 8:
                break
            length = struct.unpack("!Q", buffer[offset : offset + 8])[0]
            offset += 8

        mask = b""
        if masked:
            if len(buffer) < offset + 4:
                break
            mask = bytes(buffer[offset : offset + 4])
            offset += 4

        end = offset + length
        if len(buffer) < end:
            break

        payload = bytes(buffer[offset:end])
        del buffer[:end]
        if masked:
            payload = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
        frames.append((opcode, payload))
    return frames
