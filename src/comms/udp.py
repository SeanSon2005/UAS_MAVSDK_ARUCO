from __future__ import annotations

import socket
from typing import Optional

from comms.protocol import (
    MavlinkStatus,
    VisionSample,
    decode_mavlink_status,
    decode_vision_sample,
    encode_payload,
)


class VisionSampleUdpSender:
    def __init__(self, host: str, port: int):
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)

    def send(self, sample: VisionSample) -> None:
        try:
            self._sock.sendto(encode_payload(sample), self._addr)
        except (BlockingIOError, OSError):
            pass

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


class MavStatusUdpSender:
    def __init__(self, host: str, port: int):
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)

    def send(self, status: MavlinkStatus) -> None:
        try:
            self._sock.sendto(encode_payload(status), self._addr)
        except (BlockingIOError, OSError):
            pass

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


class VisionSampleReceiver:
    def __init__(self, host: str, port: int):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, port))
        self._sock.setblocking(False)
        self.latest: Optional[VisionSample] = None
        self._latest_seq = -1

    def poll(self) -> Optional[VisionSample]:
        while True:
            try:
                data, _ = self._sock.recvfrom(8192)
            except BlockingIOError:
                break
            except OSError:
                break
            sample = decode_vision_sample(data)
            if sample is None:
                continue
            if sample.seq <= self._latest_seq:
                continue
            self._latest_seq = sample.seq
            self.latest = sample
        return self.latest

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


class MavStatusReceiver:
    def __init__(self, host: str, port: int):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, port))
        self._sock.setblocking(False)
        self.latest: Optional[MavlinkStatus] = None

    def poll(self) -> Optional[MavlinkStatus]:
        while True:
            try:
                data, _ = self._sock.recvfrom(8192)
            except BlockingIOError:
                break
            except OSError:
                break
            status = decode_mavlink_status(data)
            if status is not None:
                self.latest = status
        return self.latest

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass
