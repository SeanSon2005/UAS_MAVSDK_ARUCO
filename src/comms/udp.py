from __future__ import annotations

import asyncio
import logging
import socket
import time
from collections.abc import Callable

from comms.protocol import (
    CONTROL_STATUS_TYPE,
    PROTO_VERSION,
    ControlStatus,
    VisionSample,
    decode_control_status,
    decode_vision_sample,
    encode_payload,
)
from flight.state import VelocityCommand


class VisionDatagramProtocol(asyncio.DatagramProtocol):
    def __init__(self, on_sample: Callable[[VisionSample], None], logger: logging.Logger):
        self._on_sample = on_sample
        self._logger = logger

    def datagram_received(self, data: bytes, addr) -> None:
        sample = decode_vision_sample(data)
        if sample is None:
            self._logger.debug("Dropped malformed vision packet from %s", addr)
            return
        self._on_sample(sample)


class VisionUdpReceiver:
    def __init__(self, host: str, port: int, logger: logging.Logger):
        self._host = host
        self._port = port
        self._logger = logger
        self._transport = None

    async def start(self, on_sample: Callable[[VisionSample], None]) -> None:
        loop = asyncio.get_running_loop()
        self._transport, _ = await loop.create_datagram_endpoint(
            lambda: VisionDatagramProtocol(on_sample=on_sample, logger=self._logger),
            local_addr=(self._host, self._port),
        )

    def stop(self) -> None:
        if self._transport is not None:
            self._transport.close()
            self._transport = None


class ControlStatusUdpSender:
    def __init__(self, host: str, port: int):
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)

    def publish(self, phase: str, cmd: VelocityCommand, alt_m: float, target_visible: bool) -> None:
        status = ControlStatus(
            v=PROTO_VERSION,
            type=CONTROL_STATUS_TYPE,
            phase=phase,
            vx=cmd.vx,
            vy=cmd.vy,
            vz=cmd.vz,
            alt_m=alt_m,
            target_visible=target_visible,
            t_mono_ns=time.monotonic_ns(),
        )
        try:
            self._sock.sendto(encode_payload(status), self._addr)
        except (BlockingIOError, OSError):
            pass

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


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


class ControlStatusReceiver:
    def __init__(self, host: str, port: int):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, port))
        self._sock.setblocking(False)
        self.latest: ControlStatus | None = None

    def poll(self) -> ControlStatus | None:
        while True:
            try:
                data, _ = self._sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError:
                break
            status = decode_control_status(data)
            if status is not None:
                self.latest = status
        return self.latest

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass
