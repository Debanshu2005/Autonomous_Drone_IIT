import asyncio
import json
import logging
import socket
import time
from dataclasses import dataclass
from typing import Optional

LOGGER = logging.getLogger(__name__)

@dataclass(frozen=True)
class HotspotContainmentConfig:
    enabled: bool = False
    max_radius_m: float = 25.0
    network_watchdog_enabled: bool = False
    drone_id: str = "drone-1"
    expected_peer_ids: Optional[list[str]] = None
    udp_port: int = 50555
    broadcast_ip: str = "255.255.255.255"
    peer_unicast_ips: Optional[list[str]] = None
    heartbeat_interval_s: float = 1.0
    peer_timeout_s: float = 5.0
    require_peers_before_arm: bool = True
    
    def __post_init__(self):
        if self.expected_peer_ids is None:
            object.__setattr__(self, 'expected_peer_ids', [])
        if self.peer_unicast_ips is None:
            object.__setattr__(self, 'peer_unicast_ips', [])


class PeerHeartbeatProtocol(asyncio.DatagramProtocol):
    def __init__(self, drone_id: str) -> None:
        self.drone_id = drone_id
        self.last_seen_s: dict[str, float] = {}

    def datagram_received(self, data: bytes, addr: tuple[str, int]) -> None:
        try:
            message = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return

        if message.get("type") != "drone_heartbeat":
            return

        peer_id = str(message.get("id", ""))
        if not peer_id or peer_id == self.drone_id:
            return

        self.last_seen_s[peer_id] = time.monotonic()
        LOGGER.debug("Peer heartbeat from %s at %s", peer_id, addr[0])


class PeerLink:
    def __init__(self, config: HotspotContainmentConfig) -> None:
        self.config = config
        self.protocol = PeerHeartbeatProtocol(config.drone_id)
        self.transport: Optional[asyncio.DatagramTransport] = None
        self._task: Optional[asyncio.Task[None]] = None

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        transport, _ = await loop.create_datagram_endpoint(
            lambda: self.protocol,
            local_addr=("0.0.0.0", self.config.udp_port),
            allow_broadcast=True,
        )
        self.transport = transport
        self._task = asyncio.create_task(self._send_heartbeats())
        LOGGER.info(
            "Hotspot peer heartbeat active as %s on UDP %d",
            self.config.drone_id,
            self.config.udp_port,
        )

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)
        if self.transport:
            self.transport.close()

    def missing_peers(self) -> list[str]:
        now = time.monotonic()
        missing = []
        for peer_id in self.config.expected_peer_ids or []:
            last_seen = self.protocol.last_seen_s.get(peer_id, 0.0)
            if now - last_seen > self.config.peer_timeout_s:
                missing.append(peer_id)
        return missing

    async def _send_heartbeats(self) -> None:
        assert self.transport is not None
        while True:
            payload = json.dumps(
                {
                    "type": "drone_heartbeat",
                    "id": self.config.drone_id,
                    "sent_at": time.time(),
                },
                separators=(",", ":"),
            ).encode("utf-8")

            targets = [(self.config.broadcast_ip, self.config.udp_port)]
            targets.extend((ip, self.config.udp_port) for ip in (self.config.peer_unicast_ips or []))
            for target in targets:
                try:
                    self.transport.sendto(payload, target)
                except (OSError, socket.error):
                    LOGGER.exception("Failed to send heartbeat to %s:%d", *target)
            await asyncio.sleep(self.config.heartbeat_interval_s)
