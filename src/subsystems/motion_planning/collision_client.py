"""Planner-side adapter for the shared collision-worker pool.

Status
------
Experimental and not production-ready. This transport/oracle adapter is
partially tested and currently intended for validation workflows.

Responsibilities
----------------
The motion planner only asks whether configurations or discretized edges are
collision-free. This module translates those calls into
`req.collision_check` messages, sends a bounded number concurrently through
the existing ROUTER/DEALER collision broker, matches out-of-order replies by
request id, and restores results to input order.

This module does not create, own, or terminate collision workers. The
standalone validation tool starts a temporary pool; the future production
planner will connect to the pool already owned by the main launcher.

Shared-pool compatibility
-------------------------
Each `CollisionWorkerClient` owns a separate DEALER socket. ZeroMQ assigns it
a unique routing identity, so its request ids may overlap with request ids
from jogging clients without mixing replies. The broker load-balances bundles
from every connected client across whichever REP workers become available.

Correctness is therefore safe when jogging and free-motion planning share the
pool. The broker does not implement request priority, however. A free-motion
client should use a bounded `max_in_flight` and modest `batch_size` in
production if low-latency jogging requests can occur concurrently. When game
state guarantees the two workloads do not overlap, the planner may use the
full worker count for maximum throughput.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Iterable

import zmq

from core import bus


@dataclass(frozen=True)
class CollisionBatchResult:
    """Result for one batch sent to the collision-worker pool.

    Attributes:
        ok: True when the worker processed the request without a worker
            exception.
        free: One boolean per input configuration. True means the robot
            configuration is not in collision.
        error: Worker-side error string, or None when ok is True.
        compute_ms: Worker-reported collision computation time.
    """

    ok: bool
    free: list[bool]
    error: str | None
    compute_ms: float


class CollisionWorkerClient:
    """Small DEALER client for `req.collision_check` bundles.

    The caller owns process startup. This class owns only one DEALER request
    socket and must be used from the thread/process that created it.
    """

    def __init__(
        self,
        *,
        endpoint: str = bus.COLLISION_ROUTER_ENDPOINT,
        producer: str = "free_motion_planner",
        timeout_s: float = 30.0,
    ) -> None:
        """Create a socket connected to the collision broker.

        Args:
            endpoint: ROUTER endpoint exposed by `apps.collision_broker`.
            producer: Envelope producer name used in logs and replies.
            timeout_s: Per-request timeout. Increase when bundled edges
                contain many finely discretized states.
        """
        self.endpoint = endpoint  # ROUTER socket address to connect to.
        self.producer = producer  # Message producer id for diagnostics.
        self.timeout_s = timeout_s  # Maximum seconds to wait per batch.
        self._ctx = zmq.Context.instance()  # Shared process-wide ZMQ context.
        self._sock = self._open_socket()  # Async request socket.
        self._request_id = 0  # Monotonic id used to match replies.
        time.sleep(0.25)  # Allow ROUTER/DEALER routing to settle.

    def close(self) -> None:
        """Close the DEALER socket owned by this client."""
        self._sock.close(0)

    def check_configs(self, configs_rad: Iterable[Iterable[float]]) -> CollisionBatchResult:
        """Check a batch of robot configurations in radians.

        Args:
            configs_rad: Iterable of six-joint configurations, each in
                bus joint order and radians.

        Returns:
            CollisionBatchResult with one `free` entry per input config.

        Raises:
            TimeoutError: No matching collision reply arrived before
                `timeout_s`.
            RuntimeError: The worker replied with `ok=false`.
        """
        configs = [[float(v) for v in q] for q in configs_rad]
        return self.check_configs_parallel(
            configs,
            batch_size=max(1, len(configs)),
            max_in_flight=1,
        )

    def check_configs_parallel(
        self,
        configs_rad: Iterable[Iterable[float]],
        *,
        batch_size: int,
        max_in_flight: int,
    ) -> CollisionBatchResult:
        """Check configurations through multiple workers concurrently.

        Requests are kept within a bounded in-flight window. The collision
        broker round-robins those requests across available REP workers;
        replies may arrive out of order and are restored to input order.

        Args:
            configs_rad: Six-axis configurations in radians.
            batch_size: Configurations processed sequentially by one worker.
            max_in_flight: Maximum outstanding worker requests. Set this to
                the collision-worker count for full pool utilization.
        """
        configs = [[float(v) for v in q] for q in configs_rad]
        if not configs:
            return CollisionBatchResult(ok=True, free=[], error=None, compute_ms=0.0)
        batch_size = max(1, int(batch_size))  # Configurations per worker request.
        max_in_flight = max(1, int(max_in_flight))  # Bounded outstanding requests.
        chunks = [
            (start, configs[start:start + batch_size])
            for start in range(0, len(configs), batch_size)
        ]
        answers = [False] * len(configs)  # Collision-free flags in original order.
        pending: dict[int, tuple[int, int]] = {}  # Request id -> (start index, count).
        next_chunk = 0  # Index of the next undispatched chunk.
        compute_ms = 0.0  # Sum of worker compute times, not wall-clock latency.
        deadline = time.perf_counter() + self.timeout_s

        try:
            while next_chunk < len(chunks) or pending:
                while next_chunk < len(chunks) and len(pending) < max_in_flight:
                    start, chunk = chunks[next_chunk]
                    request_id = self._send_request(chunk)
                    pending[request_id] = (start, len(chunk))
                    next_chunk += 1

                remaining_ms = max(1, int((deadline - time.perf_counter()) * 1000))
                if time.perf_counter() >= deadline or self._sock.poll(remaining_ms) == 0:
                    raise TimeoutError(
                        f"collision replies timed out after {self.timeout_s:.1f}s; "
                        f"pending={len(pending)}"
                    )
                reply = self._recv_reply()
                request_id = reply.get("request_id")
                if not isinstance(request_id, int) or request_id not in pending:
                    continue
                start, expected_count = pending.pop(request_id)
                if not reply.get("ok"):
                    error = str(reply.get("error") or "collision worker error")
                    raise RuntimeError(error)
                results = reply.get("results") or []
                if len(results) != expected_count:
                    raise RuntimeError(
                        f"collision worker returned {len(results)} results; "
                        f"expected {expected_count}"
                    )
                answers[start:start + expected_count] = [
                    not bool(item.get("collision")) for item in results
                ]
                compute_ms += float(reply.get("compute_ms") or 0.0)
        except BaseException:
            # Outstanding replies would contaminate the next operation. A
            # fresh DEALER identity discards them safely at the ROUTER.
            self._reset_socket()
            raise

        return CollisionBatchResult(
            ok=True,
            free=answers,
            error=None,
            compute_ms=compute_ms,
        )

    def check_edge_until_collision(
        self,
        points_rad: Iterable[Iterable[float]],
        *,
        batch_size: int,
        max_in_flight: int,
    ) -> tuple[bool, int, int]:
        """Probe an edge and stop sending work after first collision.

        Returns:
            Tuple of (edge_free, configs_sent, batches_sent).
        """
        points = [[float(v) for v in q] for q in points_rad]
        if not points:
            return True, 0, 0
        batch_size = max(1, int(batch_size))
        max_in_flight = max(1, int(max_in_flight))
        chunks = [
            points[start:start + batch_size]
            for start in range(0, len(points), batch_size)
        ]
        pending: dict[int, int] = {}  # Request id -> expected result count.
        next_chunk = 0
        configs_sent = 0
        batches_sent = 0
        deadline = time.perf_counter() + self.timeout_s

        try:
            while next_chunk < len(chunks) or pending:
                while next_chunk < len(chunks) and len(pending) < max_in_flight:
                    chunk = chunks[next_chunk]
                    request_id = self._send_request(chunk)
                    pending[request_id] = len(chunk)
                    next_chunk += 1
                    configs_sent += len(chunk)
                    batches_sent += 1

                remaining_ms = max(1, int((deadline - time.perf_counter()) * 1000))
                if time.perf_counter() >= deadline or self._sock.poll(remaining_ms) == 0:
                    raise TimeoutError(
                        f"collision replies timed out after {self.timeout_s:.1f}s; "
                        f"pending={len(pending)}"
                    )
                reply = self._recv_reply()
                request_id = reply.get("request_id")
                if not isinstance(request_id, int) or request_id not in pending:
                    continue
                expected_count = pending.pop(request_id)
                if not reply.get("ok"):
                    error = str(reply.get("error") or "collision worker error")
                    raise RuntimeError(error)
                results = reply.get("results") or []
                if len(results) != expected_count:
                    raise RuntimeError(
                        f"collision worker returned {len(results)} results; "
                        f"expected {expected_count}"
                    )
                if any(bool(item.get("collision")) for item in results):
                    # Outstanding replies would contaminate the next operation.
                    # Replace the DEALER identity and fail fast.
                    if pending:
                        self._reset_socket()
                    return False, configs_sent, batches_sent
        except BaseException:
            self._reset_socket()
            raise

        return True, configs_sent, batches_sent

    def is_config_free(self, q_rad: Iterable[float]) -> bool:
        """Return True when one six-axis configuration is collision-free."""
        return self.check_configs([q_rad]).free[0]

    def _open_socket(self) -> zmq.Socket:
        """Create and connect one non-blocking-linger DEALER socket."""
        sock = self._ctx.socket(zmq.DEALER)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(self.endpoint)
        return sock

    def _reset_socket(self) -> None:
        """Discard outstanding replies by replacing the DEALER socket."""
        self._sock.close(0)
        self._sock = self._open_socket()

    def _send_request(self, configs_rad: list[list[float]]) -> int:
        """Send one collision bundle and return its unique request id."""
        self._request_id += 1
        request_id = self._request_id
        env = bus.make_envelope(self.producer, seq=request_id)
        env.update({
            "request_id": request_id,
            "configs_rad": configs_rad,
            "check_self": True,
            "check_world": True,
        })
        self._sock.send_multipart([
            b"",
            b"req.collision_check",
            json.dumps(env, separators=(",", ":")).encode("utf-8"),
        ])
        return request_id

    def _recv_reply(self) -> dict:
        """Receive and decode one collision reply multipart message."""
        frames = self._sock.recv_multipart()
        payload = frames[2] if len(frames) >= 3 and frames[0] == b"" else frames[-1]
        decoded = json.loads(payload.decode("utf-8"))
        return decoded if isinstance(decoded, dict) else {}


class WorkerCollisionOracle:
    """Collision oracle used by `BiRRTConnectPlanner`.

    The oracle batches edge-discretization checks so the planner can ask
    "is this whole local motion valid?" without knowing about ZeroMQ.
    """

    def __init__(
        self,
        client: CollisionWorkerClient,
        *,
        batch_size: int = 64,
        edge_batch_size: int | None = None,
        edge_max_in_flight: int | None = None,
        max_in_flight: int = 1,
    ) -> None:
        """Wrap a collision client.

        Args:
            client: Connected worker-pool client.
            batch_size: Number of configurations per worker request.
                Larger values improve throughput but increase latency.
            edge_batch_size: Configurations checked per ordered edge probe.
                Defaults to `batch_size` when omitted.
            edge_max_in_flight: Maximum concurrent edge probes before a
                reply is required. Defaults to `min(max_in_flight, 8)`
                to avoid over-dispatching colliding edges.
            max_in_flight: Concurrent requests issued to the broker. Set to
                the configured worker count to keep the pool occupied.
        """
        self.client = client  # Connected ZeroMQ collision client.
        self.batch_size = max(1, int(batch_size))  # Configs per request.
        edge_size = self.batch_size if edge_batch_size is None else edge_batch_size
        self.edge_batch_size = max(1, int(edge_size))  # Edge probe request size.
        self.max_in_flight = max(1, int(max_in_flight))  # Concurrent worker bundles.
        default_edge_in_flight = min(self.max_in_flight, 8)
        edge_in_flight = default_edge_in_flight if edge_max_in_flight is None else edge_max_in_flight
        self.edge_max_in_flight = max(1, int(edge_in_flight))
        self.config_checks = 0  # Total individual configs sent.
        self.batch_checks = 0  # Total worker requests sent.

    def are_configs_free(
        self,
        configs_rad: list[list[float]],
        *,
        batch_size: int | None = None,
    ) -> list[bool]:
        """Return collision-free booleans for a list of configurations.

        `batch_size=1` is useful for parallel screening of unrelated random
        endpoints; the configured larger batch remains efficient for paths.
        """
        request_batch_size = self.batch_size if batch_size is None else max(1, int(batch_size))
        result = self.client.check_configs_parallel(
            configs_rad,
            batch_size=request_batch_size,
            max_in_flight=self.max_in_flight,
        )
        self.config_checks += len(configs_rad)
        self.batch_checks += (len(configs_rad) + request_batch_size - 1) // request_batch_size
        return result.free

    def is_config_free(self, q_rad: list[float]) -> bool:
        """Return True when one configuration is collision-free."""
        return self.are_configs_free([q_rad])[0]

    def is_edge_free(self, points_rad: list[list[float]]) -> bool:
        """Return True when every discretized edge point is collision-free.

        This path is intentionally fail-fast: once any in-flight probe reports
        a collision, no additional edge probes are sent.
        """
        edge_free, configs_sent, batches_sent = self.client.check_edge_until_collision(
            points_rad,
            batch_size=self.edge_batch_size,
            max_in_flight=self.edge_max_in_flight,
        )
        self.config_checks += configs_sent
        self.batch_checks += batches_sent
        return edge_free
