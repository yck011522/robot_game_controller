"""Bus broker — XSUB/XPUB proxy + 1 Hz heartbeat.

How the broker works
--------------------
ZeroMQ's PUB/SUB pattern requires every subscriber to know every
publisher's address, which would make adding a new process a config
change everywhere. The standard fix is a tiny **forwarding broker** that
exposes two well-known endpoints and relays every message between them:

    publishers ──PUB──> tcp://127.0.0.1:5550 (XSUB)
                                              │  zmq.proxy(xsub, xpub)
    subscribers <─SUB── tcp://127.0.0.1:5551 (XPUB)

XSUB is "the SUB side, but exposed as a server bind" — publishers
`connect()` their PUB sockets to it. XPUB is the same idea for the
other direction: subscribers connect their SUB sockets there. The
broker just calls `zmq.proxy(xsub, xpub)`, which is a blocking C-level
loop that forwards every frame from XSUB to XPUB and forwards
subscription-update frames from XPUB back to XSUB so upstream
publishers know which topics actually have listeners. We run that
inside a daemon thread so the main thread is free for our own
heartbeat publishing.

Why we still need a heartbeat on the broker
-------------------------------------------
The proxy itself produces no traffic. To prove the broker is alive
(and that the bus is actually carrying messages), the broker also acts
as a regular publisher: it owns its own PUB socket, `connect()`s it
back to its own XSUB endpoint, and emits `heartbeat.bus_broker` at
1 Hz with the full BUS.md §6.9 schema. The supervisor and every test
subscribes to `heartbeat.*` and uses these as the liveness signal.

Shutdown
--------
`zmq.proxy()` is a blocking C call; the only way out is to close the
sockets it's holding. We do that with `ctx.destroy(linger=0)` in the
finally block, which causes `proxy()` to raise `ContextTerminated` in
the daemon thread and lets the process exit cleanly.

This module is invoked by the supervisor via the
[SUPERVISOR.md §3](../../../docs/architecture/SUPERVISOR.md#3-spawn-contract)
spawn contract:

    python -m apps.bus_broker --profile <yaml> --proc bus_broker

It can also be launched by hand for development — that is the entire
point of the CLI-only contract.
"""
