"""Bus broker — XSUB/XPUB proxy + 1 Hz heartbeat.

Per [BUS.md §1](../../../docs/architecture/BUS.md#1-topology--one-shared-bus-via-xpubxsub-proxy):
binds the two well-known endpoints, runs `zmq.proxy(xsub, xpub)` in a
daemon thread, and publishes `heartbeat.bus_broker` at 1 Hz on the main
thread.
"""
