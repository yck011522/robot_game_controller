"""Global COM port lock registry.

Prevents multiple discovery threads (haptic, LED, etc.) from simultaneously
opening the same serial port for probing.  Each subsystem calls
``acquire_port(port)`` before opening a port and ``release_port(port)``
when done.  ``get_claimed_ports()`` returns ports currently held by any
subsystem so others can skip them.

Thread-safe by design — all public functions use an internal lock.
"""

import threading
from typing import Set

_lock = threading.Lock()
_held_ports: dict[str, str] = {}   # port → owner label


def acquire_port(port: str, owner: str = "") -> bool:
    """Try to claim *port* for exclusive probe/connect use.

    Returns True if the port was free and is now claimed.
    Returns False if another owner already holds it.
    """
    with _lock:
        if port in _held_ports:
            return False
        _held_ports[port] = owner
        return True


def release_port(port: str) -> None:
    """Release a previously acquired port."""
    with _lock:
        _held_ports.pop(port, None)


def get_claimed_ports() -> Set[str]:
    """Return the set of ports currently held by any subsystem."""
    with _lock:
        return set(_held_ports.keys())


def is_port_claimed(port: str) -> bool:
    """Check whether *port* is currently claimed."""
    with _lock:
        return port in _held_ports
