"""Arena light-column subsystem.

Modules:
    layout     - load strip groupings/routing from the device config.
    frames     - pure color/frame composition helpers.
    transport  - low-level (near-stateless) RS485 serial transport.
    controller - high-level, stateful per-stage animation + send scheduler.
"""
