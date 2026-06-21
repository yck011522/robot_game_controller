"""Per-bucket LED scoreboard package.

A pure consumer of the game controller's ``state.full`` snapshot that drives the
six per-bucket LED text panels over a single RS485/USB-serial port. Mirrors the
``light_column`` subsystem split:

* :mod:`subsystems.scoreboard.layout` - device-file wiring (port + bucket->panel).
* :mod:`subsystems.scoreboard.transport` - serial port + text-command builders.
* :mod:`subsystems.scoreboard.controller` - stage->panel-content brain + sender.
"""
