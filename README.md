# Robot Game Controller

A two-team arcade installation. Each team has six players turning haptic
dials that jog one joint of a UR10e robotic arm; balls are scooped from
a shared pool into per-team buckets and weighed for score. The system
runs 24/7 on a single Windows PC plus a Vision PC, an Audio PC, and a
handful of Raspberry Pi display nodes.

This repository is mid-migration from a single-process threaded
prototype to a multi-process ZeroMQ-based architecture. The legacy
code under `src/*.py` (top level) still runs; the new code is being
built up under `src/{core,subsystems,apps}/` one phase at a time per
[docs/MIGRATION_PLAN.md](docs/MIGRATION_PLAN.md).

## Where things live

```
src/                          # Source.
  core/        subsystems/    # New architecture (currently scaffolds; populated through P1+).
  apps/                       # New architecture.
  main.py, game_controller.py, gamemaster_ui.py, ...   # Legacy single-process code, still runnable.
config/profiles/              # YAML profiles for the new launcher (CONFIG.md).
docs/                         # All documentation.
  architecture/               # Confirmed target architecture (read in this order):
    OVERVIEW.md  SYSTEM_MAP.md  BUS.md  CONFIG.md  SUPERVISOR.md  LOGGING.md
  MIGRATION_PLAN.md           # Phased path from legacy to target.
  GAME_MECHANICS.md           # What the game is.
  HAPTIC_PROTOCOL.md          # ESP32 dial firmware wire protocol.
  NETWORK_PROTOCOL.md         # UDP payload to the RPi display nodes (unchanged from legacy).
  LED_COLUMN.md               # Light-column hardware (wiring, addressing, RS-485).
  DEPLOYMENT.md               # Windows + Conda setup on the deployment PC.
incoming_code/                # Third-party assets to be lifted into src/ by upcoming phases.
  ur10e_robot/                # URDF + meshes consumed by SimRobotIO and CollisionWorker (P2).
  rtde_core.py                # Becomes the real RobotIO impl (P3).
archive/                      # Reference-only; not on any import path. See archive/README.md.
tests/                        # Real pytest tests live here. Ad-hoc probes are in archive/.
tools/                        # Operator scripts (bus tap, view game state, ...).
NEXT_STEPS.md                 # Live planning document — read this for the current state of the migration.
```

## Reading order for a new contributor

1. This file.
2. [docs/architecture/OVERVIEW.md](docs/architecture/OVERVIEW.md) — the
   system in one paragraph plus the four ideas everything else follows
   from.
3. [docs/architecture/SYSTEM_MAP.md](docs/architecture/SYSTEM_MAP.md) —
   what processes exist and how they connect.
4. [docs/MIGRATION_PLAN.md](docs/MIGRATION_PLAN.md) — what is being
   built next.
5. [NEXT_STEPS.md](NEXT_STEPS.md) — current status, feature inventory,
   live decisions.

For domain-specific work, jump directly to the relevant doc under
[docs/](docs/).

## Setup

See [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) for the supported Python
version, Conda environment, and install steps. Short version: Windows,
Python 3.12 via Miniforge, conda env named `game`, then
`pip install -r requirements.txt`.

## Running

The new launcher is not wired up yet; that lands in P1 of
[docs/MIGRATION_PLAN.md](docs/MIGRATION_PLAN.md). Until then, the
legacy single-process app still works:

```powershell
conda activate game
python src/main.py
```

Once P1 ships, the entry point becomes:

```powershell
python -m apps.launcher --profile config/profiles/<name>.yaml
```

with individual processes also launchable by hand per
[docs/architecture/SUPERVISOR.md §3.1](docs/architecture/SUPERVISOR.md#31-launching-a-single-process-for-development).
