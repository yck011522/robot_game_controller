# archive/

Reference-only material. **Nothing in this folder is on any runtime
import path** and nothing here is exercised by the test suite. Files
live here either because:

- they are legacy single-process code being replaced by the new
  architecture (see [docs/MIGRATION_PLAN.md](../docs/MIGRATION_PLAN.md)), or
- they are exploration scripts whose useful parts are being lifted
  into the new tree on the phase that needs them.

Do not import from `archive/` in any code under `src/`. When a piece
of logic here gets ported into the new tree, the destination should
not depend on the archived original — copy what is needed and leave
the archive untouched as a historical reference.

## Current contents

| Item | Why archived | Where it goes |
|------|--------------|---------------|
| `bullet_collision_keyboard_explorer.py` | pybullet sandbox with keyboard jog + collision worker pool. | Logic extracted into `src/subsystems/collision_worker/` (P2) and the keyboard-input UI (`src/apps/keyboard_input_ui/`, P2). |
| `bullet_collision_keyboard_explorer_design.md` | Design spec that accompanies the above script. | Reference reading for P2 implementers; not promoted into `docs/`. |
| `test_led_animation_rate.py` | Ad-hoc rate test for the legacy LED stack. | Superseded by P10 acceptance criteria in `docs/MIGRATION_PLAN.md`. |
| `test_led_comm.py` | Ad-hoc LED RS-485 probe. | Same. |
| `test_probe_94.py` | One-shot probe for the LED controller's `0x94` address-query command. | Same. |

## Why not deleted

The exploration scripts contain hard-won timing constants, pybullet
setup details, and RS-485 quirks that are easier to read in their
original form than to reconstruct from comments. Keeping them in
`archive/` makes them grep-able without polluting the production tree.
