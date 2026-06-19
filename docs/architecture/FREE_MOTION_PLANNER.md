# Free-Motion Planner Architecture

## Current status

- Free-motion planning is **experimental** and **not production-ready**.
- Implementation is partially tested in standalone validation only.
- Keep `src/subsystems/motion_planning/` and `tools/validate_free_motion_planner.py`
  for continued development, but do not wire this path into production reset flow yet.

## What is validated

- Layer split is working:
  - search core in `planner_core.py` + `birrt_connect.py`
  - collision transport/oracle in `collision_client.py`
  - standalone validation harness in `tools/validate_free_motion_planner.py`
- Collision workers do parallelize in practice (benchmark scaling from about
  543 checks/s with 1 worker to about 4847 checks/s with 18 workers at
  bundle size 2).
- Baseline hard-set behavior is stable: cases 1,2,3,5,7,10 pass; cases 4,6,8,9
  fail by `total_timeout` under 30 s total timeout.

## Parameter sweeps already tested

- `extend-step-deg`: 5, 10, 20, 30, 45, 60, 90, 120, 180 (case 4 only)
  - No success within 30 s for any value.
  - Smaller values increase explored nodes/connect attempts but still time out.
- `goal-sample-rate`: 0.05, 0.12, 0.2, 0.35, 0.5, 0.7, 0.9 (case 4 only)
  - No success within 30 s.
- `max-connect-steps`: 64 vs 200
  - Did not clear failing hard cases (4,6,8,9) at 30 s budget.
- `max-in-flight`: 1, 2, 4, 8, 18, 32 (case 4 only)
  - Best progress observed around 8.
  - Too low under-utilizes workers; too high over-dispatches colliding probes.
- Fail-fast edge probing was added in `collision_client.py`.
  - Reduced collision checks substantially (example hard-set baseline:
    463631 -> 206294 checks) without changing pass rate (still 6/10).

## Known missing pieces

- Hard cases 4, 6, 8, 9 still fail under current 30 s budget.
- No narrow-passage-specific sampling heuristic yet (bridge/obstacle-biased
  or medial-axis style sampling).
- No production integration contract yet (request/reply API and lifecycle in
  a dedicated planner app process still pending).

## Reproduce current failing cases

Baseline hard-set run (shows cases 4,6,8,9 failing):

```powershell
C:/Users/yck01/miniconda3/envs/game/python.exe tools/validate_free_motion_planner.py --case-set hard --max-cases 10 --iterations-per-attempt 100 --attempt-timeout-s 10 --max-restarts 100 --total-timeout-s 30 --batch-size 2 --step-deg 5.0 --extend-step-deg 180 --max-connect-steps 200
```

Single-case repros (custom datasets):

```powershell
C:/Users/yck01/miniconda3/envs/game/python.exe tools/validate_free_motion_planner.py --case-set custom --dataset tools/free_motion_cases/hard_case_04.json --max-cases 1 --iterations-per-attempt 100 --attempt-timeout-s 10 --max-restarts 100 --total-timeout-s 30 --batch-size 2 --step-deg 5.0 --extend-step-deg 180 --max-connect-steps 200
C:/Users/yck01/miniconda3/envs/game/python.exe tools/validate_free_motion_planner.py --case-set custom --dataset tools/free_motion_cases/hard_case_06.json --max-cases 1 --iterations-per-attempt 100 --attempt-timeout-s 10 --max-restarts 100 --total-timeout-s 30 --batch-size 2 --step-deg 5.0 --extend-step-deg 180 --max-connect-steps 200
C:/Users/yck01/miniconda3/envs/game/python.exe tools/validate_free_motion_planner.py --case-set custom --dataset tools/free_motion_cases/hard_case_08.json --max-cases 1 --iterations-per-attempt 100 --attempt-timeout-s 10 --max-restarts 100 --total-timeout-s 30 --batch-size 2 --step-deg 5.0 --extend-step-deg 180 --max-connect-steps 200
C:/Users/yck01/miniconda3/envs/game/python.exe tools/validate_free_motion_planner.py --case-set custom --dataset tools/free_motion_cases/hard_case_09.json --max-cases 1 --iterations-per-attempt 100 --attempt-timeout-s 10 --max-restarts 100 --total-timeout-s 30 --batch-size 2 --step-deg 5.0 --extend-step-deg 180 --max-connect-steps 200
```

Notes:

- Validation outputs now default to `tools/runs/free_motion_planner/`.
- Collision benchmark outputs now default to `tools/runs/collision_benchmark/`.
