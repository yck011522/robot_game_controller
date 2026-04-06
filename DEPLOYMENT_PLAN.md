# Deployment Machine Setup Plan

**Platform: Windows 11 (64-bit)**
Dedicated PC for running the robot game controller in production.

---

## 1. Python Version Recommendation

**Python 3.12.x** (latest patch, e.g. 3.12.8 or newer 3.12.z)

Rationale:
- Matches the current development environment (Python 3.12.8)
- `ur_rtde` 1.6.3 has prebuilt Windows x64 wheels for CPython 3.12 ✅
- `pyserial` supports Python 3.5+ — no issue
- Python 3.12 is a mature, stable release with long-term security support (EOL: October 2028)
- Avoid 3.6–3.9 (EOL or nearing EOL)

> **Do not use Python 3.13+** until `ur_rtde` explicitly publishes 3.13 wheels and we verify them.

---

## 2. Environment Management (Conda)

**Use Miniforge/Conda** with a dedicated `robot_game` environment.

This has been tested on the development machine and confirmed working:
- `robot_game` environment with Python 3.12.13
- `ur_rtde 1.6.3` prebuilt wheel installs and imports correctly
- `pyserial 3.5` installs correctly
- All three ur_rtde modules verified: `rtde_control`, `rtde_receive`, `rtde_io`

The environment can be exported from the development machine and replicated
exactly on the deployment machine (see Section 4).

---

## 3. Required Dependencies

Current (`requirements.txt`):
```
pyserial>=3.5
```

To add for robot arm integration:
```
ur_rtde>=1.6.0
```

Prebuilt Windows wheel verified: `ur_rtde-1.6.3-cp312-cp312-win_amd64.whl` (2.6 MB).
The wheel bundles the compiled C++ library with Boost already statically linked.
**No manual Boost or Visual Studio install needed** — those are only required if building from source.

All other imports in the project are Python standard library (`tkinter`, `threading`, `socket`, `json`, etc.) or local modules.

### Updated `requirements.txt` (when ready):
```
pyserial>=3.5
ur_rtde>=1.6.0
```

### Library Choice: `ur_rtde` (SDU Robotics) vs Official UR RTDE Client

Two RTDE libraries exist. We chose **`ur_rtde`** (SDU Robotics):

| | `ur_rtde` (SDU Robotics) — **CHOSEN** | `RTDE_Python_Client_Library` (Universal Robots) |
|---|---|---|
| **Install** | `pip install ur_rtde` (prebuilt wheels) | `pip install git+...` from GitHub |
| **Level** | High-level: `servoJ()`, `moveJ()`, `getActualQ()` | Low-level: raw RTDE register read/write |
| **Control loop** | Built-in `initPeriod()` / `waitPeriod()` timing | You build everything yourself |
| **ServoJ support** | Direct `servoJ(q, vel, acc, dt, lookahead, gain)` call | Must upload URScript, manage registers manually |
| **Python** | C++ with pybind11 bindings (3.6–3.12 wheels) | Pure Python (2.7+) |
| **Version** | 1.6.3 on PyPI | 2.7.12 on GitHub |

**Why `ur_rtde`:** Our architecture sends filtered joint targets via `servoJ` at
high frequency and reads back `actual_q` for haptic feedback. `ur_rtde` provides
exactly this as ready-to-use API calls. The official UR library only handles raw
RTDE data transport — we would need to write our own URScript command dispatch,
control script upload, and timing synchronization on top of it.

---

## 4. Installation Steps (Deployment Machine)

### 4.1 Install Miniforge
1. Download **Miniforge** (64-bit) from https://conda-forge.org/miniforge/
2. Run installer:
   - ✅ Install for **All Users** (or current user)
   - ✅ Add to PATH (or use the Miniforge Prompt)
3. Verify:
   ```powershell
   conda --version
   ```

### 4.2 Replicate the Environment from Development Machine

**On the development machine** (this computer), export the environment:
```powershell
conda activate robot_game
conda env export -n robot_game > environment.yml
```

Copy `environment.yml` to the deployment machine (via USB, network, or include in repo).

**On the deployment machine**, create the environment from the export:
```powershell
conda env create -f environment.yml
conda activate robot_game
```

This reproduces the exact same Python version and package versions.

> **Alternative — create from scratch** (if environment.yml is not available):
> ```powershell
> conda create -n robot_game python=3.12 -y
> conda activate robot_game
> pip install pyserial ur_rtde
> ```

### 4.3 Clone / Copy the Project
```powershell
git clone <repo-url> C:\robot_game_controller
```
Or copy the project folder to a fixed path like `C:\robot_game_controller`.

### 4.4 Verify ur_rtde Installation
```powershell
conda activate robot_game
python -c "import rtde_control; import rtde_receive; print('ur_rtde OK')"
```

Or run the test script:
```powershell
python tests/test_ur_rtde_import.py
```

### 4.5 Verify Serial Access
Plug in the haptic controller USB boards and verify:
```powershell
python -c "import serial.tools.list_ports; [print(p) for p in serial.tools.list_ports.comports()]"
```

---

## 5. Auto-Start on Boot (Optional, Later)

For production deployment, set up the game controller to launch automatically:

1. Create a batch file `C:\robot_game_controller\start_game.bat`:
   ```bat
   @echo off
   call C:\Users\<USER>\miniforge3\Scripts\activate.bat robot_game
   cd /d C:\robot_game_controller
   python src\main.py
   ```
2. Add a shortcut to `shell:startup` or create a Windows Task Scheduler entry

---

## 6. Summary Checklist

- [ ] Windows 11 machine provisioned
- [ ] Miniforge installed
- [ ] `robot_game` conda environment created from `environment.yml`
- [ ] `ur_rtde` and `pyserial` verified (`tests/test_ur_rtde_import.py` passes)
- [ ] Project files deployed to fixed path
- [ ] USB serial access to haptic boards verified
- [ ] Auto-start batch file configured (optional)
