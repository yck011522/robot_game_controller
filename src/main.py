"""Entry point — wires GameSettings, GameController, and GameMasterUI together.

Threading model:
  - Main thread: Tkinter UI (required by Tkinter)
  - Game loop thread: GameController (50 Hz)
  - Haptic threads: reader/writer/discovery (managed by HapticSystem)
  - Robot thread: physics simulation (managed by SimulatedRobotInterface)

Usage:
    python main.py              # Normal mode (requires hardware)
    python main.py --simulate   # Simulated haptic controllers (no hardware)
"""

import sys
import os
import argparse

# Ensure src directory is on the path
sys.path.insert(0, os.path.dirname(__file__))

from game_settings import GameSettings
from game_controller import GameController
from gamemaster_ui import GameMasterUI


def main():
    parser = argparse.ArgumentParser(description="Robot Game Controller")
    parser.add_argument(
        "--sim-haptics",
        action="store_true",
        help="Use simulated haptic dial controllers (no ESP32 hardware required)",
    )
    parser.add_argument(
        "--sim-robot",
        action="store_true",
        help="Use simulated robot instead of connecting via RTDE",
    )
    parser.add_argument(
        "--sim-weights",
        action="store_true",
        help="Use simulated weight sensors instead of real load cells",
    )
    parser.add_argument(
        "--sim-all",
        action="store_true",
        help="Enable all simulations (haptics, robot, weight sensors)",
    )
    parser.add_argument(
        "--robot-ip",
        metavar="IP",
        help="IP address of the UR robot (default: 192.168.56.101)",
    )
    args = parser.parse_args()

    settings = GameSettings()
    if args.sim_all or args.sim_haptics:
        settings.set("simulate_haptics", True)
    if args.sim_all or args.sim_robot:
        settings.set("simulate_robot", True)
    if args.sim_all or args.sim_weights:
        settings.set("simulate_weight_sensors", True)
    if args.robot_ip:
        settings.set("simulate_robot", False)
        settings.set("robot_ip", args.robot_ip)

    controller = GameController(settings)
    ui = GameMasterUI(settings)

    # Start the game controller (runs on its own thread)
    controller.start()

    try:
        # Run Tkinter on the main thread (blocking)
        ui.run()
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()
