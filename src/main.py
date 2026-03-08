"""Entry point — wires GameSettings, GameController, and GameMasterUI together.

Threading model:
  - Main thread: Tkinter UI (required by Tkinter)
  - Game loop thread: GameController (50 Hz)
  - Haptic threads: reader/writer/discovery (managed by HapticSystem)
  - Robot thread: physics simulation (managed by SimulatedRobotInterface)

Usage:
    python main.py
"""

import sys
import os

# Ensure src directory is on the path
sys.path.insert(0, os.path.dirname(__file__))

from game_settings import GameSettings
from game_controller import GameController
from gamemaster_ui import GameMasterUI


def main():
    settings = GameSettings()
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
