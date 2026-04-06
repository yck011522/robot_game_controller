"""Test that ur_rtde is properly installed and its modules are importable.

This script verifies:
  1. The core modules can be imported
  2. Key classes and methods exist
  3. (Optional) A live connection to a UR simulator if reachable

Run:  python tests/test_ur_rtde_import.py
"""

import sys


def test_imports():
    """Verify that ur_rtde Python bindings are importable."""
    print(f"Python {sys.version}")
    print()

    # --- rtde_control ---
    print("[1/3] Importing rtde_control ...", end=" ")
    import rtde_control
    print("OK")

    print("  RTDEControlInterface exists:", hasattr(rtde_control, "RTDEControlInterface"))
    print("  Flags available:")
    for flag in ["FLAG_VERBOSE", "FLAG_UPLOAD_SCRIPT", "FLAG_USE_EXT_UR_CAP",
                 "FLAG_CUSTOM_SCRIPT", "FLAG_NO_WAIT"]:
        val = getattr(rtde_control.RTDEControlInterface, flag, None)
        if val is not None:
            print(f"    {flag} = {val}")

    # --- rtde_receive ---
    print()
    print("[2/3] Importing rtde_receive ...", end=" ")
    import rtde_receive
    print("OK")

    print("  RTDEReceiveInterface exists:", hasattr(rtde_receive, "RTDEReceiveInterface"))

    # --- rtde_io ---
    print()
    print("[3/3] Importing rtde_io ...", end=" ")
    try:
        import rtde_io
        print("OK")
        print("  RTDEIOInterface exists:", hasattr(rtde_io, "RTDEIOInterface"))
    except ImportError as e:
        print(f"SKIPPED ({e})")

    print()
    print("=" * 50)
    print("All ur_rtde imports successful!")
    print(f"ur_rtde version: {getattr(rtde_control, '__version__', 'unknown')}")
    print("=" * 50)


def test_live_connection(host="127.0.0.1"):
    """Attempt a live RTDE receive connection (optional, needs running simulator)."""
    print()
    print(f"[Optional] Attempting live connection to {host} ...")
    import rtde_receive
    try:
        rtde_r = rtde_receive.RTDEReceiveInterface(host)
        # Printing real-time data
        # actual_q = rtde_r.getActualQ()
        # timestamp = rtde_r.getTimestamp()
        # print(f"  Connected! Joint positions (rad): {[f'{q:.4f}' for q in actual_q]}, Robot timestamp: {timestamp:.3f}s")

        # Keep printing data until interrupted (Ctrl+C)
        print("  Press Ctrl+C to stop and disconnect.")
        while True:
            actual_q = rtde_r.getActualQ()
            timestamp = rtde_r.getTimestamp()
            # Print and overwrite the same line with real-time data
            print(f"\r  Joint positions (rad): {[f'{q:.4f}' for q in actual_q]}, Robot timestamp: {timestamp:.3f}s", end="")
    # Disconnect cleanly on interrupt
    except KeyboardInterrupt:
        rtde_r.disconnect()
        print("  Disconnected cleanly.")
        return True
    except Exception as e:
        print(f"  Could not connect (expected if no simulator running): {e}")
        return False


if __name__ == "__main__":
    test_imports()

    # Uncomment the line below to test a live connection to a UR simulator:
    test_live_connection("192.168.56.101")
