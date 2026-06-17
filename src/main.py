"""Compatibility entry point for the retired single-process runtime.

The maintained launch path is now the process supervisor in `apps.launcher`.
This stub exists so old `python src/main.py` commands fail with a useful
message instead of importing archived modules.
"""

from __future__ import annotations

import sys


def main() -> int:
    """Print the supported launcher command and exit with a usage error."""

    print(
        "The root single-process runtime has been archived. "
        "Use `python -m apps.launcher --profile <profile>` from the repo root."
    )
    return 2


if __name__ == "__main__":
    sys.exit(main())
