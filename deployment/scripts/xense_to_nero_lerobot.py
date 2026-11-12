#!/usr/bin/env python3
"""Thin entry point for deployment/tools/xense_to_nero_lerobot.py."""

import sys
from pathlib import Path

DEPLOYMENT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DEPLOYMENT_DIR))

from tools.xense_to_nero_lerobot import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
