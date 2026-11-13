#!/usr/bin/env python3
"""Run this folder's hardware-free tests using only the Python standard library."""

from pathlib import Path
import unittest


def main():
    directory = Path(__file__).resolve().parent
    suite = unittest.defaultTestLoader.discover(str(directory), pattern="test_*.py")
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
