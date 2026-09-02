"""Make the package importable whether pytest runs here or from a parent repo."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
