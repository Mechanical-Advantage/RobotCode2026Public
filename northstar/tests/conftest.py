# Ensure the northstar root directory is first in sys.path so that northstar's
# local modules (config/, pipeline/, etc.) take priority over any installed
# packages with the same names (e.g. hub_counter's "config" sub-package that
# gets installed when running the full test suite in a shared virtual env).
import sys
from pathlib import Path

_northstar_root = str(Path(__file__).parent.parent)
if _northstar_root not in sys.path:
    sys.path.insert(0, _northstar_root)
