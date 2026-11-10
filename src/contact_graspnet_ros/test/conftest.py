import sys
from pathlib import Path

# Make ``contact_graspnet_ros`` importable when running pytest from the repo
# root without a colcon install.
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
