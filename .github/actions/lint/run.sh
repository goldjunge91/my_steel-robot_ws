#!/bin/bash
set -euo pipefail

# Run repository scripts from the mounted workspace explicitly. act and GH Actions
# mount the repo at /github/workspace inside the container, and on Windows-hosted
# Docker the executable bit may not be preserved. Invoking via bash avoids
# "Permission denied" errors.
SCRIPT_DIR="/github/workspace"

/bin/bash "$SCRIPT_DIR/setup.sh"
exec /bin/bash -lc "ament_${LINTER} src/"
