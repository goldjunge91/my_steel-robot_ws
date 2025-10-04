#!/usr/bin/env bash
# Remove stale resource symlinks that can cause Errno 17 during symlink-based installs.
# This is conservative: it only removes broken symlinks or resource markers that point
# back into the workspace build/install trees in ways that commonly collide.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[CLEANUP] Removing stale resource symlinks under build/ that point to install/ ..."

find "$ROOT_DIR/build" -type l -print0 | while IFS= read -r -d $'\0' link; do
  target=$(readlink -f "$link" || true)
  if [ -z "$target" ]; then
    echo "[CLEANUP] Broken symlink: $link -> $(readlink "$link") (removing)"
    rm -f "$link"
    continue
  fi
  # If the link points into install/ and the install target also exists as a file,
  # skip removal; otherwise remove and let the build recreate it.
  if [[ "$target" == *"/install/"* ]]; then
    if [ ! -e "$target" ]; then
      echo "[CLEANUP] Stale symlink: $link -> $target (target missing) -> removing"
      rm -f "$link"
    fi
  fi
done

echo "[CLEANUP] Done."
