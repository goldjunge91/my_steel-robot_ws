#!/usr/bin/env bash
set -euo pipefail

REPO_URL="https://github.com/ohmybash/oh-my-bash.git"
OH_DIR="$HOME/.oh-my-bash"
EXAMPLE_DEST="$HOME/.bashrc_example"

echo "==> Installing oh-my-bash to: $OH_DIR"

if command -v git >/dev/null 2>&1; then
  if [ -d "$OH_DIR" ]; then
    echo "Repository already exists, updating..."
    git -C "$OH_DIR" pull --ff-only || true
  else
    echo "Cloning $REPO_URL..."
    git clone --depth=1 "$REPO_URL" "$OH_DIR"
  fi
else
  echo "ERROR: 'git' is required but not installed. Install git and re-run this script." >&2
  exit 2
fi

# Rename existing ~/.bashrc to ~/.bashrc_bak (if ~/.bashrc_bak exists, keep it and create a timestamped fallback)
if [ -f "$HOME/.bashrc" ]; then
  if [ -e "$HOME/.bashrc_bak" ]; then
    bak_back="$HOME/.bashrc_bak.$(date +%s)"
    echo "Existing .bashrc_bak found; moving it to: $bak_back"
    mv -f "$HOME/.bashrc_bak" "$bak_back"
  fi
  echo "Renaming existing ~/.bashrc -> ~/.bashrc_bak"
  mv -f "$HOME/.bashrc" "$HOME/.bashrc_bak"
fi

# Try to locate an example bashrc inside the cloned repo
echo "Searching for an example bashrc inside $OH_DIR..."
example=$(find "$OH_DIR" -maxdepth 4 -type f \( -iname "bashrc*" -o -iname "*bashrc*" \) 2>/dev/null | head -n1 || true)

if [ -n "$example" ] && [ -f "$example" ]; then
  echo "Found example: $example"
  cp -a "$example" "$EXAMPLE_DEST"
  echo "Copied example to: $EXAMPLE_DEST"
else
  echo "No example found in repo; creating minimal $EXAMPLE_DEST"
  cat > "$EXAMPLE_DEST" <<'EOF'
# Minimal .bashrc_example to source oh-my-bash
export OSH="$HOME/.oh-my-bash"
source "$OSH/oh-my-bash.sh"

# You can add your customizations below
EOF
  echo "Created $EXAMPLE_DEST"
fi

cat <<EOF

Installation finished.
- oh-my-bash installed at: $OH_DIR
- Example bashrc copied to: $EXAMPLE_DEST

To activate immediately, either:
  1) Copy the example to your active .bashrc:
     cp -i "$EXAMPLE_DEST" "$HOME/.bashrc" && source "$HOME/.bashrc"
  2) Or manually source the oh-my-bash script now:
     source "$OH_DIR/oh-my-bash.sh"

EOF
