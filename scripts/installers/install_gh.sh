#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"
: ${REAL_USER:=${SUDO_USER:-}}

if command -v gh &>/dev/null; then
    log SUCCESS "gh bereits installiert"
    exit 0
fi

log INFO "Installiere GitHub CLI..."
mkdir -p /etc/apt/keyrings
chmod 755 /etc/apt/keyrings
GH_KEY_TMP=$(mktemp)
if ! curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg -o "$GH_KEY_TMP"; then
    rm -f "$GH_KEY_TMP"
    log_result fail "gh key download"
    exit 1
fi
write_if_changed /etc/apt/keyrings/githubcli-archive-keyring.gpg < "$GH_KEY_TMP"
chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg || true
rm -f "$GH_KEY_TMP"

ARCH=$(dpkg --print-architecture)
cat > /tmp/new_github_repo <<EOF
deb [arch=$ARCH signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main
EOF
write_if_changed /etc/apt/sources.list.d/github-cli.list < /tmp/new_github_repo
rm -f /tmp/new_github_repo

wait_for_apt || exit 1
if ! run_action "apt update (gh)" apt update -y -qq; then
    log WARN "apt update (gh) had errors, proceeding..."
fi
if ! install_and_check "gh"; then
    exit 1
fi
exit 0
