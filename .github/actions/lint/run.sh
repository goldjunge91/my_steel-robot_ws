#!/usr/bin/env bash
set -euo pipefail

repo_root="$(git rev-parse --show-toplevel)"

# Prefer HTTPS in CI when a token is available
if [ -n "${GITHUB_TOKEN:-}" ]; then
  git config --global url."https://x-access-token:${GITHUB_TOKEN}@github.com/".insteadOf git@github.com:
  git config --global url."https://x-access-token:${GITHUB_TOKEN}@github.com/".insteadOf ssh://git@github.com/
  git config --global url."https://x-access-token:${GITHUB_TOKEN}@github.com/".insteadOf https://github.com/
fi

# Ensure GitHub's host key is present should SSH be used somewhere
mkdir -p ~/.ssh
chmod 700 ~/.ssh
ssh-keyscan -H github.com >> ~/.ssh/known_hosts 2>/dev/null || true
chmod 600 ~/.ssh/known_hosts

./setup.sh
ament_ src/
