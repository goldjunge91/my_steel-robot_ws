#!/usr/bin/env bash
# Minimal script: delete ALL GitHub Actions caches for the current repository using gh CLI
# Usage:
#   scripts/delete_caches.sh [--all] [--yes]
# Example:
#   scripts/delete_caches.sh --all --yes

set -euo pipefail

# Defaults
FORCE="false"
ALL="false"

print_usage() {
  cat <<EOF
Usage: $0 [--all] [--yes]

Deletes ALL GitHub Actions caches for the current repository using the gh CLI.
This simplified script only supports deleting all caches. It ignores date filters.

Options:
  --all                 Delete ALL caches (default behavior)
  --yes                 Do not prompt for confirmation (force)
  -h, --help            Show this help

Note: Requires 'gh' (GitHub CLI) and 'jq'.
EOF
}

# parse args (only --all / --yes supported)
while [[ $# -gt 0 ]]; do
  case "$1" in
    --all)
      ALL="true"; shift 1;;
    --yes)
      FORCE="true"; shift 1;;
    -h|--help)
      print_usage; exit 0;;
    *)
      echo "Unknown arg: $1"; print_usage; exit 2;;
  esac
done

if ! command -v gh >/dev/null 2>&1; then
  echo "Error: gh (GitHub CLI) is required. Install from https://cli.github.com/"
  exit 1
fi
if ! command -v jq >/dev/null 2>&1; then
  echo "Error: jq is required. Install it with your package manager."
  exit 1
fi

# determine repo (owner/repo)
REPO_FULL=$(gh repo view --json nameWithOwner --jq .nameWithOwner) || {
  echo "Failed to detect repository. Run 'gh auth login' and ensure you are in a git repo."; exit 1; }

echo "Repository: $REPO_FULL"
echo "Deleting ALL caches (no date filtering)"

if [[ "$FORCE" != "true" ]]; then
  read -r -p "Proceed to delete ALL caches? [y/N] " yn
  case "$yn" in
    [Yy]*) ;;
    *) echo "Aborted by user"; exit 0;;
  esac
fi

page=1
deleted=0

while :; do
  echo "Fetching caches (page $page) ..."
  resp=$(gh api -H "Accept: application/vnd.github+json" "/repos/$REPO_FULL/actions/caches?per_page=100&page=$page") || true
  # if response is empty or not JSON break
  if [[ -z "$resp" ]]; then
    break
  fi

  count=$(echo "$resp" | jq '.actions_caches | length')
  if [[ "$count" -eq 0 ]]; then
    break
  fi

  echo "Found $count caches on page $page"

  echo "$resp" | jq -r '.actions_caches[] | [.id, .key] | @tsv' | \
  while IFS=$'\t' read -r id key; do
    echo "Deleting cache id=$id key=\"$key\""
    if gh api -X DELETE -H "Accept: application/vnd.github+json" "/repos/$REPO_FULL/actions/caches/$id"; then
      echo "Deleted cache $id"
      deleted=$((deleted+1))
    else
      echo "Failed to delete cache $id" >&2
    fi
  done

  page=$((page+1))
done

echo "Done. Deleted: $deleted"

exit 0
