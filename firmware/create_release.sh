#!/usr/bin/env bash
# Create firmware release with GitHub CLI

set -e

if [ -z "$1" ]; then
    echo "Usage: ./create_release.sh <version>"
    echo "Example: ./create_release.sh v1.0.0"
    exit 1
fi

VERSION=$1
UF2_FILE="releases/my_firmware_${VERSION}.uf2"

echo "Creating release ${VERSION}..."

# Build release version
make release VERSION=${VERSION}

# Create git tag
git tag -a ${VERSION} -m "Release ${VERSION}"
git push origin ${VERSION}

# Create GitHub release (requires GitHub CLI: gh)
if command -v gh >/dev/null 2>&1; then
    echo "Creating GitHub release..."
    gh release create ${VERSION} \
        ${UF2_FILE} \
        --title "Firmware ${VERSION}" \
        --notes "Release ${VERSION}"
    echo "✅ GitHub release created!"
else
    echo "⚠️  GitHub CLI (gh) not found."
    echo "Upload manually: https://github.com/goldjunge91/my_steel-robot_ws/releases/new"
    echo "File: ${UF2_FILE}"
fi
