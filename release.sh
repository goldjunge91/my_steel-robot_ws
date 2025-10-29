#!/bin/bash
# Release-Skript für my_steel Robot
# Erstellt einen Git-Tag und löst alle Docker-Build-Workflows aus

set -e

# Farben für Output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  my_steel Robot Release Tool${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Prüfe ob Version als Parameter übergeben wurde
if [ -z "$1" ]; then
    echo -e "${YELLOW}Usage: $0 <version>${NC}"
    echo -e "${YELLOW}Example: $0 1.0.0${NC}"
    echo ""
    echo "Dieser Befehl wird:"
    echo "  1. Git-Tag v<version> erstellen"
    echo "  2. Tag zu GitHub pushen"
    echo "  3. Alle Docker-Build-Workflows auslösen"
    echo ""
    exit 1
fi

VERSION=$1
TAG="v${VERSION}"

# Validiere Versions-Format (semantic versioning)
if ! [[ $VERSION =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo -e "${RED}❌ ERROR: Version muss im Format X.Y.Z sein (z.B. 1.0.0)${NC}"
    exit 1
fi

# Prüfe ob Tag bereits existiert
if git rev-parse "$TAG" >/dev/null 2>&1; then
    echo -e "${RED}❌ ERROR: Tag $TAG existiert bereits!${NC}"
    echo ""
    echo "Vorhandene Tags:"
    git tag -l "v*" | tail -5
    echo ""
    echo "Zum Löschen des Tags:"
    echo "  git tag -d $TAG"
    echo "  git push origin :refs/tags/$TAG"
    exit 1
fi

# Zeige aktuelle Tags
echo -e "${BLUE}Vorhandene Release-Tags:${NC}"
if git tag -l "v*" | grep -q .; then
    git tag -l "v*" | tail -5
else
    echo "  (keine Tags vorhanden)"
fi
echo ""

# Bestätigung
echo -e "${YELLOW}Neuer Release: ${GREEN}${TAG}${NC}"
echo ""
echo "Dies wird folgende Docker-Images bauen:"
echo "  • humble-${VERSION}-robot-pi-YYYYMMDD"
echo "  • humble-${VERSION}-robot-pi-v2-YYYYMMDD"
echo "  • humble-${VERSION}-hardware-YYYYMMDD"
echo "  • humble-${VERSION}-simulation-YYYYMMDD"
echo ""
echo "Auf folgenden Registries:"
echo "  • Docker Hub: goldjunge491/my-steel-robot"
echo "  • GHCR (Workspace): ghcr.io/goldjunge91/my_steel-robot_ws"
echo "  • GHCR (Unified): ghcr.io/goldjunge91/my-steel-robot"
echo ""
read -p "Fortfahren? (y/N) " -n 1 -r
echo ""

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${YELLOW}Abgebrochen.${NC}"
    exit 0
fi

# Erstelle Tag
echo ""
echo -e "${BLUE}📝 Erstelle Git-Tag ${TAG}...${NC}"
git tag -a "$TAG" -m "Release ${VERSION}"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Tag erstellt${NC}"
else
    echo -e "${RED}❌ Fehler beim Erstellen des Tags${NC}"
    exit 1
fi

# Pushe Tag
echo ""
echo -e "${BLUE}🚀 Pushe Tag zu GitHub...${NC}"
git push origin "$TAG"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Tag gepusht${NC}"
else
    echo -e "${RED}❌ Fehler beim Pushen des Tags${NC}"
    echo ""
    echo "Tag lokal löschen:"
    echo "  git tag -d $TAG"
    exit 1
fi

# Erfolg
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  ✅ Release ${TAG} erfolgreich!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Die folgenden Workflows wurden ausgelöst:"
echo "  • Build Robot Pi Docker Image (Docker Hub)"
echo "  • Build Robot Pi Docker Image (GHCR)"
echo "  • Build/Publish ROS Docker Image"
echo ""
echo "Status prüfen:"
echo "  https://github.com/goldjunge91/my_steel-robot_ws/actions"
echo ""
echo "Images werden verfügbar sein als:"
echo "  • humble-${VERSION}-robot-pi"
echo "  • humble-${VERSION}-robot-pi-v2"
echo "  • humble-${VERSION}-hardware"
echo "  • humble-${VERSION}-simulation"
echo ""
echo -e "${YELLOW}⏱️  Build-Zeit: ca. 30-45 Minuten pro Image${NC}"
echo ""