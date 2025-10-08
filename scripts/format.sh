#!/usr/bin/env bash
# set -e beendet das Skript sofort, wenn ein Befehl fehlschlägt.
set -e

# --- Farbdefinitionen für die Ausgabe ---
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# --- Header für die Übersichtlichkeit ---
echo -e "\n${BLUE}=======================================================================${NC}"
echo -e "${BLUE}===== Automatisches Formatieren und Beheben von Linting-Fehlern =====${NC}"
echo -e "${BLUE}=======================================================================${NC}"

# --- 1. Benötigte Tools installieren ---
echo -e "\n${YELLOW}Installiere Auto-Formatierungs-Tools (autopep8, autoflake)...${NC}"
# Wir leiten die Ausgabe nach /dev/null, um das Log sauber zu halten.

python3 -m pip install --upgrade pip
python3 -m pip install autopep8 autoflake ruff

export PATH="$HOME/.local/bin:$PATH"
# --- 2. Automatisches Beheben von Fehlern ---

# Definiere die Verzeichnisse, die formatiert werden sollen.
# Du kannst hier weitere Verzeichnisse hinzufügen, falls nötig.
TARGET_DIRS="src"

echo -e "\n${YELLOW}Entferne ungenutzte Imports und Variablen mit 'autoflake'...${NC}"
# Dieser Befehl behebt Fehler wie F401 ('os' imported but unused) und F841 (local variable ... is assigned to but never used).
# --remove-all-unused-imports: Entfernt alle ungenutzten Importe.
# --remove-unused-variables: Entfernt ungenutzte Variablen.
# --in-place: Ändert die Dateien direkt.
# --recursive: Durchsucht alle Unterverzeichnisse.
autoflake --remove-all-unused-imports --remove-unused-variables --in-place --recursive $TARGET_DIRS

echo -e "\n${YELLOW}Formatiere Code nach PEP8-Standard mit 'autopep8'...${NC}"
# Dieser Befehl behebt die meisten Stil-Fehler wie E501 (line too long), W291/W293 (whitespace errors) und E302 (expected blank lines).
# --in-place: Ändert die Dateien direkt.
# --recursive: Durchsucht alle Unterverzeichnisse.
# --aggressive --aggressive: Wendet stärkere Regeln an, um z.B. lange Zeilen besser umzubrechen.
autopep8 --in-place --recursive --aggressive --aggressive $TARGET_DIRS

echo -e "\n${GREEN}SUCCESS:${NC} Automatische Code-Formatierung abgeschlossen."
echo -e "${YELLOW}Hinweis:${NC} Nicht alle Fehler können automatisch behoben werden. Bitte überprüfe die verbleibenden Linting-Warnungen nach dem Lauf."
