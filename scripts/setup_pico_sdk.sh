#!/bin/bash

# Das Skript wird bei einem Fehler sofort beendet
set -e

# Der Zielpfad für das SDK wird in einer Variable definiert
export PICO_SDK_DIR="/pico/pico-sdk"
export PICO_SDK_PATH=${PICO_SDK_DIR}

echo ">>> 1. Paketlisten werden aktualisiert..."
apt-get update

echo ">>> 2. Notwendige Abhängigkeiten werden installiert..."
# --no-install-recommends sorgt für eine schlankere Installation
apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    git

echo ">>> 3. Verzeichnis /pico wird erstellt und das SDK wird geklont..."
# -p sorgt dafür, dass keine Fehler auftreten, wenn das Verzeichnis bereits existiert
mkdir -p /pico
cd /pico

# Überprüfe, ob das SDK bereits existiert
if [ -d "${PICO_SDK_DIR}" ]; then
    echo ">>> SDK bereits vorhanden in ${PICO_SDK_DIR}. Überspringe Klonen."
else
    # --depth 1 klont nur den aktuellsten Stand ohne die komplette Historie, was schneller ist
    git clone --depth 1 https://github.com/raspberrypi/pico-sdk.git ${PICO_SDK_DIR}
fi

# --- Schritt für die Persistenz ---
echo ">>> 4. Die Umgebungsvariable PICO_SDK_PATH wird persistent gemacht..."
# Fügt den export-Befehl zur .bashrc des aktuellen Benutzers hinzu.
# Dieser wird bei jeder neuen interaktiven Shell-Sitzung ausgeführt.
echo "export PICO_SDK_PATH=${PICO_SDK_DIR}" >> ${HOME}/.bashrc

echo ""
echo "✅ Setup erfolgreich abgeschlossen!"
echo "➡️ Führen Sie 'source ~/.bashrc' aus oder starten Sie eine neue Shell (z.B. mit 'bash'), um die Variable zu aktivieren."
echo "   Alternativ ist PICO_SDK_PATH bereits für diese Session gesetzt."