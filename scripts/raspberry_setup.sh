#!/usr/bin/env bash

set -u -o pipefail -E
# set -x  # Debug-Modus (optional)
# -u: Fehler bei undefinierten Variablen
# -o pipefail: Fehler in Pipelines beachten
# -x: Debug-Modus (zeigt jeden Befehl)
# -E: Trap-Funktionen erben Fehler
# Ensure script can signal/kill its whole process group
# (so Ctrl+C in the terminal oder `kill -TERM -PGID` auch apt/sudo trifft)
set -m || true

# Der Zielpfad für das SDK wird in einer Variable definiert
# Destination for Pico SDK (fixed location in home directory)
SDK_DEST="$HOME/pico-sdk"
export PICO_SDK_PATH="$SDK_DEST"

PICO_SDK_VERSION="2.0.0"
PICOTOOL_VERSION="2.0.0"
FAILURES=()
SKIP_PICOTOOL=0
PICOTOOL_INSTALLED=0
SDK_INSTALLED=0
TMP_DIR=""

# Farbdefinitionen
RED='\033[31m'
GREEN='\033[32m'
BLUE='\033[34m'
RESET='\033[0m'
log() {
	local level="$1"
	shift
	local color="${RESET}"
	[ "$level" = "INFO" ] && color="${BLUE}"
	[ "$level" = "WARN" ] && color="${RED}"
	echo -e "${color}[$level]${RESET} $*"
	echo "$(date): $level: $*" >>setup.log 2>/dev/null || true
}
# Simple logging / helper functions
log() {
	echo -e "${BLUE}[INFO]${RESET} $*"
	echo "$(date): INFO: $*" >>setup.log 2>/dev/null || true
}
log_warn() {
	echo -e "${RED}[WARN]${RESET} $*"
	echo "$(date): WARN: $*" >>errors.log 2>/dev/null || true
}
record_failure() {
	FAILURES+=("$*")
	echo "$(date): FAILURE: $*" >>errors.log 2>/dev/null || true
}
run_command() {
	local desc="$1"
	shift
	echo -e "${BLUE}[CMD]${RESET} $desc"
	if "$@" 2>&1 | tee -a command.log; then
		echo -e "${GREEN}✓ $desc erfolgreich${RESET}"
		echo "$(date): $desc succeeded" >>setup.log 2>/dev/null || true
		return 0
	else
		echo -e "${RED}✗ $desc fehlgeschlagen${RESET}"
		echo "$(date): $desc failed" >>errors.log 2>/dev/null || true
		return 1
	fi
}
# Cleanup funktion zum Abbrechen bei ctrl+c
cleanup() {
	echo -e "\n${RED}Abbruch durch Signal. Stoppe Kindprozesse...${RESET}"
	pgid="$(ps -o pgid= $$ | tr -d '[:space:]')"
	if [ -n "$pgid" ]; then
		sudo kill -TERM -"${pgid}" 2>/dev/null || true
	fi
	pkill -P $$ 2>/dev/null || true
	exit 1
}
trap cleanup INT TERM
# # Beispiel-Ausgaben
# echo -e "${BLUE}[STEP]${RESET} Beispiel Ausgabe"
# echo -e "${BLUE}[STEP] Beispiel Ausgabe${RESET}". <-- hganze zeile farblich
# echo -e "${GREEN}✓ Erfolgreich: Beispiel Ausgabe${RESET}"
# echo -e "${RED}✗ Fehler: Beispiel Ausgabe fehlgeschlagen${RESET}"

# Funktion für Installation und Prüfung
# Warte bis apt frei ist (verhindert Lock-Konflikte)
wait_for_apt() {
	while sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1 ||
		sudo fuser /var/lib/apt/lists/lock >/dev/null 2>&1; do
		echo -e "${BLUE}Warte auf apt-Lock...${RESET}"
		sleep 2
	done
}

# Install funktion benötigt paketname als Argument
install_and_check() {
	local packages=("$@")
	echo -e "${BLUE}[STEP] ${packages[*]} installieren${RESET}"

	# Laufzeit-Cache initialisieren (verhindert doppelte Prüfungen)
	if [ -z "${INSTALL_CACHE+x}" ]; then
		INSTALL_CACHE=""
	fi

	local missing=()
	local pkg
	for pkg in "${packages[@]}"; do
		# Wenn bereits im Cache, überspringen
		if [[ " $INSTALL_CACHE " == *" $pkg "* ]]; then
			echo -e "${GREEN}✓ $pkg bereits geprüft (Cache)${RESET}"
			continue
		fi

		# Robustere Prüfung, ob Paket bereits installiert ist
		if dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
			echo -e "${GREEN}✓ $pkg bereits installiert${RESET}"
			echo "$(date): $pkg bereits installiert" >>setup.log
			INSTALL_CACHE="$INSTALL_CACHE $pkg"
		else
			missing+=("$pkg")
		fi
	done

	# Nichts zu tun
	if [ ${#missing[@]} -eq 0 ]; then
		echo -e "${GREEN}✓ Keine neuen Pakete zu installieren${RESET}"
		return 0
	fi

	# Batch-Installation der fehlenden Pakete
	wait_for_apt
	if sudo DEBIAN_FRONTEND=noninteractive apt install -y "${missing[@]}" 2>&1 | tee -a apt_install.log; then
		local failed=()
		for pkg in "${missing[@]}"; do
			if dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
				echo -e "${GREEN}✓ $pkg erfolgreich installiert${RESET}"
				echo "$(date): $pkg erfolgreich installiert" >>setup.log
				INSTALL_CACHE="$INSTALL_CACHE $pkg"
			else
				# Meta-Paket-Fallback prüfen
				if grep -qE "(is already the newest version|newly installed)" apt_install.log; then
					echo -e "${GREEN}✓ $pkg scheint installiert (Meta/keine Aktion)${RESET}"
					echo "$(date): $pkg installiert (Meta/keine Aktion)" >>setup.log
					INSTALL_CACHE="$INSTALL_CACHE $pkg"
				else
					echo -e "${RED}✗ $pkg Installation fehlgeschlagen${RESET}"
					echo "$(date): $pkg Installation fehlgeschlagen" >>errors.log
					failed+=("$pkg")
				fi
			fi
		done

		[ ${#failed[@]} -eq 0 ]
		return $?
	else
		# Wenn Batch fehlschlägt: Einzelinstallation für bessere Diagnose
		echo -e "${RED}✗ Batch-Installation fehlgeschlagen, versuche Einzelinstallation${RESET}"
		for pkg in "${missing[@]}"; do
			wait_for_apt
			if sudo DEBIAN_FRONTEND=noninteractive apt install -y "$pkg" 2>&1 | tee -a apt_install.log; then
				if dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
					echo -e "${GREEN}✓ $pkg erfolgreich installiert${RESET}"
					echo "$(date): $pkg erfolgreich installiert" >>setup.log
					INSTALL_CACHE="$INSTALL_CACHE $pkg"
				else
					echo -e "${RED}✗ $pkg Installation fehlgeschlagen${RESET}"
					echo "$(date): $pkg Installation fehlgeschlagen" >>errors.log
					return 1
				fi
			else
				echo -e "${RED}✗ $pkg Installation fehlgeschlagen${RESET}"
				echo "$(date): $pkg Installation fehlgeschlagen" >>errors.log
				return 1
			fi
		done
		return 0
	fi
}

###############################################################################
# System-Update & Essentials
###############################################################################
echo -e "${BLUE}[STEP] System updaten${RESET}"
wait_for_apt
sudo apt update
if [ $? -eq 0 ]; then
	echo -e "${GREEN}✓ apt update erfolgreich${RESET}"
	echo "$(date): apt update erfolgreich" >>setup.log
else
	echo -e "${RED}✗ apt update fehlgeschlagen${RESET}"
	echo "$(date): apt update fehlgeschlagen" >>errors.log
fi

echo -e "${BLUE}[STEP] universe Repository hinzufügen${RESET}"
wait_for_apt
sudo add-apt-repository universe -y
if [ $? -eq 0 ]; then
	echo -e "${GREEN}✓ universe Repository hinzugefügt${RESET}"
	echo "$(date): universe Repository hinzugefügt" >>setup.log
else
	echo -e "${RED}✗ universe Repository fehlgeschlagen${RESET}"
	echo "$(date): universe Repository fehlgeschlagen" >>errors.log
fi

echo -e "${BLUE}[STEP] System upgraden${RESET}"
wait_for_apt
sudo apt upgrade -y
if [ $? -eq 0 ]; then
	echo -e "${GREEN}✓ apt upgrade erfolgreich${RESET}"
	echo "$(date): apt upgrade erfolgreich" >>setup.log
else
	echo -e "${RED}✗ apt upgrade fehlgeschlagen${RESET}"
	echo "$(date): apt upgrade fehlgeschlagen" >>errors.log
fi

# Installs essential packages:
echo -e "${BLUE}[STEP] Essentielle Pakete installieren${RESET}"
# install_and_check "git"
# install_and_check "ca-certificates"
# install_and_check "curl"
# install_and_check "gnupg"
# install_and_check "python3-pip"
# install_and_check "joystick"
# install_and_check " jstest-gtk"
# install_and_check "evtest"
FAILED_PACKAGES=()
for pkg in git ca-certificates curl gnupg python3-pip joystick jstest-gtk evtest; do
	if ! install_and_check "$pkg"; then
		FAILED_PACKAGES+=("$pkg")
	fi
done

if [ ${#FAILED_PACKAGES[@]} -gt 0 ]; then
	echo -e "${RED}Folgende Pakete konnten nicht installiert werden: ${FAILED_PACKAGES[*]}${RESET}"
else
	echo -e "${GREEN}✓ Alle essentiellen Pakete erfolgreich installiert${RESET}"
fi

sudo apt install -y ca-certificates curl gnupg
# Add GPG Key
echo -e "${BLUE}[STEP] GPG-Schlüssel hinzufügen${RESET}"
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg |
	sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
# Add Docker Repository for Ubuntu
echo -e "${BLUE}[STEP] Docker-Repository hinzufügen${RESET}"
echo \
	"deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" |
	sudo tee /etc/apt/sources.list.d/docker.list >/dev/null

# Update
echo -e "${BLUE}[STEP] Paketlisten aktualisieren${RESET}"
sudo apt update
echo -e "${GREEN}[DONE] Paketlisten aktualisiert${RESET}"

###############################################################################
# Docker + Docker Compose
###############################################################################
echo -e "${BLUE}[STEP] Docker installieren${RESET}"
# sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
# install_and_check "docker-ce"
# install_and_check "docker-ce-cli"
# install_and_check "containerd.io"
# install_and_check "docker-buildx-plugin"
# install_and_check "docker-compose-plugin"
docker_pkgs=(sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin)
wait_for_apt
for pkg in docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin; do
	install_and_check "$pkg"
done
wait_for_apt
# install_and_check "docker.io"
# install_and_check "docker-compose-plugin"

# Checking Docker installation
echo -e "${BLUE}[STEP] Docker-Installation prüfen${RESET}"
if docker --version &>/dev/null; then
	echo -e "${GREEN}✓ Docker is installed correctly: $(docker --version)${RESET}"
	# Erfolg loggen
	echo -e "${GREEN}$(date): Docker-Prüfung erfolgreich${RESET}" >>setup.log
	# Docker-Dienst aktivieren
	# sudo systemctl enable --now docker
	# Benutzer in Gruppen (ab- und wieder anmelden, oder 'newgrp' nutzen)
	sudo usermod -aG docker,dialout,video,plugdev,gpio,i2c,spi $USER
else
	echo -e "${RED}✗ Docker installation failed.${RESET}"
	# Logge Fehler mit trap (automatisch bei ERR)
	echo -e "${RED}✗ Docker installation failed.${RESET}"
	trap 'echo "${RED} $(date): Fehler bei Docker-Prüfung auf $LINENO" >> errors.log' ERR
	# Zusätzlich manuell loggen
	echo -e "${RED}$(date): Docker-Fehler" >>errors.log
fi

# Checking Docker Compose installation
echo -e "${BLUE}[STEP]${RESET} Docker Compose-Installation prüfen"
if docker compose version &>/dev/null; then
	echo -e "${GREEN}✓ Docker Compose is installed correctly: $(docker compose version)${RESET}"
else
	echo -e "${RED}✗ Docker Compose installation failed.${RESET}"
	echo -e "${RED}$(date): Docker Compose-Fehler${RESET}" >>errors.log
	# trap 'echo "${RED}$(date): Fehler bei Compose-Prüfung auf $LINENO${RESET}" >> errors.log' ERR
fi
###############################################################################
# Zeitzone Europe/Berlin (idempotent)
###############################################################################
echo -e "${BLUE}[STEP] Zeitzone: Europe/Berlin${RESET}"
CURRENT_TZ=$(timedatectl show --property=Timezone --value 2>/dev/null || echo "")
if [ "$CURRENT_TZ" != "Europe/Berlin" ]; then
	sudo timedatectl set-timezone Europe/Berlin
	sudo sed -i '/^TZ=/d' /etc/environment 2>/dev/null || true
	echo 'TZ=Europe/Berlin' | sudo tee -a /etc/environment >/dev/null
	echo -e "${GREEN}✓ Zeitzone auf Europe/Berlin gesetzt${RESET}"
	echo "$(date): Zeitzone auf Europe/Berlin gesetzt" >>setup.log
else
	echo -e "${GREEN}✓ Zeitzone bereits Europe/Berlin${RESET}"
fi

###############################################################################
# ROS2 Humble Installation (optional)
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#
###############################################################################
echo -e "${BLUE}[ABFRAGE] Soll ros-humble-desktop installiert werden? (y/n)${RESET}"
read -r response
if [[ "$response" =~ ^[Yy]$ ]]; then
	sudo apt update
	wait_for_apt
	export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
	curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
	sudo dpkg -i /tmp/ros2-apt-source.deb
	wait_for_apt
	sudo apt update
	wait_for_apt
	echo -e "${BLUE}[ROS2 Base Install]${RESET}"
	install_and_check "ros-humble-ros-base" # Für ROS2
	if [ $? -eq 0 ]; then
		echo -e "${GREEN}✓ ros-humble-ros-base installiert${RESET}"
	else
		echo -e "${RED}✗ ros-humble-ros-base Installation fehlgeschlagen${RESET}"
	fi
	wait_for_apt
	echo -e "${BLUE}[ROS2 Base dev tools] installiere dev tools${RESET}"
	install_and_check "ros-dev-tools" # Für ROS2
	if [ $? -eq 0 ]; then
		echo -e "${GREEN}✓ ros-dev-tools installiert${RESET}"
	else
		echo -e "${RED}✗ ros-dev-tools Installation fehlgeschlagen${RESET}"
	fi
else
	echo -e "${BLUE}Überspringe ros-humble-desktop${RESET}"
fi

###############################################################################
# I2C/SPI/Kamera aktivieren
###############################################################################
echo -e "${BLUE}[STEP]${RESET} I2C/SPI/Kamera aktivieren"
CFG=/boot/firmware/config.txt
# echo -e "${BLUE}[INFO] Konfigurationsdatei: cp $CFG $CFG.bak${RESET}"
sudo cp "$CFG" "$CFG.bak"
sudo sed -i '/^dtparam=i2c_arm=/d; /^dtparam=spi=/d; /^camera_auto_detect=/d' "$CFG" 2>/dev/null || true
echo -e 'dtparam=i2c_arm=on\ndtparam=spi=on\ncamera_auto_detect=1' | sudo tee -a "$CFG" >/dev/null
echo 'i2c-dev
spi-bcm2835
spi-dev' | sudo tee /etc/modules-load.d/rpi-interfaces.conf >/dev/null
echo -e "${GREEN}✓ Schnittstellen aktiviert${RESET}"

###############################################################################
# Swap 2GB
###############################################################################
echo -e "${BLUE}[STEP] Swap auf 2GB${RESET}"
if [ ! -f /swapfile ] || [ "$(stat -c%s /swapfile 2>/dev/null)" -lt $((2 * 1024 * 1024 * 1024)) ]; then
	sudo swapoff /swapfile 2>/dev/null || true
	sudo rm /swapfile 2>/dev/null || true
	sudo fallocate -l 2G /swapfile && sudo chmod 600 /swapfile
	sudo mkswap /swapfile && sudo swapon /swapfile
	grep -q '/swapfile' /etc/fstab || echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab >/dev/null
	echo -e "${GREEN}✓ Swap 2GB konfiguriert${RESET}"
fi

echo -e "${BLUE}Swap Status:${RESET}"
free -h | grep -i swap

echo -e "${GREEN}[✓] Setup fertig!${RESET}"
echo -e "${BLUE}Reboot empfohlen: sudo reboot${RESET}"
echo "$(date): Setup abgeschlossen" >>setup.log

###############################################################################
# Pico SDK 2.0.x + Picotool
###############################################################################
echo -e "${BLUE}[STEP] Pico SDK Abhängigkeiten installieren${RESET}"

# Prüfe, ob alle benötigten Pakete bereits installiert sind
PICO_PKGS=(build-essential cmake ninja-build gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib libusb-1.0-0-dev pkg-config)
ALREADY_INSTALLED=()
NEED_INSTALL=()
for pkg in "${PICO_PKGS[@]}"; do
	if dpkg -l 2>/dev/null | grep -qE "^ii\s+$pkg\s"; then
		ALREADY_INSTALLED+=("$pkg")
	else
		NEED_INSTALL+=("$pkg")
	fi
done

if [ ${#ALREADY_INSTALLED[@]} -gt 0 ]; then
	echo -e "${GREEN}✓ Bereits installiert: ${ALREADY_INSTALLED[*]}${RESET}"
fi
if [ ${#NEED_INSTALL[@]} -gt 0 ]; then
	for pkg in "${NEED_INSTALL[@]}"; do
		install_and_check "$pkg"
	done
else
	echo -e "${GREEN}✓ Alle Pico SDK Abhängigkeiten sind bereits installiert${RESET}"
fi

# --- Pico SDK Installation nur, wenn nicht vorhanden ---
if [ -d "$SDK_DEST" ]; then
	log "✓ Pico SDK bereits vorhanden: $SDK_DEST"
	SDK="$SDK_DEST"
	SDK_INSTALLED=1
else
	mkdir -p "$(dirname "$SDK_DEST")"
	log "Versuche Pico SDK Version $PICO_SDK_VERSION zu klonen..."
	if git clone --branch "$PICO_SDK_VERSION" --depth 1 --recursive https://github.com/raspberrypi/pico-sdk.git "$SDK_DEST" 2>/dev/null; then
		log "✓ Pico SDK $PICO_SDK_VERSION erfolgreich geklont"
		SDK="$SDK_DEST"
		SDK_INSTALLED=1
	else
		log_warn "✗ Version $PICO_SDK_VERSION nicht gefunden, versuche neueste Version..."
		if git clone --recursive --depth 1 https://github.com/raspberrypi/pico-sdk.git "$SDK_DEST"; then
			log "✓ Neueste Pico SDK erfolgreich geklont"
			SDK="$SDK_DEST"
			SDK_INSTALLED=1
		else
			log_warn "✗ Pico SDK konnte nicht geklont werden"
			FAILURES+=("Pico SDK Clone fehlgeschlagen")
			SKIP_PICOTOOL=1
		fi
	fi
fi

# Pico SDK installiert (Versionsprüfung ausgelassen)
log "Pico SDK bereit: $SDK_DEST"

# Nur bei erfolgreicher SDK-Installation Environment-Variablen setzen
if [ "$SDK_INSTALLED" -eq 1 ] && [ -n "$SDK" ] && [ -d "$SDK" ]; then
	# Direkt $PICO_SDK_PATH (vom Skriptanfang) verwenden
	log "Using Pico SDK at: $PICO_SDK_PATH"

	# Systemweite Konfiguration in /etc/profile.d/
	if sudo sh -c "printf 'export PICO_SDK_PATH=\"%s\"\n' \"$PICO_SDK_PATH\" > /etc/profile.d/pico_sdk.sh"; then
		sudo chmod 644 /etc/profile.d/pico_sdk.sh
		log "Added PICO_SDK_PATH to /etc/profile.d/pico_sdk.sh"
	else
		log_warn "Failed to write /etc/profile.d/pico_sdk.sh"
		record_failure "Writing /etc/profile.d/pico_sdk.sh"
	fi

	# Benutzer-spezifische .bashrc Konfiguration
	BASHRC="$HOME/.bashrc"

	# PICO_SDK_PATH hinzufügen (idempotent)
	if ! grep -q "PICO_SDK_PATH=" "$BASHRC" 2>/dev/null; then
		if echo "export PICO_SDK_PATH=\"$PICO_SDK_PATH\"" >>"$BASHRC"; then
			log "Added PICO_SDK_PATH to ~/.bashrc"
		else
			log_warn "Failed to write to ~/.bashrc"
			record_failure "Writing to ~/.bashrc"
		fi
	else
		log "PICO_SDK_PATH already present in ~/.bashrc"
	fi

	# ~/.local/bin zu PATH hinzufügen (für Picotool)
	if ! grep -q '.local/bin' "$BASHRC" 2>/dev/null; then
		if echo 'export PATH="$HOME/.local/bin:$PATH"' >>"$BASHRC"; then
			log "Added ~/.local/bin to PATH in ~/.bashrc"
		else
			log_warn "Failed to add PATH to ~/.bashrc"
			record_failure "Adding PATH to ~/.bashrc"
		fi
	else
		log "~/.local/bin already in PATH"
	fi
else
	log_warn "Pico SDK directory not available; skipping SDK environment setup."
	record_failure "Pico SDK setup"
	SKIP_PICOTOOL=1
	SDK_INSTALLED=0
fi

# --- Picotool Installation nur, wenn nicht vorhanden ---
if command -v picotool >/dev/null 2>&1; then
	echo -e "${GREEN}✓ Picotool bereits installiert: $(command -v picotool)${RESET}"
	PICOTOOL_INSTALLED=1
else
	TMP_DIR=$(mktemp -d)
	echo -e "${BLUE}[STEP] Picotool wird gebaut und installiert${RESET}"
	# Klonen
	if git clone --branch "$PICOTOOL_VERSION" --depth 1 https://github.com/raspberrypi/picotool.git "$TMP_DIR/picotool"; then
		# Build
		cmake -S "$TMP_DIR/picotool" -B "$TMP_DIR/build" -G Ninja -DCMAKE_BUILD_TYPE=Release -DPICO_SDK_PATH="$PICO_SDK_PATH"
		cmake --build "$TMP_DIR/build" --target picotool
		PICOTOOL_BIN="$TMP_DIR/build/picotool"
		[ -f "$PICOTOOL_BIN" ] || PICOTOOL_BIN="$TMP_DIR/build/tools/picotool/picotool"
		if [ -f "$PICOTOOL_BIN" ]; then
			mkdir -p "$HOME/.local/bin"
			install -m755 "$PICOTOOL_BIN" "$HOME/.local/bin/picotool"
			echo -e "${GREEN}✓ Picotool installiert: $HOME/.local/bin/picotool${RESET}"
			PICOTOOL_INSTALLED=1
		else
			echo -e "${RED}✗ Picotool Build fehlgeschlagen${RESET}"
		fi
	else
		echo -e "${RED}✗ Picotool konnte nicht geklont werden${RESET}"
	fi
	rm -rf "$TMP_DIR"
fi

if [ "$PICOTOOL_INSTALLED" -eq 1 ]; then
	log "Picotool installed successfully."
	# PATH für aktuelle Session setzen, falls ~/.local/bin nicht enthalten
	if [[ ":$PATH:" != *":$HOME/.local/bin:"* ]]; then
		export PATH="$HOME/.local/bin:$PATH"
		log "PATH für aktuelle Session ergänzt: $HOME/.local/bin"
	fi
elif [ "$SKIP_PICOTOOL" -eq 1 ]; then
	log_warn "Picotool installation skipped or incomplete due to earlier issues."
else
	log_warn "Picotool installation did not complete."
fi

# Zusammenfassung am Ende
echo -e "\n${BLUE}=== Installation Summary ===${RESET}"
if [ "$SDK_INSTALLED" -eq 1 ]; then
	echo -e "${GREEN}✓ Pico SDK installed at: $PICO_SDK_PATH${RESET}"
	if [ -n "${DETECTED_SDK_VERSION:-}" ]; then
		echo -e "${GREEN}  Version: $DETECTED_SDK_VERSION${RESET}"
	fi
else
	echo -e "${RED}✗ Pico SDK installation failed${RESET}"
fi

if [ "$PICOTOOL_INSTALLED" -eq 1 ]; then
	echo -e "${GREEN}✓ Picotool installed at: $HOME/.local/bin/picotool${RESET}"
	if [ -n "${PICOTOOL_VERSION_DETECTED:-}" ]; then
		echo -e "${GREEN}  Version: $PICOTOOL_VERSION_DETECTED${RESET}"
	fi
else
	echo -e "${RED}✗ Picotool installation failed or skipped${RESET}"
fi

if [ "${#FAILURES[@]}" -gt 0 ]; then
	log_warn "Encountered ${#FAILURES[@]} non-fatal issue(s):"
	for failure in "${FAILURES[@]}"; do
		log_warn " - ${failure}"
	done
else
	log "Completed all steps without errors."
fi

echo -e "${GREEN}--- install_picotool_and_sdk.sh: done ---${RESET}"
exit 0
