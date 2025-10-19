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
# Aktuelle Versionen (automatisch latest holen)
PICO_SDK_VERSION="latest" # Holt neueste stable Version
PICOTOOL_VERSION="latest" # Holt neueste stable Version
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

install_and_check() {
	local package=$1
	echo -e "${BLUE}[STEP] $package installieren${RESET}"

	# Prüfe zuerst, ob Paket bereits installiert ist
	if dpkg -l 2>/dev/null | grep -qE "^ii\s+$package\s"; then
		echo -e "${GREEN}✓ $package bereits installiert${RESET}"
		echo "$(date): $package bereits installiert" >>setup.log
		return 0
	fi

	wait_for_apt
	if sudo apt install -y "$package" 2>&1 | tee -a apt_install.log; then
		# Nach Installation nochmal prüfen
		if dpkg -l 2>/dev/null | grep -qE "^ii\s+$package(\s|:)"; then
			echo -e "${GREEN}✓ $package erfolgreich installiert${RESET}"
			echo "$(date): $package erfolgreich installiert" >>setup.log
			return 0
		else
			# Bei Meta-Paketen: prüfe ob apt-Befehl erfolgreich war
			if grep -qE "(is already the newest version|newly installed)" apt_install.log; then
				echo -e "${GREEN}✓ $package erfolgreich installiert${RESET}"
				echo "$(date): $package erfolgreich installiert" >>setup.log
				return 0
			fi
		fi
	fi

	echo -e "${RED}✗ $package Installation fehlgeschlagen${RESET}"
	echo "$(date): $package Installation fehlgeschlagen" >>errors.log
	return 1
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

# sudo apt install -y ca-certificates curl gnupg
# # Add GPG Key
# echo -e "${BLUE}[STEP] GPG-Schlüssel hinzufügen${RESET}"
# sudo install -m 0755 -d /etc/apt/keyrings
# curl -fsSL https://download.docker.com/linux/ubuntu/gpg |
# 	sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
# # Add Docker Repository for Ubuntu
# echo -e "${BLUE}[STEP] Docker-Repository hinzufügen${RESET}"
# echo \
# 	"deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
#   https://download.docker.com/linux/ubuntu \
#   $(lsb_release -cs) stable" |
# 	sudo tee /etc/apt/sources.list.d/docker.list >/dev/null

# Update
echo -e "${BLUE}[STEP] Paketlisten aktualisieren${RESET}"
sudo apt update

###############################################################################
# Docker + Docker Compose
###############################################################################
echo -e "${BLUE}[STEP] Docker installieren${RESET}"
# # sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
# install_and_check "docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin"
wait_for_apt
install_and_check "docker.io"
install_and_check "docker-compose-plugin"

# Checking Docker installation
echo -e "${BLUE}[STEP] Docker-Installation prüfen${RESET}"
if docker --version &>/dev/null; then
	echo -e "${GREEN}✓ Docker is installed correctly: $(docker --version)${RESET}"
	# Erfolg loggen
	echo -e "${GREEN}$(date): Docker-Prüfung erfolgreich${RESET}" >>setup.log
	# Docker-Dienst aktivieren
	sudo systemctl enable --now docker
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

# Optionale Installation für ROS2
echo -e "${BLUE}[ABFRAGE] Soll ros-humble-desktop installiert werden? (y/n)${RESET}"
read -r response
if [[ "$response" =~ ^[Yy]$ ]]; then
	install_and_check "ros-humble-desktop" # Für ROS2
else
	echo -e "${BLUE}Überspringe ros-humble-desktop${RESET}"
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
# --no-install-recommends sorgt für eine schlankere Installation
# install_and_check "build-essential"
# install_and_check "cmake"
# install_and_check "gcc-arm-none-eabi"
# install_and_check "libnewlib-arm-none-eabi"
# install_and_check "libstdc++-arm-none-eabi-newlib"
# install_and_check "ninja-build"
for pkg in build-essential cmake ninja-build gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib; do
	install_and_check "$pkg"
done

echo -e "${BLUE}[STEP] >>> --- install_picotool_and_sdk.sh: start ---${RESET}"

log_info() {
	printf "${BLUE}[INFO] %s${RESET}\n" "$*"
}

log_warn() {
	printf "${RED}[WARN] %s${RESET}\n" "$*" >&2
}

record_failure() {
	FAILURES+=("$1")
}

cleanup_sdk() {
	if [ -n "${TMP_DIR:-}" ] && [ -d "${TMP_DIR}" ]; then
		rm -rf "${TMP_DIR}"
	fi
}
trap cleanup_sdk EXIT

run_command() {
	local desc="$1"
	shift
	log_info "$desc"
	if "$@"; then
		return 0
	else
		local rc=$?
		log_warn "$desc failed (exit $rc); continuing."
		record_failure "$desc (exit $rc)"
		return $rc
	fi
}

log_info "Checking for existing Pico SDK..."
CANDIDATES=("$SDK_DEST" "$HOME/pico-sdk" "/usr/local/share/pico-sdk")

SDK=""
for p in "${CANDIDATES[@]}"; do
	if [ -n "$p" ] && [ -d "$p" ]; then
		SDK="$p"
		log_info "Found existing Pico SDK at: $SDK"
		SDK_INSTALLED=1
		break
	fi
done

if [ -z "$SDK" ]; then
	log_info "Pico SDK not found. Attempting to clone into ${SDK_DEST}."
	SDK_PARENT="$(dirname "$SDK_DEST")"
	if run_command "Creating parent directory ${SDK_PARENT}" mkdir -p "$SDK_PARENT"; then
		if run_command "Cloning Pico SDK repository (this may take a few minutes)" \
			git clone --recursive --depth 1 https://github.com/raspberrypi/pico-sdk.git "$SDK_DEST"; then
			SDK="$SDK_DEST"
			SDK_INSTALLED=1
			log_info "Successfully cloned Pico SDK to ${SDK_DEST}"
		else
			log_warn "Failed to clone Pico SDK. Continuing without it."
			SKIP_PICOTOOL=1
		fi
	else
		log_warn "Could not create parent directory for Pico SDK; skipping clone."
		SKIP_PICOTOOL=1
	fi
fi

# Nur bei erfolgreicher SDK-Installation Environment-Variablen setzen
if [ "$SDK_INSTALLED" -eq 1 ] && [ -n "$SDK" ] && [ -d "$SDK" ]; then
	if SDK_REAL=$(realpath -s "$SDK" 2>/dev/null); then
		log_info "Using Pico SDK at: $SDK_REAL"
		export PICO_SDK_PATH="$SDK_REAL"

		# Systemweite Konfiguration in /etc/profile.d/
		if sudo sh -c "printf 'export PICO_SDK_PATH=\"%s\"\n' '$SDK_REAL' > /etc/profile.d/pico_sdk.sh"; then
			sudo chmod 644 /etc/profile.d/pico_sdk.sh
			log_info "Added PICO_SDK_PATH to /etc/profile.d/pico_sdk.sh"
		else
			log_warn "Failed to write /etc/profile.d/pico_sdk.sh"
			record_failure "Writing /etc/profile.d/pico_sdk.sh"
		fi

		# Benutzer-spezifische .bashrc Konfiguration
		BASHRC="$HOME/.bashrc"
		if ! grep -q "PICO_SDK_PATH=" "$BASHRC" 2>/dev/null; then
			if echo "export PICO_SDK_PATH=\"$SDK_REAL\"" >>"$BASHRC"; then
				log_info "Added PICO_SDK_PATH to ~/.bashrc"
			else
				log_warn "Failed to write to ~/.bashrc"
				record_failure "Writing to ~/.bashrc"
			fi
		else
			log_info "PICO_SDK_PATH already present in ~/.bashrc"
		fi
	else
		log_warn "Unable to resolve Pico SDK path using realpath; skipping environment export."
		record_failure "Resolving Pico SDK path"
		SKIP_PICOTOOL=1
		SDK_INSTALLED=0
	fi
else
	log_warn "Pico SDK directory not available; skipping SDK environment setup."
	record_failure "Pico SDK setup"
	SKIP_PICOTOOL=1
	SDK_INSTALLED=0
fi

if [ "$SKIP_PICOTOOL" -eq 0 ]; then
	MISSING_CMDS=()
	for cmd in git cmake ninja install; do
		if ! command -v "$cmd" >/dev/null 2>&1; then
			MISSING_CMDS+=("$cmd")
		fi
	done
	if [ "${#MISSING_CMDS[@]}" -gt 0 ]; then
		log_warn "Missing required command(s) for picotool build: ${MISSING_CMDS[*]}"
		record_failure "Missing commands for picotool: ${MISSING_CMDS[*]}"
		SKIP_PICOTOOL=1
	fi
fi

if [ "$SKIP_PICOTOOL" -eq 0 ]; then
	TMP_DIR=$(mktemp -d) || {
		log_warn "Unable to create temporary directory for picotool build."
		record_failure "Creating temporary directory for picotool"
		SKIP_PICOTOOL=1
	}
fi

if [ "$SKIP_PICOTOOL" -eq 0 ]; then
	if run_command "Cloning picotool repository" git clone --depth 1 https://github.com/raspberrypi/picotool.git "$TMP_DIR/picotool"; then
		if run_command "Configuring picotool build" cmake -S "$TMP_DIR/picotool" -B "$TMP_DIR/build" -G Ninja -DCMAKE_BUILD_TYPE=Release; then
			JOBS=1
			if command -v nproc >/dev/null 2>&1; then
				JOBS=$(nproc)
			fi
			if run_command "Building picotool" cmake --build "$TMP_DIR/build" --target picotool -j"$JOBS"; then
				PICOTOOL_BIN="$TMP_DIR/build/tools/picotool/picotool"
				if [ -z "$PICOTOOL_BIN" ] || [ ! -f "$PICOTOOL_BIN" ]; then
					log_warn "Picotool binary not found after build."
					record_failure "Locating built picotool binary"
					SKIP_PICOTOOL=1
				else
					if run_command "Installing picotool binary" install -Dm755 "$PICOTOOL_BIN" /usr/local/bin/picotool; then
						PICOTOOL_INSTALLED=1
					else
						SKIP_PICOTOOL=1
					fi
				fi
			else
				SKIP_PICOTOOL=1
			fi
		else
			SKIP_PICOTOOL=1
		fi
	else
		SKIP_PICOTOOL=1
	fi
fi

if [ "$PICOTOOL_INSTALLED" -eq 1 ]; then
	log_info "Picotool installed successfully."
elif [ "$SKIP_PICOTOOL" -eq 1 ]; then
	log_warn "Picotool installation skipped or incomplete due to earlier issues."
else
	log_warn "Picotool installation did not complete."
fi

# Zusammenfassung am Ende
echo -e "\n${BLUE}=== Installation Summary ===${RESET}"
if [ "$SDK_INSTALLED" -eq 1 ]; then
	echo -e "${GREEN}✓ Pico SDK installed at: $SDK_REAL${RESET}"
else
	echo -e "${RED}✗ Pico SDK installation failed${RESET}"
fi

if [ "$PICOTOOL_INSTALLED" -eq 1 ]; then
	echo -e "${GREEN}✓ Picotool installed successfully${RESET}"
else
	echo -e "${RED}✗ Picotool installation failed or skipped${RESET}"
fi

if [ "${#FAILURES[@]}" -gt 0 ]; then
	log_warn "Encountered ${#FAILURES[@]} non-fatal issue(s):"
	for failure in "${FAILURES[@]}"; do
		log_warn " - ${failure}"
	done
else
	log_info "Completed all steps without errors."
fi

echo "--- install_picotool_and_sdk.sh: done ---"
exit 0
