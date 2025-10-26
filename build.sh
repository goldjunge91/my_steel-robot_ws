#!/bin/bash
set -u -o pipefail -E
# set -e

# Ensure ROS environment is sourced so CMake can find ament_cmake and friends
ROS_DISTRO=${ROS_DISTRO:-humble}
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

# Prevent unbound variable issues inside setup scripts under `set -u`
if [ -z "${AMENT_TRACE_SETUP_FILES+x}" ]; then
	export AMENT_TRACE_SETUP_FILES=""
fi

if [ -f "$ROS_SETUP" ]; then
	# shellcheck disable=SC1090
	source "$ROS_SETUP"
fi

if [ -f "install/setup.bash" ]; then
	# shellcheck disable=SC1091
	source "install/setup.bash"
fi

# Set the default build type
BUILD_TYPE=RelWithDebInfo
colcon build \
	--merge-install \
	--symlink-install \
	--cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
	-Wall -Wextra -Wpedantic

# #!/bin/bash

# set -u -o pipefail -E
# # set -x  # Debug-Modus (optional)
# # -u: Fehler bei undefinierten Variablen
# # -o pipefail: Fehler in Pipelines beachten
# # -x: Debug-Modus (zeigt jeden Befehl)
# # -E: Trap-Funktionen erben Fehler

# # Colored output keeps local feedback clear
# RED='\033[0;31m'
# GREEN='\033[0;32m'
# YELLOW='\033[1;33m'
# BLUE='\033[0;34m'
# NC='\033[0m'

# if [ -z "${AMENT_TRACE_SETUP_FILES+x}" ]; then
# 	export AMENT_TRACE_SETUP_FILES=""
# fi

# log_step() {
# 	echo -e "${BLUE}[BUILD]${NC} $1"
# }

# log_info() {
# 	echo -e "${BLUE}[INFO]${NC} $1"
# }

# log_success() {
# 	echo -e "${GREEN}[SUCCESS]${NC} $1"
# }

# log_warning() {
# 	echo -e "${YELLOW}[WARNING]${NC} $1" >&2
# }

# log_error() {
# 	echo -e "${RED}[ERROR]${NC} $1" >&2
# }

# # GitHub Actions helpers
# github_summary() {
# 	if [ "${GITHUB_ACTIONS:-false}" = "true" ] && [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
# 		echo "$*" >>"$GITHUB_STEP_SUMMARY" 2>/dev/null || true
# 	fi
# }

# github_error() {
# 	local title="${1:-}"
# 	local message="${2:-}"
# 	local file="${3:-}"
# 	local line="${4:-}"
# 	if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
# 		local annotation="::error"
# 		if [ -n "$file" ]; then
# 			annotation="${annotation} file=${file}"
# 		fi
# 		if [ -n "$line" ]; then
# 			annotation="${annotation},line=${line}"
# 		fi
# 		annotation="${annotation} title=${title}::${message}"
# 		echo "$annotation"
# 	fi
# }

# safe_source() {
# 	local file="$1"
# 	if [ -f "$file" ]; then
# 		set +u
# 		# shellcheck disable=SC1090
# 		source "$file"
# 		local status=$?
# 		set -u
# 		return $status
# 	fi
# 	return 1
# }

# ROS_DISTRO=${ROS_DISTRO:-humble}
# ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

# # Set AMENT_TRACE_SETUP_FILES to prevent unbound variable errors
# if [ -z "${AMENT_TRACE_SETUP_FILES+x}" ]; then
# 	export AMENT_TRACE_SETUP_FILES=""
# fi

# log_step "Sourcing ROS environment (${ROS_SETUP})"
# if safe_source "$ROS_SETUP"; then
# 	log_success "ROS environment sourced"
# else
# 	log_warning "ROS setup script nicht gefunden – fahre ohne globales ROS fort"
# fi

# if [ -f "install/setup.bash" ]; then
# 	log_step "Sourcing vorhandenes install/setup.bash"
# 	if safe_source "install/setup.bash"; then
# 		log_success "Bestehende Workspace-Umgebung geladen"
# 	else
# 		log_warning "install/setup.bash konnte nicht geladen werden"
# 	fi
# fi

# if ! command -v colcon >/dev/null 2>&1; then
# 	log_error "colcon nicht gefunden – bitte ROS 2 build tools installieren"
# 	exit 1
# fi

# if [ ! -w "." ] && command -v sudo >/dev/null 2>&1; then
# 	log_warning "Workspace-Verzeichnis nicht beschreibbar – passe Besitzrechte an"
# 	sudo chown "$(id -u)":"$(id -g)" .
# fi

# # Ensure workspace directories exist and are writable
# for dir in build install log; do
# 	if ! mkdir -p "$dir" 2>/dev/null; then
# 		if command -v sudo >/dev/null 2>&1 && sudo mkdir -p "$dir"; then
# 			:
# 		else
# 			log_error "Kann Verzeichnis $dir nicht erzeugen"
# 			exit 1
# 		fi
# 	fi
# 	if [ ! -w "$dir" ] && command -v sudo >/dev/null 2>&1; then
# 		if ! sudo chown -R "$(id -u)":"$(id -g)" "$dir"; then
# 			log_error "Verzeichnis $dir ist nicht beschreibbar"
# 			exit 1
# 		fi
# 	elif [ ! -w "$dir" ]; then
# 		log_error "Verzeichnis $dir ist nicht beschreibbar"
# 		exit 1
# 	fi
# done

# # BUILD_TYPE=${BUILD_TYPE:-RelWithDebInfo}
# # COLCON_MIXINS=${COLCON_MIXINS:-release}
# # COLCON_PARALLEL_WORKERS=${COLCON_PARALLEL_WORKERS:-$(nproc)}
# # COLCON_PACKAGES_SELECT=${COLCON_PACKAGES_SELECT:-}
# # COLCON_PACKAGES_UP_TO=${COLCON_PACKAGES_UP_TO:-}
# # COLCON_BUILD_EXTRA_ARGS=${COLCON_BUILD_EXTRA_ARGS:-}

# # # Ensure requested mixins are available
# # ensure_colcon_mixins() {
# # 	if [ -z "$COLCON_MIXINS" ]; then
# # 		return 0
# # 	fi

# # 	if ! command -v colcon >/dev/null 2>&1; then
# # 		return 0
# # 	fi

# # 	if ! command -v colcon-mixin >/dev/null 2>&1 && ! colcon mixin -h >/dev/null 2>&1; then
# # 		log_warning "colcon mixin plugin nicht verfügbar – überspringe Mixins"
# # 		COLCON_MIXINS=""
# # 		return 0
# # 	fi

# # 	if ! colcon mixin show default >/dev/null 2>&1; then
# # 		log_step "Registriere colcon Standard-Mixins"
# # 		colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml >/dev/null 2>&1 || true
# # 	fi

# # 	log_step "Aktualisiere colcon Mixins"
# # 	colcon mixin update default >/dev/null 2>&1 || true

# # 	for mixin in $COLCON_MIXINS; do
# # 		if ! colcon mixin show "$mixin" >/dev/null 2>&1; then
# # 			log_warning "colcon Mixin '$mixin' unbekannt – entferne aus Liste"
# # 			COLCON_MIXINS=$(
# # 				for item in $COLCON_MIXINS; do
# # 					if [ "$item" != "$mixin" ]; then
# # 						printf "%s " "$item"
# # 					fi
# # 				done | sed 's/[[:space:]]*$//'
# # 			)
# # 		fi
# # 	done
# # }

# ensure_colcon_mixins

# log_step "Starte colcon build (BUILD_TYPE=${BUILD_TYPE})"
# log_info "colcon parallel workers: ${COLCON_PARALLEL_WORKERS}"
# if [ -n "$COLCON_MIXINS" ]; then
# 	log_info "colcon mixins: ${COLCON_MIXINS}"
# fi
# if [ -n "$COLCON_PACKAGES_SELECT" ]; then
# 	log_info "Pakete (select): ${COLCON_PACKAGES_SELECT}"
# fi
# if [ -n "$COLCON_PACKAGES_UP_TO" ]; then
# 	log_info "Pakete (up-to): ${COLCON_PACKAGES_UP_TO}"
# fi

# # Record start time for GitHub Actions summary
# start_time=$(date +%s)

# colcon build \
# 	--merge-install \
# 	--symlink-install \
# 	--parallel-workers "${COLCON_PARALLEL_WORKERS}" \
# 	${COLCON_MIXINS:+--mixin ${COLCON_MIXINS}} \
# 	${COLCON_PACKAGES_SELECT:+--packages-select ${COLCON_PACKAGES_SELECT}} \
# 	${COLCON_PACKAGES_UP_TO:+--packages-up-to ${COLCON_PACKAGES_UP_TO}} \
# 	${COLCON_BUILD_EXTRA_ARGS} \
# 	--cmake-args \
# 	"-DCMAKE_BUILD_TYPE=${BUILD_TYPE}" \
# 	"-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
# 	"-DCMAKE_CXX_FLAGS=-Wall -Wextra -Wpedantic"
# build_exit_code=$?

# # Calculate build duration
# end_time=$(date +%s)
# duration=$((end_time - start_time))

# if [ $build_exit_code -ne 0 ]; then
# 	log_error "colcon build failed with exit code $build_exit_code"

# 	if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
# 		github_summary "## ❌ Build Failed"
# 		github_summary ""
# 		github_summary "### 📊 Build Summary"
# 		github_summary "- **Exit Code**: $build_exit_code"
# 		github_summary "- **Duration**: ${duration}s"
# 		github_summary "- **Build Type**: $BUILD_TYPE"
# 		github_summary ""

# 		# Analyze build failures
# 		if [ -d "log/latest_build" ]; then
# 			github_summary "### 🔍 Build Analysis"

# 			# Count failed packages
# 			failed_packages=$(find log/latest_build -name "stderr.log" -exec grep -l "CMake Error\|error:\|fatal error:\|compilation terminated" {} \; 2>/dev/null | wc -l || echo "0")

# 			github_summary "- **Packages with build errors**: $failed_packages"
# 			github_summary ""

# 			# List failed packages
# 			if [ "$failed_packages" -gt 0 ]; then
# 				github_summary "### 📦 Failed Packages"
# 				find log/latest_build -name "stderr.log" -exec grep -l "CMake Error\|error:\|fatal error:\|compilation terminated" {} \; 2>/dev/null |
# 					sed 's|log/latest_build/||' | sed 's|/stderr.log||' | head -10 |
# 					while read -r pkg; do
# 						github_summary "- \`$pkg\`"
# 						# Add error annotation
# 						if [ -f "log/latest_build/$pkg/stderr.log" ]; then
# 							error_msg=$(grep -m1 "CMake Error\|error:\|fatal error:" "log/latest_build/$pkg/stderr.log" 2>/dev/null || echo "Build failed")
# 							github_error "Build Failure" "Package '$pkg' failed: $error_msg" "src/$pkg"
# 						fi
# 					done
# 				github_summary ""
# 			fi
# 		fi

# 		github_summary "### 🛠️ Next Steps to Fix Build"
# 		github_summary ""
# 		github_summary "1. **Check build logs**:"
# 		github_summary "   \`\`\`bash"
# 		github_summary "   ls -la log/latest_build/"
# 		github_summary "   cat log/latest_build/<package_name>/stderr.log"
# 		github_summary "   \`\`\`"
# 		github_summary ""
# 		github_summary "2. **Build specific package with verbose output**:"
# 		github_summary "   \`\`\`bash"
# 		github_summary "   colcon build --packages-select <package_name> --event-handlers console_direct+"
# 		github_summary "   \`\`\`"
# 		github_summary ""
# 		github_summary "3. **Check dependencies**:"
# 		github_summary "   \`\`\`bash"
# 		github_summary "   rosdep check --from-paths src --ignore-src"
# 		github_summary "   \`\`\`"
# 		github_summary ""
# 		github_summary "4. **Clean build (if needed)**:"
# 		github_summary "   \`\`\`bash"
# 		github_summary "   rm -rf build/ install/ log/"
# 		github_summary "   colcon build"
# 		github_summary "   \`\`\`"
# 	fi

# 	exit $build_exit_code
# fi

# log_success "colcon build abgeschlossen (${duration}s)"

# # Add success summary for GitHub Actions
# if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
# 	github_summary "## ✅ Build Successful"
# 	github_summary ""
# 	github_summary "- **Duration**: ${duration}s"
# 	github_summary "- **Build Type**: $BUILD_TYPE"
# 	github_summary "- **ROS Distribution**: $ROS_DISTRO"
# fi

# if [ ! -f "install/setup.bash" ]; then
# 	log_warning "install/setup.bash nicht gefunden – wurde der Build Workspace leer?"
# fi
