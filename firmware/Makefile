.DEFAULT_GOAL := build

.PHONY: clean build compile flash test test-verbose test-coverage

# Clean: Entferne Build-Verzeichnis
clean:
	rm -rf build/ releases log 

# Clean tests
clean-tests:
	rm -rf tests/build

# Clean all
clean-all: clean clean-tests
	rm -rf build_release

# Build: CMake konfigurieren und kompilieren
build:
	mkdir -p build
	cd build && cmake ..
	make -C build
	@echo "Copying .uf2 file to releases/"
	@mkdir -p releases
	@cp build/src/my_firmware.uf2 releases/my_firmware_$(shell date +%Y%m%d_%H%M%S).uf2
	@cp build/src/my_firmware.uf2 releases/my_firmware_latest.uf2
	@echo "Firmware saved to releases/"

build_release: clean
	mkdir -p build_release
	cd build_release && cmake .. -DCMAKE_BUILD_TYPE=Release
	make -C build_release -j $(nproc)
	@echo "Copying release .uf2 file to releases/"
	@mkdir -p releases
	@cp build_release/src/my_firmware.uf2 releases/my_firmware_release_$(shell date +%Y%m%d_%H%M%S).uf2
	@cp build_release/src/my_firmware.uf2 releases/my_firmware_release_latest.uf2
	@echo "Release firmware saved to releases/"


# Create versioned release (use git tag)
release:
	@echo "Creating release build..."
	@if [ -z "$(VERSION)" ]; then \
		echo "Error: VERSION not set. Usage: make release VERSION=v1.0.0"; \
		exit 1; \
	fi
	@mkdir -p releases
	@$(MAKE) build_release
	@cp build_release/src/my_firmware.uf2 releases/my_firmware_$(VERSION).uf2
	@echo "Release created: releases/my_firmware_$(VERSION).uf2"
	@echo "Next steps:"
	@echo "  1. git add releases/my_firmware_$(VERSION).uf2"
	@echo "  2. git commit -m 'Release $(VERSION)'"
	@echo "  3. git tag $(VERSION)"
	@echo "  4. git push origin $(VERSION)"
	@echo "  5. Upload releases/my_firmware_$(VERSION).uf2 to GitHub Release"

# Compile: Alias für build (kompiliert alles)
compile: clean build

# Test: Build and run unit tests
test:
	@echo "Building and running unit tests..."
	@mkdir -p tests/build
	@cd tests/build && cmake .. && make -j$(nproc)
	@echo "\n========================================"
	@echo "  Running Firmware Unit Tests"
	@echo "========================================"
	@cd tests/build && ./firmware_tests

# Test with verbose output
test-verbose:
	@echo "Building and running unit tests (verbose)..."
	@mkdir -p tests/build
	@cd tests/build && cmake .. && make -j$(nproc)
	@echo "\n========================================"
	@echo "  Running Firmware Unit Tests (Verbose)"
	@echo "========================================"
	@cd tests/build && ./firmware_tests --gtest_color=yes --gtest_print_time=1

# Test with CTest
test-ctest:
	@echo "Building and running unit tests with CTest..."
	@mkdir -p tests/build
	@cd tests/build && cmake .. && make -j$(nproc)
	@cd tests/build && ctest --output-on-failure --verbose

# Run specific test suite
test-pid:
	@cd tests/build && ./firmware_tests --gtest_filter=MotorPIDTest.*

test-odometry:
	@cd tests/build && ./firmware_tests --gtest_filter=OdometryTest.*

test-math:
	@cd tests/build && ./firmware_tests --gtest_filter=MathUtilsTest.*

# Flash: UF2-Datei auf Raspberry Pi Pico laden
flash: build
	@echo "Flashing firmware to Pico..."
	@if [ -d "/media/marco/RPI-RP2/" ]; then \
		cp build/src/my_firmware.uf2 /media/marco/RPI-RP2/ && \
		echo "Firmware flashed via USB mass storage"; \
	elif command -v picotool >/dev/null 2>&1; then \
		picotool load -f build/src/my_firmware.uf2 && \
		picotool reboot && \
		echo "Firmware flashed via picotool"; \
	else \
		echo "Error: No flash method available. Either mount Pico in BOOTSEL mode or install picotool"; \
		exit 1; \
	fi

# Flash release version
flash-release: build_release
	@echo "Flashing release firmware to Pico..."
	@if [ -d "/media/marco/RPI-RP2/" ]; then \
		cp build_release/src/my_firmware.uf2 /media/marco/RPI-RP2/ && \
		echo "Release firmware flashed via USB mass storage"; \
	elif command -v picotool >/dev/null 2>&1; then \
		picotool load -f build_release/src/my_firmware.uf2 && \
		picotool reboot && \
		echo "Release firmware flashed via picotool"; \
	else \
		echo "Error: No flash method available. Either mount Pico in BOOTSEL mode or install picotool"; \
		exit 1; \
	fi
