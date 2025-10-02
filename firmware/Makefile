.PHONY: clean build compile flash

# Clean: Entferne Build-Verzeichnis
clean:
	rm -rf build

# Build: CMake konfigurieren und kompilieren
build:
	mkdir -p build
	cd build && cmake ..
	make -C build

build_release: clean
	mkdir -p build_release
	cd build_release && cmake .. -DCMAKE_BUILD_TYPE=Release
	make -C build_release -j $(nproc)

# Compile: Alias für build (kompiliert alles)
compile: clean build

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
