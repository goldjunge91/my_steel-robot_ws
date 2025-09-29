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

# # Flash: UF2-Datei auf Raspberry Pi Pico laden und rebooten
# flash:	
# 	cd /home/marco/ros2_steel_ws/my_steel-robot_ws/firmware
# 	cp build/src/my_firmware.uf2 /media/marco/RPI-RP2/
# # 	picotool load -f build/src/my_firmware.uf2
# # 	picotool reboot
