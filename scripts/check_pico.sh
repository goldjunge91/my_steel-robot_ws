#!/bin/bash
set -euo pipefail

echo "=== date ==="
date

echo "=== whoami ==="
whoami

echo "=== dmesg tail ==="
dmesg | tail -n 50 || true

echo "=== lsusb ==="
lsusb || true

echo "=== /dev listing ==="
ls -l /dev/ttyACM* /dev/ttyUSB* /dev/pico_micro_ros || true

echo "=== udevadm for /dev/ttyACM0 ==="
if [ -e /dev/ttyACM0 ]; then
  udevadm info -a -n /dev/ttyACM0 | sed -n '1,240p' || true
else
  echo "/dev/ttyACM0 does not exist"
fi

echo "=== processes holding /dev/ttyACM0 ==="
sudo lsof /dev/ttyACM0 || true

echo "=== end ==="
