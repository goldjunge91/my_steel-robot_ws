#!/usr/bin/env python3
import subprocess
import os

# Farben für die Konsole
GREEN = "\033[92m"
RED = "\033[91m"
BLUE = "\033[94m"
RESET = "\033[0m"

def check_env(cmd, name, is_alias=False):
    """Prüft, ob ein Befehl oder Alias in einer interaktiven Shell existiert."""
    # Wir nutzen 'bash -ic', um die .bashrc tatsächlich zu laden
    check_cmd = f"alias {cmd}" if is_alias else f"type {cmd}"
    
    result = subprocess.run(
        ["bash", "-ic", check_cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    
    if result.returncode == 0:
        print(f"  {GREEN}✓{RESET} {name:<20} gefunden")
        return True
    else:
        print(f"  {RED}✗{RESET} {name:<20} FEHLT")
        return False

print(f"\n{BLUE}=== Umgebungstest für ROS 2 & Pico SDK ==={RESET}\n")

# 1. System Tools
print("System Tools:")
check_env("shfmt", "shfmt")
check_env("just", "just")
check_env("gh", "GitHub CLI")
check_env("docker", "Docker")
check_env("nvm", "NVM (Node Manager)")

# 2. Pico & Toolchain
print("\nPico Development:")
check_env("picotool", "Picotool")
check_env("arm-none-eabi-gcc", "ARM Toolchain")

# 3. ROS 2 & Workspace Aliase
print("\nROS 2 & Aliase:")
check_env("ros2", "ROS 2 Core")
check_env("cb", "Alias: colcon build", is_alias=True)
check_env("sbs", "Alias: Simulation Mode", is_alias=True)
check_env("rte", "fzf: Topic Echo", is_alias=False) # Funktion

# 4. Pfad-Check
print("\nPfad-Validierung:")
pico_path = os.environ.get('PICO_SDK_PATH', 'NICHT GESETZT')
if "pico-sdk" in pico_path:
    print(f"  {GREEN}✓{RESET} PICO_SDK_PATH: {pico_path}")
else:
    print(f"  {RED}✗{RESET} PICO_SDK_PATH: {pico_path}")

print(f"\n{BLUE}==========================================={RESET}\n")
