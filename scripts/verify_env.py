import subprocess
import os

def check_command(cmd_name, is_alias=False):
    """Prüft, ob ein Befehl oder Alias in einer interaktiven Bash existiert."""
    check_type = "alias" if is_alias else "type"
    # Wir starten eine interaktive Shell, um die .bashrc zu laden
    proc = subprocess.run(
        ["bash", "-ic", f"{check_type} {cmd_name}"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    return proc.returncode == 0

# Liste deiner Tools und Aliase aus der .bashrc
checks = {
    "Core Tools": ["shfmt", "just", "gh", "docker", "nvm"],
    "Pico SDK": ["picotool"],
    "ROS 2 Aliase": ["cb", "cdws", "sbs", "sbr"],
    "fzf Funktionen": ["rte", "rtl", "rnl", "rbuild"]
}

print(f"{' COMPONENT ':=^40}")
for category, items in checks.items():
    print(f"\n{category}:")
    for item in items:
        status = "✅ OK" if check_command(item) else "❌ MISSING"
        print(f"  {item:<15} {status}")

print(f"\n{' CHECK FINISHED ':=^40}")
