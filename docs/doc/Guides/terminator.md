# Terminator Guide

## Key Features
- **Grid System**: Split windows horizontally and vertically.
- **Tabs**: Multiple tabs for different workspaces.
- **Broadcast**: Type in multiple terminals simultaneously (Group broadcasting).
- **Layouts**: Save and restore custom window layouts.

## Shortcuts Cheat Sheet

### Window & Tab Management
| Action | Shortcut |
|--------|----------|
| **Split Horizontally** | `Ctrl + Shift + O` |
| **Split Vertically** | `Ctrl + Shift + E` |
| **New Tab** | `Ctrl + Shift + T` |
| **Close Terminal** | `Ctrl + Shift + W` |
| **Close Window** | `Ctrl + Shift + Q` |
| **New Window** | `Ctrl + Shift + I` |

### Navigation & View
| Action | Shortcut |
|--------|----------|
| **Switch Terminal** | `Alt + Arrow Keys` |
| **Resize Terminal** | `Ctrl + Shift + Arrow Keys` |
| **Zoom/Maximize Active** | `Ctrl + Shift + X` (Toggle) |
| **Toggle Fullscreen** | `F11` |
| **Scroll Up/Down** | `Shift + PageUp / PageDown` |
| **Search in Terminal** | `Ctrl + Shift + F` |
| **Clear Scrollback** | `Ctrl + Shift + K` |

### Grouping & Broadcasting (Power User)
Essential for running commands on multiple identical nodes/servers.
| Action | Shortcut |
|--------|----------|
| **Group All in Tab** | `Super + T` |
| **Group All in Window** | `Super + G` |
| **Ungroup All** | `Super + Shift + G` |
| **Broadcast Off** | `Alt + A` (Broadcast to None) |
| **Broadcast Group** | `Alt + G` (Broadcast to Group) |
| **Broadcast All** | `Alt + O` (Broadcast to All) |

*(Note: `Super` is usually the Windows/Command key)*

## Recommended Bash Aliases

Add these to your `~/.bashrc` to speed up your workflow.

```bash
# --- Terminator Power Aliases ---

# 1. Quick Launch
alias term='terminator &'

# 2. Open Terminator in current directory (detached)
alias termhere='terminator -u -w "$PWD" &'

# 3. Layout Shortcuts (Customize these names!)
alias term-ros='terminator -l ros_layout &'
alias term-monitor='terminator -l monitor_layout &'


# 4. Configuration Editing
alias edit-term='nano ~/.config/terminator/config'

# 5. Kill all terminator instances (Use with caution)
alias kill-term='pkill -f /usr/bin/terminator'
```
