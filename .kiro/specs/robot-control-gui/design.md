# Design Document

## Overview

The Robot Control GUI is a desktop application built with Python and PyQt6 that provides a unified interface for managing ROS2 robot operations. The application follows a modular architecture with clear separation between UI, business logic, and system interaction layers. It supports both local operations (simulation, firmware builds) and remote operations (SSH-based robot control) through a consistent interface.

The application will be designed as a single-window GUI with a tabbed interface, where each tab represents a major functional area (Dashboard, Teleoperation, Robot Control, Simulation, Firmware, Monitoring). Process management will be handled through Python's subprocess module with real-time output streaming to the GUI.

NEVER MAKE CODE FILES WITH MORE THEN 300 LINES OF CODE
## Path

'/home/marco/ros2_steel_ws/my_steel-robot_ws/robot_control_gui'

## Architecture

### High-Level Architecture

```txt
┌─────────────────────────────────────────────────────────────┐
│                     PyQt6 GUI Layer                         │
│  ┌──────────┬──────────┬──────────┬──────────┬──────────┐  │
│  │Dashboard │ Teleop   │  Robot   │   Sim    │ Firmware │  │
│  │  Tab     │   Tab    │   Tab    │   Tab    │   Tab    │  │
│  └──────────┴──────────┴──────────┴──────────┴──────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │            Monitoring Tab (Topics/Nodes)             │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                   Business Logic Layer                      │
│  ┌──────────────┬──────────────┬──────────────────────┐    │
│  │   Process    │   Config     │    ROS2 Interface    │    │
│  │   Manager    │   Manager    │                      │    │
│  └──────────────┴──────────────┴──────────────────────┘    │
│  ┌──────────────┬──────────────┬──────────────────────┐    │
│  │     SSH      │   Firmware   │    Log Manager       │    │
│  │   Manager    │   Builder    │                      │    │
│  └──────────────┴──────────────┴──────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                   System Interface Layer                    │
│  ┌──────────────┬──────────────┬──────────────────────┐    │
│  │  subprocess  │   paramiko   │    ROS2 CLI/API      │    │
│  │              │   (SSH)      │                      │    │
│  └──────────────┴──────────────┴──────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

### Technology Stack

- **GUI Framework**: PyQt6 (modern, cross-platform, rich widget set)
- **Process Management**: Python `subprocess` module with `Popen` for async execution
- **SSH Communication**: `paramiko` library for remote command execution
- **ROS2 Integration**: `rclpy` for ROS2 API access, subprocess for CLI commands
- **Configuration**: JSON-based config file stored in `~/.config/robot_control_gui/config.json`
- **Logging**: Python `logging` module with file and console handlers

### Design Rationale

1. **PyQt6 over alternatives**: Provides native look-and-feel, excellent documentation, and mature ecosystem
2. **Tabbed interface**: Organizes functionality logically while keeping all features accessible
3. **Subprocess-based execution**: Allows real-time output capture and process control without blocking the GUI
4. **Paramiko for SSH**: Industry-standard library with robust connection handling and authentication
5. **JSON configuration**: Human-readable, easy to edit manually if needed

## Components and Interfaces

### 1. Main Window (MainWindow)

**Responsibilities:**
- Host the tabbed interface
- Manage global application state
- Coordinate between tabs
- Handle menu bar and status bar

**Key Methods:**
```python
class MainWindow(QMainWindow):
    def __init__(self)
    def setup_ui(self)
    def create_menu_bar(self)
    def update_status_bar(self, message: str)
    def load_config(self)
    def save_config(self)
    def closeEvent(self, event)  # Cleanup on exit
```

### 2. Dashboard Tab (DashboardWidget)

**Responsibilities:**
- Display overall system status
- Show quick-start buttons for common operations
- Display connection status (robot, ROS2, micro-ROS)
- Show recent logs and alerts

**Key Components:**
- Status indicators (QLabel with colored icons)
- Quick action buttons (QPushButton)
- Recent activity log (QTextEdit, read-only)
- Connection status panel

**Signals:**
```python
start_teleoperation = pyqtSignal()
start_robot = pyqtSignal()
start_simulation = pyqtSignal()
start_microros = pyqtSignal()
```

### 3. Teleoperation Tab (TeleopWidget)

**Responsibilities:**
- Start/stop teleoperation nodes
- Display joystick connection status
- Show controller button mappings
- Display teleoperation logs

**Key Components:**
- Start/Stop buttons (QPushButton)
- Joystick status indicator (QLabel)
- Controller mapping display (QGroupBox with QLabel grid)
- Log viewer (QTextEdit)
- Process status panel (PID, uptime)

**Interface with ProcessManager:**
```python
def start_teleoperation(self):
    self.process_manager.start_process(
        name="teleoperation",
        commands=[
            ["ros2", "run", "joy", "joy_node"],
            ["ros2", "run", "teleop_twist_joy", "teleop_node", 
             "--ros-args", "--params-file", config_path]
        ],
        callback=self.on_output_received
    )

def stop_teleoperation(self):
    self.process_manager.stop_process("teleoperation")
```

### 4. Robot Control Tab (RobotWidget)

**Responsibilities:**
- Start/stop robot system remotely via SSH
- Display robot component status (micro-ROS, bringup, Foxglove)
- Show Foxglove connection URL
- Display robot logs in real-time

**Key Components:**
- Connection settings (QLineEdit for hostname, port, username)
- Start/Stop Robot buttons (QPushButton)
- Component status indicators (QLabel with icons)
- Foxglove URL display (QLabel with copy button)
- Log viewer with tabs for each component (QTabWidget with QTextEdit)
- Test Connection button

**Interface with SSHManager:**
```python
def start_robot(self):
    hostname = self.hostname_input.text()
    username = self.username_input.text()
    
    self.ssh_manager.connect(hostname, username, key_path)
    self.ssh_manager.execute_script(
        script_path="scripts/start_robot.sh",
        callback=self.on_robot_output
    )

def stop_robot(self):
    self.ssh_manager.execute_command(
        "pkill -f 'micro_ros_agent|robot_bringup|foxglove_bridge'"
    )
```

### 5. Simulation Tab (SimulationWidget)

**Responsibilities:**
- Start/stop Gazebo simulation
- Launch RViz and other visualization tools
- Display simulation status
- Show simulation logs

**Key Components:**
- Start/Stop Simulation buttons (QPushButton)
- World selection dropdown (QComboBox)
- Launch RViz button (QPushButton)
- Tmux mode checkbox (QCheckBox)
- Simulation status panel
- Log viewer (QTextEdit)

**Interface with ProcessManager:**
```python
def start_simulation(self):
    world = self.world_selector.currentText()
    use_tmux = self.tmux_checkbox.isChecked()
    
    if use_tmux:
        self.process_manager.start_process(
            name="simulation",
            commands=[["bash", "scripts/start_sim_tmux.sh"]],
            callback=self.on_output_received
        )
    else:
        self.process_manager.start_process(
            name="simulation",
            commands=[[
                "ros2", "launch", "robot", "launch_sim.launch.py",
                f"world:={world}", "use_sim_time:=true"
            ]],
            callback=self.on_output_received
        )
```

### 6. Firmware Tab (FirmwareWidget)

**Responsibilities:**
- Build firmware (debug/release)
- Flash firmware to Pico
- Monitor firmware output
- Display build logs

**Key Components:**
- Build mode selector (QComboBox: Debug/Release)
- Build Firmware button (QPushButton)
- Flash Firmware button (QPushButton)
- Monitor Firmware button (QPushButton)
- Build progress bar (QProgressBar)
- Build/flash log viewer (QTextEdit)

**Interface with FirmwareBuilder:**
```python
def build_firmware(self):
    mode = self.build_mode_selector.currentText()  # "Debug" or "Release"
    
    self.firmware_builder.build(
        mode=mode.lower(),
        callback=self.on_build_output,
        completion_callback=self.on_build_complete
    )

def flash_firmware(self):
    mode = self.build_mode_selector.currentText().lower()
    
    self.firmware_builder.flash(
        mode=mode,
        callback=self.on_flash_output,
        completion_callback=self.on_flash_complete
    )
```

### 7. Monitoring Tab (MonitoringWidget)

**Responsibilities:**
- List active ROS2 nodes
- List active ROS2 topics
- Display topic messages in real-time
- Show node details

**Key Components:**
- Node list (QListWidget)
- Topic list (QListWidget)
- Topic message viewer (QTextEdit)
- Refresh button (QPushButton)
- Topic echo controls (Start/Stop echo)

**Interface with ROS2Interface:**
```python
def refresh_nodes(self):
    nodes = self.ros2_interface.get_node_list()
    self.node_list.clear()
    self.node_list.addItems(nodes)

def refresh_topics(self):
    topics = self.ros2_interface.get_topic_list()
    self.topic_list.clear()
    for topic, msg_type in topics:
        self.topic_list.addItem(f"{topic} [{msg_type}]")

def echo_topic(self, topic_name: str):
    self.ros2_interface.echo_topic(
        topic_name,
        callback=self.on_topic_message
    )
```

### 8. Process Manager (ProcessManager)

**Responsibilities:**
- Start and stop local processes
- Capture stdout/stderr in real-time
- Track process status (PID, uptime, exit code)
- Handle process cleanup

**Key Methods:**
```python
class ProcessManager:
    def __init__(self):
        self.processes: Dict[str, ProcessInfo] = {}
    
    def start_process(self, name: str, commands: List[List[str]], 
                     callback: Callable[[str, str], None]) -> bool:
        """Start one or more processes and stream output"""
        
    def stop_process(self, name: str) -> bool:
        """Stop a process and all its children"""
        
    def get_process_status(self, name: str) -> ProcessInfo:
        """Get current status of a process"""
        
    def is_running(self, name: str) -> bool:
        """Check if a process is running"""
        
    def cleanup_all(self):
        """Stop all managed processes"""
```

**ProcessInfo Data Class:**
```python
@dataclass
class ProcessInfo:
    name: str
    pid: int
    start_time: float
    status: str  # "running", "stopped", "error"
    exit_code: Optional[int]
    processes: List[subprocess.Popen]
```

### 9. SSH Manager (SSHManager)

**Responsibilities:**
- Establish SSH connections to remote robot
- Execute commands and scripts remotely
- Stream remote output to GUI
- Handle authentication (password, key-based)

**Key Methods:**
```python
class SSHManager:
    def __init__(self):
        self.client: Optional[paramiko.SSHClient] = None
        self.connected: bool = False
    
    def connect(self, hostname: str, username: str, 
                password: Optional[str] = None,
                key_path: Optional[str] = None) -> bool:
        """Establish SSH connection"""
        
    def disconnect(self):
        """Close SSH connection"""
        
    def execute_command(self, command: str, 
                       callback: Callable[[str], None]) -> Tuple[int, str, str]:
        """Execute a single command and stream output"""
        
    def execute_script(self, script_path: str,
                      callback: Callable[[str], None]) -> int:
        """Execute a script file remotely"""
        
    def test_connection(self) -> bool:
        """Test if connection is alive"""
```

### 10. ROS2 Interface (ROS2Interface)

**Responsibilities:**
- Query ROS2 system (nodes, topics, services)
- Echo topic messages
- Publish test messages
- Check ROS2 environment availability

**Key Methods:**
```python
class ROS2Interface:
    def __init__(self):
        self.node: Optional[rclpy.node.Node] = None
        self.executor: Optional[rclpy.executors.Executor] = None
    
    def initialize(self) -> bool:
        """Initialize ROS2 context"""
        
    def get_node_list(self) -> List[str]:
        """Get list of active nodes"""
        
    def get_topic_list(self) -> List[Tuple[str, str]]:
        """Get list of topics with message types"""
        
    def echo_topic(self, topic_name: str, msg_type: str,
                   callback: Callable[[str], None]):
        """Subscribe to topic and stream messages"""
        
    def stop_echo(self, topic_name: str):
        """Stop echoing a topic"""
        
    def is_ros2_available(self) -> bool:
        """Check if ROS2 environment is sourced"""
```

### 11. Firmware Builder (FirmwareBuilder)

**Responsibilities:**
- Build firmware using CMake/Make
- Flash firmware to Pico
- Detect Pico in BOOTSEL mode
- Monitor build progress

**Key Methods:**
```python
class FirmwareBuilder:
    def __init__(self, workspace_path: str):
        self.workspace_path = workspace_path
        self.firmware_path = os.path.join(workspace_path, "firmware")
    
    def build(self, mode: str, callback: Callable[[str], None],
             completion_callback: Callable[[bool, str], None]):
        """Build firmware in debug or release mode"""
        
    def flash(self, mode: str, callback: Callable[[str], None],
             completion_callback: Callable[[bool, str], None]):
        """Flash firmware to Pico"""
        
    def detect_pico_bootsel(self) -> Optional[str]:
        """Detect Pico in BOOTSEL mode"""
        
    def get_firmware_path(self, mode: str) -> str:
        """Get path to built firmware .uf2 file"""
```

### 12. Configuration Manager (ConfigManager)

**Responsibilities:**
- Load and save application configuration
- Provide default values
- Validate configuration
- Handle config file migration

**Configuration Schema:**
```python
{
    "robot": {
        "hostname": "robot.local",
        "username": "ubuntu",
        "ssh_key_path": "~/.ssh/id_rsa",
        "ros_domain_id": 0
    },
    "teleoperation": {
        "config_file": "config/xbox_teleop.yaml",
        "auto_detect_joystick": true
    },
    "simulation": {
        "default_world": "obstacles.world",
        "use_tmux": false
    },
    "firmware": {
        "default_build_mode": "debug",
        "auto_flash_after_build": false
    },
    "ui": {
        "theme": "system",
        "log_max_lines": 1000,
        "auto_scroll_logs": true
    },
    "paths": {
        "workspace": "/path/to/workspace",
        "pico_sdk": "/path/to/pico-sdk"
    }
}
```

**Key Methods:**
```python
class ConfigManager:
    def __init__(self, config_path: Optional[str] = None):
        self.config_path = config_path or self._get_default_config_path()
        self.config: Dict = {}
    
    def load(self) -> Dict:
        """Load configuration from file"""
        
    def save(self, config: Dict):
        """Save configuration to file"""
        
    def get(self, key_path: str, default: Any = None) -> Any:
        """Get config value using dot notation (e.g., 'robot.hostname')"""
        
    def set(self, key_path: str, value: Any):
        """Set config value using dot notation"""
        
    def validate(self) -> List[str]:
        """Validate configuration and return list of errors"""
```

### 13. Log Manager (LogManager)

**Responsibilities:**
- Manage log files for each process
- Rotate logs based on size/age
- Provide log search functionality
- Export logs

**Key Methods:**
```python
class LogManager:
    def __init__(self, log_dir: str):
        self.log_dir = log_dir
        self.log_files: Dict[str, str] = {}
    
    def create_log_file(self, process_name: str) -> str:
        """Create timestamped log file for process"""
        
    def write_log(self, process_name: str, message: str):
        """Write message to process log file"""
        
    def get_log_path(self, process_name: str) -> Optional[str]:
        """Get path to current log file for process"""
        
    def rotate_logs(self, max_size_mb: int = 10):
        """Rotate logs that exceed size limit"""
        
    def search_logs(self, process_name: str, query: str) -> List[str]:
        """Search log file for matching lines"""
```

## Data Models

### ProcessInfo

```python
@dataclass
class ProcessInfo:
    name: str
    pid: int
    start_time: float
    status: str  # "running", "stopped", "error"
    exit_code: Optional[int]
    processes: List[subprocess.Popen]
    
    @property
    def uptime(self) -> float:
        """Calculate uptime in seconds"""
        return time.time() - self.start_time if self.status == "running" else 0
    
    @property
    def uptime_str(self) -> str:
        """Format uptime as human-readable string"""
        uptime = self.uptime
        hours = int(uptime // 3600)
        minutes = int((uptime % 3600) // 60)
        seconds = int(uptime % 60)
        return f"{hours:02d}:{minutes:02d}:{seconds:02d}"
```

### RobotStatus

```python
@dataclass
class RobotStatus:
    connected: bool
    microros_running: bool
    bringup_running: bool
    foxglove_running: bool
    foxglove_url: Optional[str]
    last_update: float
```

### TopicInfo

```python
@dataclass
class TopicInfo:
    name: str
    msg_type: str
    publishers: int
    subscribers: int
```

### NodeInfo

```python
@dataclass
class NodeInfo:
    name: str
    namespace: str
    publishers: List[str]
    subscribers: List[str]
    services: List[str]
```

## Error Handling

### Error Categories

1. **Connection Errors**: SSH connection failures, ROS2 environment not found
2. **Process Errors**: Process launch failures, unexpected termination
3. **Configuration Errors**: Invalid config values, missing required settings
4. **Hardware Errors**: Joystick not found, Pico not detected
5. **Build Errors**: Firmware build failures, missing dependencies

### Error Handling Strategy

- **User-Facing Errors**: Display in QMessageBox with clear message and suggested actions
- **Background Errors**: Log to file and show notification in status bar
- **Critical Errors**: Show modal dialog and prevent further actions until resolved
- **Recoverable Errors**: Offer retry button and automatic retry with exponential backoff

### Example Error Messages

```python
ERROR_MESSAGES = {
    "ssh_connection_failed": {
        "title": "SSH Connection Failed",
        "message": "Could not connect to robot at {hostname}",
        "suggestions": [
            "Check that the robot is powered on",
            "Verify the hostname/IP address",
            "Ensure SSH is enabled on the robot",
            "Check network connectivity"
        ]
    },
    "joystick_not_found": {
        "title": "Joystick Not Found",
        "message": "No joystick detected at /dev/input/js*",
        "suggestions": [
            "Connect your Xbox controller via USB",
            "Check USB cable connection",
            "Try a different USB port"
        ]
    },
    "ros2_not_available": {
        "title": "ROS2 Not Available",
        "message": "ROS2 environment not found",
        "suggestions": [
            "Source ROS2: source /opt/ros/humble/setup.bash",
            "Check ROS2 installation",
            "Restart the application after sourcing ROS2"
        ]
    }
}
```

## Testing Strategy

### Unit Tests

- **ProcessManager**: Test process start/stop, output capture, cleanup
- **SSHManager**: Test connection, command execution (using mock SSH server)
- **ConfigManager**: Test load/save, validation, default values
- **ROS2Interface**: Test node/topic listing (using mock ROS2 environment)
- **FirmwareBuilder**: Test build command generation, Pico detection

### Integration Tests

- **Teleoperation Flow**: Start joy + teleop nodes, verify output, stop cleanly
- **Robot Control Flow**: SSH connection, remote script execution, log streaming
- **Simulation Flow**: Launch Gazebo, verify nodes, stop simulation
- **Firmware Flow**: Build firmware, verify .uf2 file created

### UI Tests

- **Widget Interaction**: Button clicks, input validation, tab switching
- **Log Display**: Output streaming, auto-scroll, log clearing
- **Status Updates**: Real-time status indicator updates
- **Configuration**: Settings dialog, save/load, validation

### Manual Testing Checklist

1. Start teleoperation with joystick connected
2. Start teleoperation without joystick (verify error)
3. Start robot remotely via SSH
4. Start simulation in normal and tmux modes
5. Build and flash firmware
6. Monitor ROS2 topics and nodes
7. Test connection to robot
8. Save and load configuration
9. Verify log file creation and rotation
10. Test graceful shutdown with processes running

## UI/UX Design

### Color Scheme

- **Status Indicators**:
  - Green (#4CAF50): Running/Connected
  - Red (#F44336): Error/Stopped
  - Gray (#9E9E9E): Inactive/Unknown
  - Yellow (#FFC107): Warning/Connecting

- **Theme**: Follow system theme (light/dark mode support)

### Layout Principles

1. **Consistent spacing**: 10px padding, 5px margins
2. **Logical grouping**: Related controls in QGroupBox
3. **Clear hierarchy**: Important actions prominent, secondary actions smaller
4. **Responsive design**: Minimum window size 1024x768, resizable

### Accessibility

- **Keyboard shortcuts**: Ctrl+T (teleoperation), Ctrl+R (robot), Ctrl+S (simulation)
- **Tab order**: Logical tab navigation through controls
- **Screen reader support**: Proper labels and descriptions
- **High contrast**: Support for high contrast themes

### Wireframe (Dashboard Tab)

```
┌─────────────────────────────────────────────────────────────┐
│ File  Edit  View  Tools  Help                               │
├─────────────────────────────────────────────────────────────┤
│ [Dashboard] [Teleop] [Robot] [Simulation] [Firmware] [Mon.] │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  System Status                                                │
│  ┌──────────────┬──────────────┬──────────────┬────────────┐│
│  │ ● Teleoperation │ ● Robot System │ ● Simulation │ ● micro-ROS││
│  │   Stopped      │   Connected   │   Stopped    │   Running  ││
│  └──────────────┴──────────────┴──────────────┴────────────┘│
│                                                               │
│  Quick Actions                                                │
│  ┌──────────────────────────────────────────────────────────┐│
│  │ [Start Teleoperation]  [Start Robot]  [Start Simulation] ││
│  │ [Start micro-ROS Agent]  [Build Firmware]  [Open RViz]   ││
│  └──────────────────────────────────────────────────────────┘│
│                                                               │
│  Recent Activity                                              │
│  ┌──────────────────────────────────────────────────────────┐│
│  │ 14:32:15 - Teleoperation started                         ││
│  │ 14:30:42 - Connected to robot at robot.local             ││
│  │ 14:28:10 - Firmware build completed successfully         ││
│  │ 14:25:33 - micro-ROS agent started on /dev/ttyACM0       ││
│  │                                                           ││
│  └──────────────────────────────────────────────────────────┘│
│                                                               │
├─────────────────────────────────────────────────────────────┤
│ Status: Ready                                    14:35:22    │
└─────────────────────────────────────────────────────────────┘
```

## Deployment

### Installation

1. **Dependencies**:
   ```bash
   sudo apt install python3-pyqt6 python3-paramiko python3-rclpy
   pip install pyqt6 paramiko
   ```

2. **Application Installation**:
   ```bash
   cd scripts/robot_control_gui
   pip install -e .
   ```

3. **Desktop Entry** (optional):
   Create `~/.local/share/applications/robot-control-gui.desktop`

### Packaging

- **Python Package**: Setup with `setup.py` for pip installation
- **AppImage**: Bundle with PyInstaller for standalone executable
- **Debian Package**: Create .deb for easy installation on Ubuntu

### Configuration

- Default config location: `~/.config/robot_control_gui/config.json`
- Logs location: `~/.local/share/robot_control_gui/logs/`
- First-run wizard to configure robot connection settings

## Security Considerations

1. **SSH Credentials**: Store SSH key path, not password; use SSH agent when possible
2. **Configuration File**: Set permissions to 600 (user read/write only)
3. **Process Isolation**: Run processes with user privileges, not root
4. **Input Validation**: Sanitize all user inputs before passing to subprocess/SSH
5. **Network Security**: Warn user when connecting to untrusted hosts

## Performance Considerations

1. **Output Buffering**: Limit log viewer to last 1000 lines, write full logs to file
2. **Process Monitoring**: Poll process status every 1 second, not continuously
3. **ROS2 Topic Echo**: Throttle message display to 10 Hz to prevent GUI freezing
4. **SSH Connection**: Reuse connection for multiple commands, implement connection pooling
5. **UI Updates**: Use Qt signals/slots for thread-safe updates from background threads

## Future Enhancements

1. **Multi-Robot Support**: Manage multiple robots from single GUI
2. **Custom Launch Configurations**: Save and load custom launch profiles
3. **Diagnostic Tools**: Built-in network diagnostics, ROS2 graph visualization
4. **Recording/Playback**: Record and replay ROS2 bag files
5. **Plugin System**: Allow custom tabs/widgets for project-specific functionality
6. **Web Interface**: Optional web-based UI for remote access
7. **Notification System**: Desktop notifications for important events
8. **Auto-Discovery**: Automatically discover robots on local network
