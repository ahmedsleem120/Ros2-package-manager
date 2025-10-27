#!/usr/bin/env python3
"""
ROS2 GUI Launcher - Advanced Full-Featured Version (Fixed)
A comprehensive GUI tool for managing ROS2 launch files, nodes, and scripts
with monitoring, visualization, and configuration management
"""

import sys
import os
import subprocess
import threading
import queue
import json
import yaml
import psutil
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import List, Dict, Optional, Any
from enum import Enum
from datetime import datetime
import signal
import re

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTreeWidget, QTreeWidgetItem, QPushButton, QTextEdit, QSplitter,
    QLabel, QFileDialog, QMessageBox, QStatusBar, QToolBar, QAction,
    QTabWidget, QLineEdit, QComboBox, QDialog, QFormLayout, QSpinBox,
    QCheckBox, QDialogButtonBox, QTableWidget, QTableWidgetItem, QMenu,
    QGroupBox, QProgressBar, QScrollArea, QListWidget, QListWidgetItem,
    QInputDialog, QTreeWidgetItemIterator, QAbstractItemView
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QSize
from PyQt5.QtGui import QIcon, QFont, QColor, QPainter, QPen


class ItemType(Enum):
    """Types of items in the project tree"""
    WORKSPACE = "workspace"
    PACKAGE = "package"
    LAUNCH_FILE = "launch_file"
    PYTHON_NODE = "python_node"
    CPP_NODE = "cpp_node"
    SCRIPT = "script"


@dataclass
class ROS2Item:
    """Represents a ROS2 project item"""
    name: str
    path: Path
    item_type: ItemType
    package_name: Optional[str] = None
    parameters: Optional[Dict[str, Any]] = None
    
    def get_command(self, params: Optional[Dict[str, Any]] = None) -> Optional[List[str]]:
        """Get the command to execute this item"""
        if self.item_type == ItemType.LAUNCH_FILE:
            cmd = ["ros2", "launch", self.package_name, self.path.name]
            if params:
                for key, value in params.items():
                    cmd.append(f"{key}:={value}")
            return cmd
        elif self.item_type == ItemType.PYTHON_NODE:
            return ["ros2", "run", self.package_name, self.path.stem]
        elif self.item_type == ItemType.SCRIPT:
            if self.path.suffix == ".py":
                return ["python3", str(self.path)]
            else:
                return ["bash", str(self.path)]
        return None


@dataclass
class ProcessInfo:
    """Information about a running process"""
    name: str
    pid: int
    start_time: datetime
    command: List[str]
    cpu_percent: float = 0.0
    memory_mb: float = 0.0
    status: str = "running"


class ConfigManager:
    """Manages user configurations and favorites"""
    
    def __init__(self, config_path: Path):
        self.config_path = config_path
        self.config_path.mkdir(parents=True, exist_ok=True)
        self.favorites_file = self.config_path / "favorites.json"
        self.recent_file = self.config_path / "recent_workspaces.json"
        self.launch_configs_file = self.config_path / "launch_configs.json"
        self.settings_file = self.config_path / "settings.json"
        self.session_file = self.config_path / "session.json"
        
    def load_favorites(self) -> List[Dict]:
        """Load favorite launch configurations"""
        if self.favorites_file.exists():
            with open(self.favorites_file, 'r') as f:
                return json.load(f)
        return []
    
    def save_favorite(self, item: ROS2Item, params: Optional[Dict] = None, alias: str = ""):
        """Save a favorite launch configuration"""
        favorites = self.load_favorites()
        favorite = {
            "alias": alias or item.name,
            "name": item.name,
            "package": item.package_name,
            "path": str(item.path),
            "type": item.item_type.value,
            "parameters": params or {},
            "timestamp": datetime.now().isoformat()
        }
        favorites.append(favorite)
        with open(self.favorites_file, 'w') as f:
            json.dump(favorites, f, indent=2)
    
    def remove_favorite(self, index: int):
        """Remove a favorite"""
        favorites = self.load_favorites()
        if 0 <= index < len(favorites):
            favorites.pop(index)
            with open(self.favorites_file, 'w') as f:
                json.dump(favorites, f, indent=2)
    
    def load_recent_workspaces(self) -> List[str]:
        """Load recently opened workspaces"""
        if self.recent_file.exists():
            with open(self.recent_file, 'r') as f:
                return json.load(f)
        return []
    
    def add_recent_workspace(self, workspace_path: str):
        """Add workspace to recent list"""
        recent = self.load_recent_workspaces()
        if workspace_path in recent:
            recent.remove(workspace_path)
        recent.insert(0, workspace_path)
        recent = recent[:10]  # Keep last 10
        with open(self.recent_file, 'w') as f:
            json.dump(recent, f, indent=2)
    
    def save_launch_config(self, name: str, params: Dict):
        """Save launch file parameter configuration"""
        configs = {}
        if self.launch_configs_file.exists():
            with open(self.launch_configs_file, 'r') as f:
                configs = json.load(f)
        configs[name] = params
        with open(self.launch_configs_file, 'w') as f:
            json.dump(configs, f, indent=2)
    
    def load_launch_config(self, name: str) -> Optional[Dict]:
        """Load launch file parameter configuration"""
        if self.launch_configs_file.exists():
            with open(self.launch_configs_file, 'r') as f:
                configs = json.load(f)
                return configs.get(name)
        return None

    # Settings management
    def load_setting(self, key: str, default: Any = None) -> Any:
        if self.settings_file.exists():
            try:
                with open(self.settings_file, 'r') as f:
                    data = json.load(f)
                    return data.get(key, default)
            except Exception:
                return default
        return default

    def save_setting(self, key: str, value: Any):
        data = {}
        if self.settings_file.exists():
            try:
                with open(self.settings_file, 'r') as f:
                    data = json.load(f)
            except Exception:
                data = {}
        data[key] = value
        with open(self.settings_file, 'w') as f:
            json.dump(data, f, indent=2)

    # Session management (save/restore open tabs and logs)
    def save_session(self, session_data: List[Dict[str, Any]]):
        try:
            with open(self.session_file, 'w') as f:
                json.dump(session_data, f, indent=2)
        except Exception:
            pass

    def load_session(self) -> List[Dict[str, Any]]:
        if self.session_file.exists():
            try:
                with open(self.session_file, 'r') as f:
                    return json.load(f)
            except Exception:
                return []
        return []


class ProcessManager:
    """Manages running ROS2 processes with monitoring"""
    
    def __init__(self):
        self.processes: Dict[str, subprocess.Popen] = {}
        self.process_info: Dict[str, ProcessInfo] = {}
        self.output_queues: Dict[str, queue.Queue] = {}
        self.log_files: Dict[str, Path] = {}
        self.stop_flags: Dict[str, bool] = {}
    
    def start_process(self, name: str, command: List[str], workspace_path: Path, 
                     log_dir: Optional[Path] = None) -> bool:
        """Start a new process"""
        if name in self.processes and self.processes[name].poll() is None:
            return False
        
        try:
            setup_file = workspace_path / "install" / "setup.bash"
            if setup_file.exists():
                full_command = f"source {setup_file} && {' '.join(command)}"
            else:
                full_command = ' '.join(command)
            
            process = subprocess.Popen(
                full_command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                stdin=subprocess.PIPE,
                executable='/bin/bash',
                preexec_fn=os.setsid,
                text=True,
                bufsize=1
            )
            
            self.processes[name] = process
            self.stop_flags[name] = False
            self.process_info[name] = ProcessInfo(
                name=name,
                pid=process.pid,
                start_time=datetime.now(),
                command=command
            )
            self.output_queues[name] = queue.Queue()
            
            # Setup log file
            if log_dir:
                log_dir.mkdir(parents=True, exist_ok=True)
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                log_file = log_dir / f"{name.replace('/', '_')}_{timestamp}.log"
                self.log_files[name] = log_file
            
            thread = threading.Thread(
                target=self._read_output,
                args=(name, process),
                daemon=True
            )
            thread.start()
            
            return True
        except Exception as e:
            print(f"Error starting process: {e}")
            return False
    
    def _read_output(self, name: str, process: subprocess.Popen):
        """Read process output in a separate thread"""
        try:
            for line in process.stdout:
                if self.stop_flags.get(name, False):
                    break
                if name in self.output_queues:
                    self.output_queues[name].put(line)
                    
                    # Write to log file
                    if name in self.log_files:
                        try:
                            with open(self.log_files[name], 'a') as f:
                                f.write(line)
                        except:
                            pass
        except Exception as e:
            if name in self.output_queues and not self.stop_flags.get(name, False):
                self.output_queues[name].put(f"Error reading output: {e}\n")
    
    def stop_process(self, name: str, preferred_first_signal: str = "SIGINT") -> bool:
        """Stop a running process"""
        if name not in self.processes:
            return False
        
        process = self.processes[name]
        if process.poll() is not None:
            return False
        
        self.stop_flags[name] = True
        
        try:
            # Get the process group ID
            pgid = os.getpgid(process.pid)
            # Determine first preferred signal
            first = signal.SIGINT if preferred_first_signal == "SIGINT" else signal.SIGTERM
            second = signal.SIGTERM if first == signal.SIGINT else signal.SIGINT

            # Try preferred first (SIGINT or SIGTERM)
            try:
                os.killpg(pgid, first)
            except Exception:
                pass

            # Wait a short time for graceful shutdown
            try:
                process.wait(timeout=3)
                return True
            except subprocess.TimeoutExpired:
                # If still running, send the other signal
                try:
                    os.killpg(pgid, second)
                except Exception:
                    pass

                try:
                    process.wait(timeout=3)
                    return True
                except subprocess.TimeoutExpired:
                    # Force kill if still running
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                    except Exception:
                        pass
                    try:
                        process.wait(timeout=2)
                    except Exception:
                        pass
                
        except ProcessLookupError:
            # Process already terminated
            pass
        except Exception as e:
            print(f"Error stopping process {name}: {e}")
            try:
                # Last resort - kill the process directly
                process.kill()
                process.wait()
            except:
                pass
        
        if name in self.process_info:
            self.process_info[name].status = "stopped"
        
        return True
    
    def is_running(self, name: str) -> bool:
        """Check if a process is running"""
        if name not in self.processes:
            return False
        return self.processes[name].poll() is None
    
    def get_output(self, name: str) -> List[str]:
        """Get available output lines"""
        if name not in self.output_queues:
            return []
        
        lines = []
        try:
            while True:
                lines.append(self.output_queues[name].get_nowait())
        except queue.Empty:
            pass
        return lines
    
    def update_resource_usage(self):
        """Update CPU and memory usage for all processes"""
        for name, process in list(self.processes.items()):
            if process.poll() is None and name in self.process_info:
                try:
                    p = psutil.Process(process.pid)
                    self.process_info[name].cpu_percent = p.cpu_percent()
                    self.process_info[name].memory_mb = p.memory_info().rss / 1024 / 1024
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
    
    def get_process_info(self, name: str) -> Optional[ProcessInfo]:
        """Get process information"""
        return self.process_info.get(name)
    
    def export_log(self, name: str, export_path: Path) -> bool:
        """Export log file"""
        if name in self.log_files and self.log_files[name].exists():
            import shutil
            shutil.copy(self.log_files[name], export_path)
            return True
        return False


class BuildDialog(QDialog):
    """Dialog for colcon build options"""
    
    def __init__(self, workspace_path: Path, parent=None):
        super().__init__(parent)
        self.workspace_path = workspace_path
        self.setup_ui()
    
    def setup_ui(self):
        self.setWindowTitle("Build Workspace")
        self.setMinimumWidth(500)
        
        layout = QVBoxLayout(self)
        
        layout.addWidget(QLabel(f"Workspace: {self.workspace_path}"))
        
        # Build options
        options_group = QGroupBox("Build Options")
        options_layout = QVBoxLayout()
        
        self.symlink_install = QCheckBox("Symlink install")
        self.symlink_install.setChecked(True)
        options_layout.addWidget(self.symlink_install)
        
        self.cmake_args = QLineEdit()
        self.cmake_args.setPlaceholderText("Additional cmake args (optional)")
        options_layout.addWidget(QLabel("CMake Args:"))
        options_layout.addWidget(self.cmake_args)
        
        # Parallel jobs
        parallel_layout = QHBoxLayout()
        parallel_layout.addWidget(QLabel("Parallel jobs:"))
        self.parallel_jobs = QSpinBox()
        self.parallel_jobs.setMinimum(1)
        self.parallel_jobs.setMaximum(32)
        self.parallel_jobs.setValue(4)
        parallel_layout.addWidget(self.parallel_jobs)
        parallel_layout.addStretch()
        options_layout.addLayout(parallel_layout)
        
        # Packages to build
        self.packages_input = QLineEdit()
        self.packages_input.setPlaceholderText("Leave empty to build all packages")
        options_layout.addWidget(QLabel("Specific packages (space-separated):"))
        options_layout.addWidget(self.packages_input)
        
        options_group.setLayout(options_layout)
        layout.addWidget(options_group)
        
        # Buttons
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)
    
    def get_build_command(self) -> List[str]:
        """Get the colcon build command"""
        cmd = ["colcon", "build"]
        
        if self.symlink_install.isChecked():
            cmd.append("--symlink-install")
        
        cmd.extend(["--parallel-workers", str(self.parallel_jobs.value())])
        
        if self.cmake_args.text().strip():
            cmd.extend(["--cmake-args", self.cmake_args.text().strip()])
        
        packages = self.packages_input.text().strip()
        if packages:
            cmd.extend(["--packages-select"] + packages.split())
        
        return cmd


class ROS2Inspector:
    """Inspects ROS2 system state - nodes, topics, services"""
    
    @staticmethod
    def get_active_nodes() -> List[str]:
        """Get list of active ROS2 nodes"""
        try:
            result = subprocess.run(
                ["ros2", "node", "list"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                return [line.strip() for line in result.stdout.split('\n') if line.strip()]
        except Exception as e:
            print(f"Error getting nodes: {e}")
        return []
    
    @staticmethod
    def get_topics() -> List[Dict[str, str]]:
        """Get list of active topics with types"""
        try:
            result = subprocess.run(
                ["ros2", "topic", "list", "-t"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                topics = []
                for line in result.stdout.split('\n'):
                    if line.strip():
                        parts = line.strip().split()
                        if len(parts) >= 2:
                            topics.append({"name": parts[0], "type": parts[1]})
                return topics
        except Exception as e:
            print(f"Error getting topics: {e}")
        return []
    
    @staticmethod
    def get_services() -> List[str]:
        """Get list of active services"""
        try:
            result = subprocess.run(
                ["ros2", "service", "list"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                return [line.strip() for line in result.stdout.split('\n') if line.strip()]
        except Exception as e:
            print(f"Error getting services: {e}")
        return []
    
    @staticmethod
    def get_node_info(node_name: str) -> Dict[str, Any]:
        """Get detailed information about a node"""
        try:
            result = subprocess.run(
                ["ros2", "node", "info", node_name],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                return {"info": result.stdout}
        except Exception:
            pass
        return {}


class WorkspaceScanner:
    """Scans ROS2 workspace for packages and files"""
    
    # Directories to exclude from scanning
    EXCLUDED_DIRS = {
        'install', 'build', 'log', 'logs', '.git', '.vscode', 
        '__pycache__', 'build-*', 'install-*', 'log-*'
    }
    
    @staticmethod
    def scan_workspace(workspace_path: Path) -> List[ROS2Item]:
        """Scan workspace and return all ROS2 items"""
        items = []
        src_path = workspace_path / "src"
        if not src_path.exists():
            return items
        
        # Recursively find all packages (only in src directory)
        packages = WorkspaceScanner._find_all_packages(src_path)
        
        # Remove duplicates by converting to set of paths
        unique_packages = list(set(packages))
        
        for package_dir in unique_packages:
            items.extend(WorkspaceScanner._scan_package(package_dir))
        
        # Remove duplicate items based on path
        seen_paths = set()
        unique_items = []
        for item in items:
            if str(item.path) not in seen_paths:
                seen_paths.add(str(item.path))
                unique_items.append(item)
        
        return unique_items
    
    @staticmethod
    def _find_all_packages(root_path: Path) -> List[Path]:
        """Recursively find all ROS2 packages in src directory only"""
        packages = []
        
        def should_skip_dir(dir_name: str) -> bool:
            """Check if directory should be skipped"""
            # Skip excluded directories
            if dir_name in WorkspaceScanner.EXCLUDED_DIRS:
                return True
            # Skip hidden directories
            if dir_name.startswith('.'):
                return True
            # Skip build/install/log patterns
            if dir_name.startswith(('build', 'install', 'log')):
                return True
            return False
        
        def search_recursive(path: Path, depth: int = 0):
            # Limit recursion depth to prevent infinite loops
            if depth > 10:
                return
            
            # Check if current directory is a package
            if (path / "package.xml").exists():
                packages.append(path)
                # Continue searching subdirectories for nested packages
            
            # Search subdirectories
            try:
                for item in path.iterdir():
                    if item.is_dir() and not should_skip_dir(item.name):
                        search_recursive(item, depth + 1)
            except PermissionError:
                pass
        
        search_recursive(root_path)
        return packages
    
    @staticmethod
    def _scan_package(package_path: Path) -> List[ROS2Item]:
        """Scan a single package for launch files, nodes, and scripts"""
        items = []
        package_name = package_path.name
        
        # Only look in the direct 'launch' or 'launches' folder of the package
        # Do NOT scan subdirectories to avoid duplicates
        launch_dirs = [
            package_path / "launch",
            package_path / "launches"
        ]
        
        for launch_dir in launch_dirs:
            if launch_dir.exists() and launch_dir.is_dir():
                # Only get files directly in the launch directory, not subdirectories
                try:
                    for launch_file in launch_dir.iterdir():
                        if launch_file.is_file():
                            if launch_file.suffix == '.py' and not launch_file.name.startswith('_'):
                                params = WorkspaceScanner._extract_launch_parameters(launch_file)
                                items.append(ROS2Item(
                                    name=launch_file.name,
                                    path=launch_file,
                                    item_type=ItemType.LAUNCH_FILE,
                                    package_name=package_name,
                                    parameters=params
                                ))
                            elif launch_file.suffix == '.xml':
                                items.append(ROS2Item(
                                    name=launch_file.name,
                                    path=launch_file,
                                    item_type=ItemType.LAUNCH_FILE,
                                    package_name=package_name
                                ))
                except PermissionError:
                    pass
        
        # Find Python nodes - only in specific directories
        node_dirs = [
            package_path / "scripts",
            package_path / package_name  # Package module directory
        ]
        
        for node_dir in node_dirs:
            if node_dir.exists() and node_dir.is_dir():
                try:
                    # Only direct files, no subdirectories
                    for node_file in node_dir.iterdir():
                        if node_file.is_file() and node_file.suffix == '.py':
                            if node_file.name != "__init__.py" and not node_file.name.startswith('_'):
                                items.append(ROS2Item(
                                    name=node_file.name,
                                    path=node_file,
                                    item_type=ItemType.PYTHON_NODE,
                                    package_name=package_name
                                ))
                except PermissionError:
                    pass
        
        # Find shell scripts - only in scripts directory
        scripts_dir = package_path / "scripts"
        if scripts_dir.exists() and scripts_dir.is_dir():
            try:
                # Only direct files, no subdirectories
                for script_file in scripts_dir.iterdir():
                    if script_file.is_file() and script_file.suffix == '.sh':
                        items.append(ROS2Item(
                            name=script_file.name,
                            path=script_file,
                            item_type=ItemType.SCRIPT,
                            package_name=package_name
                        ))
            except PermissionError:
                pass
        
        return items
    
    @staticmethod
    def _extract_launch_parameters(launch_file: Path) -> Dict[str, Any]:
        """Extract parameters from launch file"""
        params = {}
        try:
            with open(launch_file, 'r') as f:
                content = f.read()
                # Improved regex to find DeclareLaunchArgument and optional default_value
                # Matches patterns like: DeclareLaunchArgument('arg', default_value='val')
                pattern = r"DeclareLaunchArgument\s*\(\s*['\"](?P<name>[^'\"]+)['\"](?:\s*,\s*default_value\s*=\s*(?P<default>[^),\n]+))?"
                matches = re.finditer(pattern, content)
                for m in matches:
                    name = m.group('name')
                    default = m.group('default')
                    if default is None:
                        params[name] = ""
                    else:
                        # Clean up common patterns: strip quotes and trailing characters
                        d = default.strip()
                        # Remove trailing commas or parentheses
                        d = re.sub(r"[\),\s]+$", "", d)
                        # Strip surrounding quotes
                        if (d.startswith("'") and d.endswith("'")) or (d.startswith('"') and d.endswith('"')):
                            d = d[1:-1]
                        params[name] = d
        except Exception:
            pass
        return params


class ParameterDialog(QDialog):
    """Dialog for editing launch file parameters"""
    
    def __init__(self, item: ROS2Item, config_manager: ConfigManager, parent=None):
        super().__init__(parent)
        self.item = item
        self.config_manager = config_manager
        self.param_widgets = {}
        self.setup_ui()
    
    def setup_ui(self):
        self.setWindowTitle(f"Parameters - {self.item.name}")
        self.setMinimumWidth(400)
        
        layout = QVBoxLayout(self)
        
        # Load saved config
        saved_config = self.config_manager.load_launch_config(
            f"{self.item.package_name}/{self.item.name}"
        )
        
        form_layout = QFormLayout()
        
        if self.item.parameters:
            for param_name, default_value in self.item.parameters.items():
                line_edit = QLineEdit()
                if saved_config and param_name in saved_config:
                    line_edit.setText(str(saved_config[param_name]))
                else:
                    line_edit.setText(str(default_value))
                self.param_widgets[param_name] = line_edit
                form_layout.addRow(f"{param_name}:", line_edit)
        else:
            label = QLabel("No parameters found for this launch file")
            layout.addWidget(label)
        
        layout.addLayout(form_layout)
        
        # Buttons
        button_box = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel | QDialogButtonBox.Save
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        save_btn = button_box.button(QDialogButtonBox.Save)
        save_btn.clicked.connect(self.save_config)
        layout.addWidget(button_box)
    
    def get_parameters(self) -> Dict[str, Any]:
        """Get parameter values"""
        params = {}
        for name, widget in self.param_widgets.items():
            params[name] = widget.text()
        return params
    
    def save_config(self):
        """Save configuration"""
        params = self.get_parameters()
        self.config_manager.save_launch_config(
            f"{self.item.package_name}/{self.item.name}",
            params
        )
        QMessageBox.information(self, "Saved", "Parameter configuration saved")


class ResourceMonitorWidget(QWidget):
    """Widget for monitoring process resources"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()
        self.process_name = None
        
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # CPU usage
        cpu_layout = QHBoxLayout()
        cpu_layout.addWidget(QLabel("CPU:"))
        self.cpu_bar = QProgressBar()
        self.cpu_bar.setMaximum(100)
        self.cpu_label = QLabel("0.0%")
        cpu_layout.addWidget(self.cpu_bar)
        cpu_layout.addWidget(self.cpu_label)
        
        # Memory usage
        mem_layout = QHBoxLayout()
        mem_layout.addWidget(QLabel("Memory:"))
        self.mem_bar = QProgressBar()
        self.mem_bar.setMaximum(1000)  # 1GB max display
        self.mem_label = QLabel("0.0 MB")
        mem_layout.addWidget(self.mem_bar)
        mem_layout.addWidget(self.mem_label)
        
        # Process info
        info_layout = QFormLayout()
        self.pid_label = QLabel("-")
        self.uptime_label = QLabel("-")
        self.status_label = QLabel("-")
        info_layout.addRow("PID:", self.pid_label)
        info_layout.addRow("Uptime:", self.uptime_label)
        info_layout.addRow("Status:", self.status_label)
        
        layout.addLayout(cpu_layout)
        layout.addLayout(mem_layout)
        layout.addLayout(info_layout)
        layout.addStretch()
    
    def update_stats(self, process_info: ProcessInfo):
        """Update resource statistics"""
        self.cpu_bar.setValue(int(process_info.cpu_percent))
        self.cpu_label.setText(f"{process_info.cpu_percent:.1f}%")
        
        self.mem_bar.setValue(int(process_info.memory_mb))
        self.mem_label.setText(f"{process_info.memory_mb:.1f} MB")
        
        self.pid_label.setText(str(process_info.pid))
        
        uptime = datetime.now() - process_info.start_time
        total = int(uptime.total_seconds())
        hours, remainder = divmod(total, 3600)
        minutes, seconds = divmod(remainder, 60)
        self.uptime_label.setText(f"{hours:02d}:{minutes:02d}:{seconds:02d}")
        
        self.status_label.setText(process_info.status)


class OutputWidget(QWidget):
    """Widget for displaying process output"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()
        
    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Output display
        self.output_text = QTextEdit()
        self.output_text.setReadOnly(True)
        self.output_text.setFont(QFont("Monospace", 9))
        
        # Resource monitor
        self.resource_monitor = ResourceMonitorWidget()
        
        # Control buttons
        button_layout = QHBoxLayout()
        
        self.clear_btn = QPushButton("Clear")
        self.clear_btn.clicked.connect(self.clear_output)
        
        self.export_btn = QPushButton("Export Log")
        
        self.stop_btn = QPushButton("Stop Process")
        self.stop_btn.setStyleSheet("background-color: #d32f2f; color: white;")
        
        button_layout.addWidget(self.clear_btn)
        button_layout.addWidget(self.export_btn)
        button_layout.addWidget(self.stop_btn)
        button_layout.addStretch()
        
        # Splitter for output and monitor
        splitter = QSplitter(Qt.Vertical)
        splitter.addWidget(self.output_text)
        splitter.addWidget(self.resource_monitor)
        splitter.setSizes([600, 150])
        
        layout.addWidget(splitter)
        layout.addLayout(button_layout)
    
    def append_output(self, text: str, color: str = "white"):
        """Append text to output"""
        self.output_text.append(f'<span style="color: {color};">{text}</span>')
        scrollbar = self.output_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def clear_output(self):
        """Clear output display"""
        self.output_text.clear()


class ROS2InspectorWidget(QWidget):
    """Widget for inspecting ROS2 system state"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()
        
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Refresh button
        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self.refresh_data)
        layout.addWidget(refresh_btn)
        
        # Tabs for different views
        tabs = QTabWidget()
        
        # Nodes
        self.nodes_list = QListWidget()
        # Make list non-editable by user
        try:
            self.nodes_list.setEditTriggers(QAbstractItemView.NoEditTriggers)
        except Exception:
            pass
        self.nodes_list.itemDoubleClicked.connect(self.show_node_info)
        tabs.addTab(self.nodes_list, "Nodes")
        
        # Topics
        self.topics_table = QTableWidget()
        self.topics_table.setColumnCount(2)
        self.topics_table.setHorizontalHeaderLabels(["Topic", "Type"])
        # Prevent manual editing of topic cells
        try:
            self.topics_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        except Exception:
            pass
        tabs.addTab(self.topics_table, "Topics")
        
        # Services
        self.services_list = QListWidget()
        try:
            self.services_list.setEditTriggers(QAbstractItemView.NoEditTriggers)
        except Exception:
            pass
        tabs.addTab(self.services_list, "Services")
        
        layout.addWidget(tabs)
        
    def refresh_data(self):
        """Refresh ROS2 system data"""
        # Nodes
        self.nodes_list.clear()
        nodes = ROS2Inspector.get_active_nodes()
        for node in nodes:
            self.nodes_list.addItem(node)
        
        # Topics
        self.topics_table.setRowCount(0)
        topics = ROS2Inspector.get_topics()
        for i, topic in enumerate(topics):
            self.topics_table.insertRow(i)
            self.topics_table.setItem(i, 0, QTableWidgetItem(topic["name"]))
            self.topics_table.setItem(i, 1, QTableWidgetItem(topic["type"]))
        # Allow double-clicking a topic to echo it in a new tab
        try:
            self.topics_table.itemDoubleClicked.connect(self.echo_topic)
        except Exception:
            pass
        
        # Services
        self.services_list.clear()
        services = ROS2Inspector.get_services()
        for service in services:
            self.services_list.addItem(service)
    
    def show_node_info(self, item):
        """Show detailed node information"""
        node_name = item.text()
        info = ROS2Inspector.get_node_info(node_name)
        if info:
            msg = QMessageBox()
            msg.setWindowTitle(f"Node Info - {node_name}")
            msg.setText(info.get("info", "No info available"))
            msg.exec_()

    def echo_topic(self, item: QTableWidgetItem):
        """Open a tab that runs `ros2 topic echo <topic>` for the double-clicked topic"""
        try:
            row = item.row()
            topic_item = self.topics_table.item(row, 0)
            if not topic_item:
                return
            topic_name = topic_item.text()
            main_window = self.window()
            tab_name = f"topic_echo/{topic_name}"
            cmd = ["ros2", "topic", "echo", topic_name]
            if hasattr(main_window, "_do_execute_item"):
                main_window._do_execute_item(tab_name, cmd)
            else:
                QMessageBox.warning(self, "Error", "Cannot echo topic (no handler)")
        except Exception as e:
            print(f"Error echoing topic: {e}")


class FavoritesWidget(QWidget):
    """Widget for managing favorite configurations"""
    
    def __init__(self, config_manager: ConfigManager, parent=None):
        super().__init__(parent)
        self.config_manager = config_manager
        self.setup_ui()
        self.load_favorites()
        
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Toolbar
        toolbar_layout = QHBoxLayout()
        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self.load_favorites)
        toolbar_layout.addWidget(refresh_btn)
        toolbar_layout.addStretch()
        layout.addLayout(toolbar_layout)
        
        # Favorites list
        self.favorites_list = QListWidget()
        self.favorites_list.setContextMenuPolicy(Qt.CustomContextMenu)
        self.favorites_list.customContextMenuRequested.connect(self.show_context_menu)
        layout.addWidget(self.favorites_list)
    
    def load_favorites(self):
        """Load and display favorites"""
        self.favorites_list.clear()
        favorites = self.config_manager.load_favorites()
        for fav in favorites:
            alias = fav.get("alias", fav["name"])
            package = fav.get("package", "")
            item_text = f"{alias} ({package})"
            self.favorites_list.addItem(item_text)
    
    def show_context_menu(self, position):
        """Show context menu for favorites"""
        menu = QMenu()
        run_action = menu.addAction("Run")
        remove_action = menu.addAction("Remove")
        
        action = menu.exec_(self.favorites_list.mapToGlobal(position))
        
        current_row = self.favorites_list.currentRow()
        if current_row >= 0:
            if action == remove_action:
                self.config_manager.remove_favorite(current_row)
                self.load_favorites()
            elif action == run_action:
                # Delegate execution to main window's run_favorite handler if available
                main_window = self.window()
                if hasattr(main_window, "run_favorite"):
                    item = self.favorites_list.currentItem()
                    main_window.run_favorite(item)
                else:
                    QMessageBox.warning(self, "Error", "Cannot run favorite (no handler)")


class NodeGraphWidget(QWidget):
    """Simple node graph visualization"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.nodes = []
        self.setMinimumSize(400, 300)
        
    def paintEvent(self, event):
        """Draw simple node graph"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw title
        painter.setPen(QPen(Qt.white, 2))
        painter.drawText(10, 20, "Active Nodes Graph")
        
        # Draw nodes in a circle
        if self.nodes:
            center_x = self.width() // 2
            center_y = self.height() // 2
            radius = min(self.width(), self.height()) // 3
            
            import math
            for i, node in enumerate(self.nodes):
                angle = 2 * math.pi * i / len(self.nodes)
                x = int(center_x + radius * math.cos(angle))
                y = int(center_y + radius * math.sin(angle))
                
                # Draw node
                painter.setBrush(QColor(100, 150, 200))
                painter.drawEllipse(x - 20, y - 20, 40, 40)
                
                # Draw label
                painter.setPen(QPen(Qt.white, 1))
                node_name = node.split('/')[-1][:10]
                painter.drawText(x - 25, y + 35, node_name)
        else:
            painter.drawText(self.width() // 2 - 50, self.height() // 2, "No active nodes")
    
    def update_nodes(self, nodes: List[str]):
        """Update node list and redraw"""
        self.nodes = nodes
        self.update()


class ROS2LauncherGUI(QMainWindow):
    """Main GUI window with all enhancements"""
    
    def __init__(self):
        super().__init__()
        self.workspace_path: Optional[Path] = None
        self.process_manager = ProcessManager()
        self.config_manager = ConfigManager(Path.home() / ".ros2_launcher")
        # Load user preference for stop signal
        self.stop_signal = self.config_manager.load_setting('stop_signal', 'SIGINT')
        self.current_items: List[ROS2Item] = []
        self.active_tabs: Dict[str, OutputWidget] = {}
        self.workspace_sourced = False
        
        self.setup_ui()
        self.setup_timers()
        self.load_recent_workspaces()
        # Restore previous session (tabs and logs)
        QTimer.singleShot(100, self._restore_session)
    
    def setup_ui(self):
        """Setup the user interface"""
        self.setWindowTitle("ROS2 GUI Launcher - Full Featured")
        self.setGeometry(100, 100, 1400, 900)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Toolbar
        self.create_toolbar()
        
        # Workspace path display
        path_layout = QHBoxLayout()
        path_layout.addWidget(QLabel("Workspace:"))
        self.workspace_label = QLabel("No workspace loaded")
        self.workspace_label.setStyleSheet("color: gray;")
        path_layout.addWidget(self.workspace_label)
        
        # Source status indicator
        self.source_indicator = QLabel("●")
        self.source_indicator.setStyleSheet("color: red; font-size: 20px;")
        self.source_indicator.setToolTip("Workspace not sourced")
        path_layout.addWidget(self.source_indicator)
        
        # Recent workspaces combo
        self.recent_combo = QComboBox()
        self.recent_combo.setMinimumWidth(200)
        self.recent_combo.currentTextChanged.connect(self.load_recent_workspace)
        path_layout.addWidget(QLabel("Recent:"))
        path_layout.addWidget(self.recent_combo)
        path_layout.addStretch()
        main_layout.addLayout(path_layout)
        
        # Main splitter (3 panels)
        main_splitter = QSplitter(Qt.Horizontal)
        
        # Left panel - Project tree and favorites
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        
        # Tabs for tree and favorites
        left_tabs = QTabWidget()
        
        # Project tree tab
        tree_widget = QWidget()
        tree_layout = QVBoxLayout(tree_widget)
        tree_layout.setContentsMargins(0, 0, 0, 0)
        
        self.search_box = QLineEdit()
        self.search_box.setPlaceholderText("Search items...")
        self.search_box.textChanged.connect(self.filter_tree)
        tree_layout.addWidget(self.search_box)
        
        self.tree_widget = QTreeWidget()
        self.tree_widget.setHeaderLabel("ROS2 Project Structure")
        self.tree_widget.setContextMenuPolicy(Qt.CustomContextMenu)
        self.tree_widget.customContextMenuRequested.connect(self.show_tree_context_menu)
        self.tree_widget.itemDoubleClicked.connect(self.on_item_double_clicked)
        tree_layout.addWidget(self.tree_widget)
        
        left_tabs.addTab(tree_widget, "Project")
        
        # Favorites tab
        self.favorites_widget = FavoritesWidget(self.config_manager)
        self.favorites_widget.favorites_list.itemDoubleClicked.connect(self.run_favorite)
        left_tabs.addTab(self.favorites_widget, "Favorites")
        
        left_layout.addWidget(left_tabs)
        
        # Middle panel - Output tabs
        self.tab_widget = QTabWidget()
        self.tab_widget.setTabsClosable(True)
        self.tab_widget.tabCloseRequested.connect(self.close_tab)
        
        # Right panel - ROS2 Inspector and Node Graph
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        
        right_tabs = QTabWidget()
        
        # ROS2 Inspector
        self.inspector_widget = ROS2InspectorWidget()
        right_tabs.addTab(self.inspector_widget, "ROS2 Inspector")
        
        # Node Graph
        self.node_graph = NodeGraphWidget()
        graph_container = QWidget()
        graph_layout = QVBoxLayout(graph_container)
        graph_layout.addWidget(self.node_graph)
        refresh_graph_btn = QPushButton("Refresh Graph")
        refresh_graph_btn.clicked.connect(self.refresh_node_graph)
        graph_layout.addWidget(refresh_graph_btn)
        right_tabs.addTab(graph_container, "Node Graph")
        
        right_layout.addWidget(right_tabs)
        
        # Add all panels to main splitter
        main_splitter.addWidget(left_panel)
        main_splitter.addWidget(self.tab_widget)
        main_splitter.addWidget(right_panel)
        main_splitter.setSizes([300, 700, 400])
        
        main_layout.addWidget(main_splitter)
        
        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready")
    
    def create_toolbar(self):
        """Create toolbar with actions"""
        toolbar = QToolBar()
        self.addToolBar(toolbar)
        
        # File menu actions
        load_action = QAction("📁 Load Workspace", self)
        load_action.triggered.connect(self.load_workspace)
        toolbar.addAction(load_action)
        
        refresh_action = QAction("🔄 Refresh", self)
        refresh_action.triggered.connect(self.refresh_workspace)
        toolbar.addAction(refresh_action)
        
        toolbar.addSeparator()
        
        # Build actions
        build_action = QAction("🔨 Build Workspace", self)
        build_action.triggered.connect(self.build_workspace)
        toolbar.addAction(build_action)
        
        source_action = QAction("⚡ Source Install", self)
        source_action.triggered.connect(self.source_workspace)
        toolbar.addAction(source_action)
        
        toolbar.addSeparator()
        
        # Process control
        stop_all_action = QAction("⏹ Stop All", self)
        stop_all_action.triggered.connect(self.stop_all_processes)
        toolbar.addAction(stop_all_action)
        
        toolbar.addSeparator()
        
        # System inspection
        inspect_action = QAction("🔍 Inspect System", self)
        inspect_action.triggered.connect(self.inspect_system)
        toolbar.addAction(inspect_action)
        
        # Export logs
        export_action = QAction("💾 Export All Logs", self)
        export_action.triggered.connect(self.export_all_logs)
        toolbar.addAction(export_action)
        # Stop mode selector
        self.stop_mode_combo = QComboBox()
        self.stop_mode_combo.addItems(["SIGINT", "SIGTERM"])
        # Load saved preference
        saved = self.config_manager.load_setting('stop_signal', 'SIGINT')
        idx = 0 if saved == 'SIGINT' else 1
        self.stop_mode_combo.setCurrentIndex(idx)
        self.stop_mode_combo.setToolTip("Preferred first signal when stopping processes")
        self.stop_mode_combo.currentTextChanged.connect(self._on_stop_mode_changed)
        toolbar.addWidget(self.stop_mode_combo)
    
    def setup_timers(self):
        """Setup timers for updates"""
        # Output update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_outputs)
        self.update_timer.start(100)
        
        # Resource monitor timer
        self.resource_timer = QTimer()
        self.resource_timer.timeout.connect(self.update_resource_monitors)
        self.resource_timer.start(1000)
        
        # Auto-refresh inspector
        self.inspector_timer = QTimer()
        self.inspector_timer.timeout.connect(self.auto_refresh_inspector)
        self.inspector_timer.start(5000)
    
    def load_recent_workspaces(self):
        """Load recent workspaces into combo box"""
        recent = self.config_manager.load_recent_workspaces()
        self.recent_combo.clear()
        self.recent_combo.addItem("-- Select Recent --")
        for workspace in recent:
            self.recent_combo.addItem(workspace)
    
    def load_recent_workspace(self, workspace_path: str):
        """Load workspace from recent list"""
        if workspace_path and workspace_path != "-- Select Recent --":
            self.workspace_path = Path(workspace_path)
            self.workspace_label.setText(str(self.workspace_path))
            self.workspace_label.setStyleSheet("color: black;")
            self.check_workspace_status()
            self.refresh_workspace()
    
    def load_workspace(self):
        """Load a ROS2 workspace"""
        workspace_path = QFileDialog.getExistingDirectory(
            self,
            "Select ROS2 Workspace",
            str(Path.home())
        )
        
        if not workspace_path:
            return
        
        self.workspace_path = Path(workspace_path)
        self.workspace_label.setText(str(self.workspace_path))
        self.workspace_label.setStyleSheet("color: black;")
        
        # Add to recent
        self.config_manager.add_recent_workspace(str(self.workspace_path))
        self.load_recent_workspaces()
        
        self.check_workspace_status()
        self.refresh_workspace()
    
    def check_workspace_status(self):
        """Check if workspace is built and sourced"""
        if not self.workspace_path:
            return
        
        install_dir = self.workspace_path / "install"
        if install_dir.exists():
            self.workspace_sourced = True
            self.source_indicator.setStyleSheet("color: green; font-size: 20px;")
            self.source_indicator.setToolTip("Workspace sourced and ready")
        else:
            self.workspace_sourced = False
            self.source_indicator.setStyleSheet("color: orange; font-size: 20px;")
            self.source_indicator.setToolTip("Workspace not built - click Build Workspace")
    
    def build_workspace(self):
        """Build the workspace using colcon"""
        if not self.workspace_path:
            QMessageBox.warning(self, "Warning", "No workspace loaded")
            return
        
        dialog = BuildDialog(self.workspace_path, self)
        if dialog.exec_() == QDialog.Accepted:
            build_cmd = dialog.get_build_command()
            
            # Create build tab
            tab_name = "Build Process"
            if tab_name not in self.active_tabs:
                output_widget = OutputWidget()
                output_widget.stop_btn.clicked.connect(
                    lambda: self.stop_process(tab_name)
                )
                output_widget.export_btn.clicked.connect(
                    lambda: self.export_log(tab_name)
                )
                self.active_tabs[tab_name] = output_widget
                self.tab_widget.addTab(output_widget, tab_name)
            
            idx = self.tab_widget.indexOf(self.active_tabs[tab_name])
            self.tab_widget.setCurrentIndex(idx)
            
            output_widget = self.active_tabs[tab_name]
            output_widget.clear_output()
            output_widget.append_output(f"Building workspace...\n", "cyan")
            output_widget.append_output(f"$ cd {self.workspace_path}\n", "cyan")
            output_widget.append_output(f"$ {' '.join(build_cmd)}\n\n", "cyan")
            
            # Start build process
            log_dir = self.workspace_path / "logs"
            success = self.process_manager.start_process(
                tab_name, build_cmd, self.workspace_path, log_dir
            )
            
            if success:
                self.status_bar.showMessage("Building workspace...")
                # Check status after build
                QTimer.singleShot(2000, self.check_build_completion)
            else:
                output_widget.append_output("Failed to start build\n", "red")
    
    def check_build_completion(self):
        """Check if build is complete and update status"""
        if not self.process_manager.is_running("Build Process"):
            self.check_workspace_status()
            if self.workspace_sourced:
                self.status_bar.showMessage("Build completed successfully")
            else:
                self.status_bar.showMessage("Build process finished")
    
    def source_workspace(self):
        """Source the workspace install/setup.bash"""
        if not self.workspace_path:
            QMessageBox.warning(self, "Warning", "No workspace loaded")
            return
        
        install_dir = self.workspace_path / "install"
        if not install_dir.exists():
            reply = QMessageBox.question(
                self,
                "Build Required",
                "Workspace not built. Build now?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                self.build_workspace()
            return
        
        self.workspace_sourced = True
        self.check_workspace_status()
        QMessageBox.information(
            self,
            "Success",
            f"Workspace sourced!\n\nAll launch commands will use:\nsource {install_dir}/setup.bash"
        )
    
    def refresh_workspace(self):
        """Refresh workspace contents"""
        if not self.workspace_path:
            QMessageBox.warning(self, "Warning", "No workspace loaded")
            return
        
        self.status_bar.showMessage("Scanning workspace...")
        self.tree_widget.clear()
        
        # Scan workspace
        self.current_items = WorkspaceScanner.scan_workspace(self.workspace_path)
        
        # Organize by package
        packages: Dict[str, List[ROS2Item]] = {}
        for item in self.current_items:
            if item.package_name not in packages:
                packages[item.package_name] = []
            packages[item.package_name].append(item)
        
        # Populate tree
        for package_name, items in sorted(packages.items()):
            package_item = QTreeWidgetItem(self.tree_widget, [package_name])
            package_item.setData(0, Qt.UserRole, None)
            
            # Group by type
            launch_files = [i for i in items if i.item_type == ItemType.LAUNCH_FILE]
            nodes = [i for i in items if i.item_type in [ItemType.PYTHON_NODE, ItemType.CPP_NODE]]
            scripts = [i for i in items if i.item_type == ItemType.SCRIPT]
            
            if launch_files:
                launch_parent = QTreeWidgetItem(package_item, ["Launch Files"])
                for item in launch_files:
                    child = QTreeWidgetItem(launch_parent, [item.name])
                    child.setData(0, Qt.UserRole, item)
                    if item.parameters:
                        child.setToolTip(0, f"Parameters: {', '.join(item.parameters.keys())}")
            
            if nodes:
                nodes_parent = QTreeWidgetItem(package_item, ["Nodes"])
                for item in nodes:
                    child = QTreeWidgetItem(nodes_parent, [item.name])
                    child.setData(0, Qt.UserRole, item)
            
            if scripts:
                scripts_parent = QTreeWidgetItem(package_item, ["Scripts"])
                for item in scripts:
                    child = QTreeWidgetItem(scripts_parent, [item.name])
                    child.setData(0, Qt.UserRole, item)
            
            package_item.setExpanded(True)
        
        self.status_bar.showMessage(f"Found {len(self.current_items)} items in {len(packages)} packages")
    
    def filter_tree(self, text: str):
        """Filter tree items based on search text"""
        iterator = QTreeWidgetItemIterator(self.tree_widget)
        while iterator.value():
            item = iterator.value()
            item_text = item.text(0).lower()
            should_show = text.lower() in item_text
            item.setHidden(not should_show and text != "")
            iterator += 1
    
    def show_tree_context_menu(self, position):
        """Show context menu for tree items"""
        item = self.tree_widget.itemAt(position)
        if not item:
            return
        
        ros2_item: ROS2Item = item.data(0, Qt.UserRole)
        if not ros2_item:
            return
        
        menu = QMenu()
        run_action = menu.addAction("Run")
        
        if ros2_item.item_type == ItemType.LAUNCH_FILE and ros2_item.parameters:
            params_action = menu.addAction("Run with Parameters...")
        else:
            params_action = None
        
        favorite_action = menu.addAction("Add to Favorites")
        
        action = menu.exec_(self.tree_widget.mapToGlobal(position))
        
        if action == run_action:
            self.execute_item(ros2_item)
        elif action == params_action:
            self.execute_with_parameters(ros2_item)
        elif action == favorite_action:
            self.add_to_favorites(ros2_item)
    
    def on_item_double_clicked(self, item: QTreeWidgetItem, column: int):
        """Handle double-click on tree item"""
        ros2_item: ROS2Item = item.data(0, Qt.UserRole)
        if not ros2_item:
            return
        
        # Check if workspace is sourced
        if not self.workspace_sourced:
            reply = QMessageBox.question(
                self,
                "Source Required",
                "Workspace needs to be built and sourced. Build now?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                self.build_workspace()
            return
        
        # If has parameters, show dialog
        if ros2_item.item_type == ItemType.LAUNCH_FILE and ros2_item.parameters:
            self.execute_with_parameters(ros2_item)
        else:
            self.execute_item(ros2_item)
    
    def execute_with_parameters(self, item: ROS2Item):
        """Execute item with parameter dialog"""
        dialog = ParameterDialog(item, self.config_manager, self)
        if dialog.exec_() == QDialog.Accepted:
            params = dialog.get_parameters()
            self.execute_item(item, params)
    
    def execute_item(self, item: ROS2Item, params: Optional[Dict] = None):
        """Execute a ROS2 item"""
        command = item.get_command(params)
        if not command:
            QMessageBox.warning(self, "Warning", "Cannot execute this item")
            return
        
        tab_name = f"{item.package_name}/{item.name}"
        
        if self.process_manager.is_running(tab_name):
            reply = QMessageBox.question(
                self,
                "Already Running",
                f"{tab_name} is already running. Restart?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                self.stop_process(tab_name)
                # Wait a bit for process to stop
                QTimer.singleShot(500, lambda: self._execute_item_delayed(item, params, tab_name, command))
                return
            else:
                return
        
        self._do_execute_item(tab_name, command)
    
    def _execute_item_delayed(self, item: ROS2Item, params: Optional[Dict], tab_name: str, command: List[str]):
        """Execute item after delay"""
        self._do_execute_item(tab_name, command)
    
    def _do_execute_item(self, tab_name: str, command: List[str]):
        """Actually execute the item"""
        # Create or reuse tab
        if tab_name not in self.active_tabs:
            output_widget = OutputWidget()
            output_widget.stop_btn.clicked.connect(
                lambda: self.stop_process(tab_name)
            )
            output_widget.export_btn.clicked.connect(
                lambda: self.export_log(tab_name)
            )
            self.active_tabs[tab_name] = output_widget
            self.tab_widget.addTab(output_widget, tab_name)
        
        idx = self.tab_widget.indexOf(self.active_tabs[tab_name])
        self.tab_widget.setCurrentIndex(idx)
        
        output_widget = self.active_tabs[tab_name]
        output_widget.clear_output()
        output_widget.append_output(f"$ {' '.join(command)}\n", "cyan")
        
        log_dir = self.workspace_path / "logs" if self.workspace_path else None
        success = self.process_manager.start_process(
            tab_name, command, self.workspace_path, log_dir
        )
        
        if success:
            self.status_bar.showMessage(f"Started: {tab_name}")
            # Save session (open tabs + logs)
            try:
                self._save_session()
            except Exception:
                pass
        else:
            output_widget.append_output("Failed to start process\n", "red")
    
    def add_to_favorites(self, item: ROS2Item):
        """Add item to favorites"""
        alias, ok = QInputDialog.getText(
            self, "Add to Favorites", "Enter alias (optional):",
            text=item.name
        )
        if ok:
            self.config_manager.save_favorite(item, alias=alias)
            self.favorites_widget.load_favorites()
            QMessageBox.information(self, "Success", "Added to favorites")
    
    def run_favorite(self, list_item):
        """Run a favorite configuration"""
        favorites = self.config_manager.load_favorites()
        idx = self.favorites_widget.favorites_list.row(list_item)
        
        if 0 <= idx < len(favorites):
            fav = favorites[idx]
            # Reconstruct ROS2Item
            item = ROS2Item(
                name=fav["name"],
                path=Path(fav["path"]),
                item_type=ItemType(fav["type"]),
                package_name=fav["package"],
                parameters=fav.get("parameters")
            )
            self.execute_item(item, fav.get("parameters"))
    
    def stop_process(self, name: str):
        """Stop a specific process"""
        if name in self.active_tabs:
            self.active_tabs[name].append_output("\n[Stopping process...]\n", "yellow")
        
        if self.process_manager.stop_process(name, preferred_first_signal=self.stop_signal):
            if name in self.active_tabs:
                self.active_tabs[name].append_output("[Process stopped]\n", "yellow")
            self.status_bar.showMessage(f"Stopped: {name}")
        else:
            if name in self.active_tabs:
                self.active_tabs[name].append_output("[Failed to stop process]\n", "red")

    def _on_stop_mode_changed(self, text: str):
        """Handle changes to preferred stop signal"""
        self.stop_signal = text
        self.config_manager.save_setting('stop_signal', text)

    def _save_session(self):
        """Save current open tabs and associated log files to session"""
        session = []
        for name, widget in self.active_tabs.items():
            entry = {"name": name}
            log_path = None
            if name in self.process_manager.log_files:
                try:
                    log_path = str(self.process_manager.log_files[name])
                except Exception:
                    log_path = None
            entry["log"] = log_path
            session.append(entry)
        self.config_manager.save_session(session)

    def _restore_session(self):
        """Restore tabs from previous session (loads logs into tabs). Does NOT restart processes."""
        session = self.config_manager.load_session()
        for entry in session:
            name = entry.get("name")
            log = entry.get("log")
            if not name:
                continue
            if name in self.active_tabs:
                continue
            output_widget = OutputWidget()
            output_widget.stop_btn.clicked.connect(lambda n=name: self.stop_process(n))
            output_widget.export_btn.clicked.connect(lambda n=name: self.export_log(n))
            self.active_tabs[name] = output_widget
            self.tab_widget.addTab(output_widget, name)
            # Load log file if present
            if log:
                try:
                    p = Path(log)
                    if p.exists():
                        with open(p, 'r') as f:
                            for line in f:
                                output_widget.append_output(line.rstrip(), "white")
                except Exception:
                    pass
    
    def stop_all_processes(self):
        """Stop all running processes"""
        count = 0
        for name in list(self.active_tabs.keys()):
            if self.process_manager.is_running(name):
                self.stop_process(name)
                count += 1
        
        if count > 0:
            QMessageBox.information(self, "Stopped", f"Stopped {count} process(es)")
        else:
            QMessageBox.information(self, "Info", "No running processes")
    
    def close_tab(self, index: int):
        """Close a tab"""
        tab_name = self.tab_widget.tabText(index)
        
        if self.process_manager.is_running(tab_name):
            reply = QMessageBox.question(
                self,
                "Confirm",
                f"Process is still running. Stop and close?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                self.stop_process(tab_name)
            else:
                return
        
        self.tab_widget.removeTab(index)
        if tab_name in self.active_tabs:
            del self.active_tabs[tab_name]
        # Update session after closing tab
        try:
            self._save_session()
        except Exception:
            pass
    
    def update_outputs(self):
        """Update output displays"""
        for name, widget in self.active_tabs.items():
            lines = self.process_manager.get_output(name)
            for line in lines:
                widget.append_output(line.rstrip(), "white")
    
    def update_resource_monitors(self):
        """Update resource monitors"""
        self.process_manager.update_resource_usage()
        
        for name, widget in self.active_tabs.items():
            process_info = self.process_manager.get_process_info(name)
            if process_info:
                widget.resource_monitor.update_stats(process_info)
    
    def inspect_system(self):
        """Refresh ROS2 system inspection"""
        self.inspector_widget.refresh_data()
        self.refresh_node_graph()
        self.status_bar.showMessage("System inspection refreshed")
    
    def auto_refresh_inspector(self):
        """Auto-refresh inspector if visible"""
        self.refresh_node_graph()
    
    def refresh_node_graph(self):
        """Refresh node graph visualization"""
        nodes = ROS2Inspector.get_active_nodes()
        self.node_graph.update_nodes(nodes)
    
    def export_log(self, name: str):
        """Export log for a specific process"""
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Export Log",
            f"{name.replace('/', '_')}.log",
            "Log Files (*.log);;All Files (*)"
        )
        
        if file_path:
            if self.process_manager.export_log(name, Path(file_path)):
                QMessageBox.information(self, "Success", "Log exported successfully")
            else:
                QMessageBox.warning(self, "Warning", "No log file available")
    
    def export_all_logs(self):
        """Export all logs"""
        directory = QFileDialog.getExistingDirectory(
            self,
            "Select Directory for Logs",
            str(Path.home())
        )
        
        if directory:
            count = 0
            for name in self.active_tabs.keys():
                export_path = Path(directory) / f"{name.replace('/', '_')}.log"
                if self.process_manager.export_log(name, export_path):
                    count += 1
            
            QMessageBox.information(
                self, "Success", f"Exported {count} log file(s)"
            )
    
    def closeEvent(self, event):
        """Handle window close"""
        # Check if any processes are running
        running_processes = [name for name in self.active_tabs.keys() 
                           if self.process_manager.is_running(name)]
        
        if running_processes:
            reply = QMessageBox.question(
                self,
                "Confirm Exit",
                f"{len(running_processes)} process(es) still running. Stop all and exit?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                self.stop_all_processes()
                # Give processes time to stop
                QTimer.singleShot(1000, lambda: event.accept())
                event.ignore()
            else:
                event.ignore()
        else:
            event.accept()


def main():
    """Main entry point"""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    # Dark theme
    palette = app.palette()
    palette.setColor(palette.Window, QColor(53, 53, 53))
    palette.setColor(palette.WindowText, Qt.white)
    palette.setColor(palette.Base, QColor(25, 25, 25))
    palette.setColor(palette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(palette.ToolTipBase, Qt.white)
    palette.setColor(palette.ToolTipText, Qt.white)
    palette.setColor(palette.Text, Qt.white)
    palette.setColor(palette.Button, QColor(53, 53, 53))
    palette.setColor(palette.ButtonText, Qt.white)
    palette.setColor(palette.BrightText, Qt.red)
    palette.setColor(palette.Link, QColor(42, 130, 218))
    palette.setColor(palette.Highlight, QColor(42, 130, 218))
    palette.setColor(palette.HighlightedText, Qt.black)
    app.setPalette(palette)
    
    window = ROS2LauncherGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()