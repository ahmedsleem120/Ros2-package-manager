ROS2 GUI Launcher
=================

A full-featured PyQt5 GUI for discovering, launching, monitoring and managing ROS 2 workspaces, launch files, nodes and scripts.

Key features
- Workspace discovery and scanning (packages, launch files, nodes, scripts)
- Tree view organized by package and item type
- Run launch files, nodes and scripts with per-process tabs and live output
- Resource monitoring (CPU / memory per process)
- Favorites and saved launch parameter sets
- Build workspace (colcon) and source install/setup.bash before running
- ROS2 system inspection: nodes, topics and services
- Simple node graph visualization
- Session persistence: restore open tabs and logs

Quick start
1. Create a Python virtual environment (optional but recommended):

```bash
python3 -m venv .venv
source .venv/bin/activate
```

2. Install dependencies:

```bash
pip install -r requirements.txt
```

3. Run the GUI:

```bash
python3 "ros2_gui_launcherV3.py"
```

4. In the GUI: click "Load Workspace" and select your ROS2 workspace root (the folder that contains `src/`).

Notes
- This project expects ROS 2 and `colcon` available on your PATH when you run launch or build commands.
- The GUI persists user settings and session data in `~/.ros2_launcher/` by default.

Testing

Run the unit tests (requires pytest):

```bash
pytest -q
```

Publishing to GitHub
1. Create an empty repository on GitHub (via website or `gh repo create`).
2. Add the remote and push:

```bash
git remote add origin git@github.com:<youruser>/<repo>.git
git branch -M main
git push -u origin main
```

If you want me to help add the remote and push, provide the repository URL (or a GitHub token) and I'll help with the push command.

License

This repository includes an MIT license (see LICENSE). Replace the placeholder name/year if desired.
