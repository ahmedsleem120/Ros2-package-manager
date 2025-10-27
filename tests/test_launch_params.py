import tempfile
import sys
import os
from pathlib import Path
# Ensure project root is on sys.path so the module can be imported
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from ros2_gui_launcherV3 import WorkspaceScanner


def test_extract_launch_params_simple(tmp_path):
    content = """
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

ld = LaunchDescription()
ld.add_action(DeclareLaunchArgument('foo', default_value='bar'))
ld.add_action(DeclareLaunchArgument("number", default_value=123))
ld.add_action(DeclareLaunchArgument('no_default'))
"""
    f = tmp_path / "test_launch.py"
    f.write_text(content)

    params = WorkspaceScanner._extract_launch_parameters(f)
    assert 'foo' in params and params['foo'] == 'bar'
    assert 'number' in params and params['number'] == '123'
    assert 'no_default' in params and params['no_default'] == ''
