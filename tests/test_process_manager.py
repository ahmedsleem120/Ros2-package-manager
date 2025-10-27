import time
import sys
import os
from pathlib import Path
# Ensure project root is on sys.path so the module can be imported
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from ros2_gui_launcherV3 import ProcessManager

def test_start_and_stop_process(tmp_path):
    # Create a small python script that handles SIGINT and exits
    script = tmp_path / "sigint_script.py"
    script.write_text("""
import signal
import time
import sys

def handler(sig, frame):
    print('caught')
    sys.stdout.flush()
    sys.exit(0)

signal.signal(signal.SIGINT, handler)
print('started')
sys.stdout.flush()
while True:
    time.sleep(0.5)
""")

    pm = ProcessManager()
    # Start the process
    log_dir = tmp_path / "logs"
    # Use tmp_path as workspace_path to avoid sourcing any local install/setup.bash
    success = pm.start_process('testproc', ['python3', str(script)], tmp_path, log_dir)
    assert success
    time.sleep(0.5)
    assert pm.is_running('testproc')

    # Stop with SIGINT preferred
    stopped = pm.stop_process('testproc', preferred_first_signal='SIGINT')
    assert stopped
    time.sleep(0.2)
    assert not pm.is_running('testproc')
    # Log file should exist
    if 'testproc' in pm.log_files:
        assert pm.log_files['testproc'].exists()
