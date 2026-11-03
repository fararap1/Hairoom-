"""Single source of truth for where this repo keeps its own files.

Everything resolves relative to this file, so the repo can be cloned or moved
anywhere without editing paths. Two escape hatches, both optional:

    HAIROOM_LANDMARKS  absolute path to the landmarks YAML
    HAIROOM_WS         ROS 2 workspace root (only needed by tooling that has
                       to reach outside this repo)
"""
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

CONFIG_DIR = os.path.join(REPO_ROOT, 'config')
SCRIPTS_DIR = os.path.join(REPO_ROOT, 'scripts')
REFERENCE_DIR = os.path.join(REPO_ROOT, 'reference')

LANDMARKS_PATH = os.environ.get(
    'HAIROOM_LANDMARKS', os.path.join(CONFIG_DIR, 'head_landmarks.yaml'))

ROS_WS = os.environ.get(
    'HAIROOM_WS', os.path.expanduser('~/hairoom_ws'))

# Robot controller. The XML-RPC command channel is the same one the official
# SDK uses; it is read-only in this repo's tooling except inside the driver.
ROBOT_IP = os.environ.get('HAIROOM_ROBOT_IP', '192.168.58.2')
ROBOT_RPC_URL = f'http://{ROBOT_IP}:20003/RPC2'
