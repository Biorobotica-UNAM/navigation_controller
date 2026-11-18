"""Validate launch integration without starting hardware nodes."""
from pathlib import Path
import xml.etree.ElementTree as ET

from launch.launch_description_sources import get_launch_description_from_any_launch_file


def test_launches_use_one_action_coordinator_and_load_its_recovery_configuration():
    directory = Path(__file__).resolve().parents[1] / 'launch'
    paths = sorted(directory.glob('navigation_*_*.launch.xml'))
    assert len(paths) == 4
    for path in paths:
        root = ET.parse(path).getroot()
        nodes = list(root.iter('node'))
        coordinators = [node for node in nodes if node.get('pkg') == 'navigation_manager']
        assert len(coordinators) == 1, path
        assert coordinators[0].get('exec') == 'navigation_manager_node'
        controllers = [node for node in nodes if node.get('pkg') == 'simple_move']
        assert len(controllers) == 1
        assert any(p.get('name') == 'motion_node' and p.get('value') == controllers[0].get('name')
                   for p in coordinators[0].findall('param'))
        assert any('navigation_manager.yaml' in p.get('from', '') for p in coordinators[0].findall('param'))
        assert not any(remap.get('from') == '/nav_control/goal' for remap in coordinators[0].findall('remap'))
        get_launch_description_from_any_launch_file(str(path))
