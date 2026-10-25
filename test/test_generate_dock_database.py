import importlib.util
import json
import math
from pathlib import Path

import pytest
import yaml


SCRIPT = (
    Path(__file__).parents[1] / 'scripts' / 'generate_dock_database.py'
)
SPEC = importlib.util.spec_from_file_location(
    'generate_dock_database', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def write_pose(path, data):
    path.write_text(json.dumps(data), encoding='utf-8')


def test_database_uses_saved_dock_pose(tmp_path):
    source = tmp_path / 'downstairs.dock.json'
    output = tmp_path / 'dock-database.yaml'
    write_pose(source, {
        'map': 'downstairs',
        'frame': 'map',
        'dock': {'x': 1.2, 'y': -0.4, 'yaw': 0.1},
    })

    entry = MODULE.read_dock_entry(source, 'downstairs', 0.215)
    MODULE.write_database(output, entry)

    assert yaml.safe_load(output.read_text(encoding='utf-8')) == {
        'docks': {
            'home_dock': {
                'type': 's5max',
                'frame': 'map',
                'pose': [1.2, -0.4, 0.1],
            },
        },
    }


def test_database_derives_dock_pose_from_existing_stage_file(tmp_path):
    source = tmp_path / 'downstairs.dock.json'
    write_pose(source, {
        'map': 'downstairs',
        'frame': 'map',
        'stage': {'x': 1.0, 'y': 2.0, 'yaw': math.pi / 2.0},
    })

    entry = MODULE.read_dock_entry(source, 'downstairs', 0.215)

    assert entry['pose'] == pytest.approx([1.0, 2.215, math.pi / 2.0])


def test_database_rejects_another_map(tmp_path):
    source = tmp_path / 'downstairs.dock.json'
    write_pose(source, {
        'map': 'upstairs',
        'frame': 'map',
        'stage': {'x': 1.0, 'y': 2.0, 'yaw': 0.0},
    })

    with pytest.raises(ValueError, match='different map'):
        MODULE.read_dock_entry(source, 'downstairs', 0.215)
