#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path
import sys

import yaml


def dock_pose_from_stage(stage, offset):
    x = float(stage['x'])
    y = float(stage['y'])
    yaw = float(stage['yaw'])
    return (
        x + float(offset) * math.cos(yaw),
        y + float(offset) * math.sin(yaw),
        yaw,
    )


def read_dock_entry(path, map_name, offset):
    with open(path, encoding='utf-8') as source:
        data = json.load(source)
    if data.get('map') != map_name:
        raise ValueError('The dock pose belongs to a different map.')
    if data.get('frame') != 'map':
        raise ValueError('The dock pose does not use the map frame.')
    if 'dock' in data:
        dock = data['dock']
        pose = (
            float(dock['x']),
            float(dock['y']),
            float(dock['yaw']),
        )
    else:
        pose = dock_pose_from_stage(data['stage'], offset)
    if not all(math.isfinite(value) for value in pose):
        raise ValueError('The dock pose contains a non-finite value.')
    return {
        'type': 's5max',
        'frame': 'map',
        'pose': list(pose),
    }


def write_database(path, dock_entry):
    destination = Path(path)
    pending = destination.with_name(f'.{destination.name}.pending')
    with open(pending, 'w', encoding='utf-8') as target:
        yaml.safe_dump(
            {'docks': {'home_dock': dock_entry}},
            target,
            sort_keys=False,
        )
    pending.replace(destination)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True)
    parser.add_argument('--map-name', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--dock-offset', type=float, default=0.215)
    options = parser.parse_args(args)

    try:
        dock_entry = read_dock_entry(
            options.input,
            options.map_name,
            options.dock_offset,
        )
        write_database(options.output, dock_entry)
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        print(f'ERROR: Cannot generate the dock database: {error}', file=sys.stderr)
        raise SystemExit(1) from error
    print(f'Generated home_dock database: {options.output}')


if __name__ == '__main__':
    main()
