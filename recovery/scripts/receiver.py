#!/usr/bin/env python3
"""Run ON THE ROBOT: build, start or inspect the receive-only ROS adapter."""
import argparse
import os
from pathlib import Path
import shlex
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from recovery_config import load_config


def command(task, config):
    workspace = Path(config['robot_workspace']).expanduser()
    commands = ['source '+shlex.quote(str(Path(config['ros_setup']).expanduser())),
        'export ROS_DOMAIN_ID='+str(config['ros_domain_id'])+' ROS_LOCALHOST_ONLY=0']
    commands.append('export FASTRTPS_DEFAULT_PROFILES_FILE='+shlex.quote(str(Path(config['dds_profile']).expanduser()))
                    if config['dds_profile'] else 'unset FASTRTPS_DEFAULT_PROFILES_FILE')
    if task == 'build':
        sources = ROOT/'ros2_ws/src'
        commands += ['mkdir -p '+shlex.quote(str(workspace)), 'cd '+shlex.quote(str(workspace)),
                     'exec colcon build --base-paths '+shlex.quote(str(sources))]
    else:
        commands.append('source '+shlex.quote(str(workspace/'install/setup.bash')))
        if task == 'start':
            ip = config['factory_ip']
            if not ip:
                raise ValueError('Set factory_ip from the verified factory network.toml')
            args = ['ros2', 'run', 'lite3_recovery_telemetry', 'native_state', '--ros-args',
                    '-p', 'bind_ip:='+ip, '-p', 'source_ip:='+ip,
                    '-p', 'receive_port:='+str(config['telemetry_port']),
                    '-p', 'factory_root:='+str(Path(config['factory_root']).expanduser())]
            commands.append('exec '+' '.join(shlex.quote(v) for v in args))
        else:
            commands += ['ros2 topic info /lite3/imu', 'ros2 topic info /lite3/native/diagnostics']
    return ' && '.join(commands)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('task', choices=['build', 'start', 'status'])
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    try:
        value = command(args.task, load_config(args.config))
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    os.execvp('bash', ['bash', '-c', value])


if __name__ == '__main__':
    main()
