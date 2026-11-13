import ast
import json
from pathlib import Path
import shlex
import subprocess
import sys

import pytest

import lite3_recovery as cli
import recovery_runtime as rt
from recovery_config import DEFAULTS, PROFILE, load_config, validate_config


def test_defaults_never_select_a_real_host_key_or_factory_address():
    assert DEFAULTS['ssh_host'] is DEFAULTS['ssh_key'] is DEFAULTS['factory_ip'] is None


def test_config_and_command_line_precedence(tmp_path):
    path = tmp_path/'robot.local.json'
    path.write_text(json.dumps(dict(ssh_host='original', factory_ip='192.0.2.10', ssh_key='~/key')))
    config = load_config(path, host='robot', key='~/different-key')
    assert config['ssh_host'] == 'robot' and config['ssh_key'] == '~/different-key'
    assert config['factory_ip'] == '192.0.2.10'


@pytest.mark.parametrize('value', [[], {'unknown': 1}, {'ssh_host': '-oProxyCommand=bad'},
    {'ssh_host': 'robot; command'}, {'factory_ip': '0.0.0.0'}, {'factory_ip': '224.0.0.1'},
    {'factory_port': True}, {'telemetry_port': 43893}, {'factory_port': 0},
    {'ros_domain_id': 233}, {'robot_workspace': 'relative/path'}, {'factory_root': '~other/path'},
    {'dds_profile': 'bad\npath'}, {'ssh_key': ''}])
def test_invalid_configuration_is_rejected(value):
    with pytest.raises(ValueError):
        validate_config(value)


@pytest.mark.parametrize('args', [['inspect'], ['watch', '--host', 'robot'], ['auto', '--host', 'robot']])
def test_remote_modes_refuse_missing_connection_details_before_ssh(args, monkeypatch):
    monkeypatch.setattr(cli.subprocess, 'Popen', lambda *a, **kw: pytest.fail('Must not connect'))
    with pytest.raises(SystemExit):
        cli.main(args)


def test_remote_paths_are_not_expanded_to_the_computers_home():
    config = validate_config(dict(factory_ip='192.0.2.10', robot_workspace='~/robot ws'))
    text = cli.remote_command(config, 'pass')
    assert '"$HOME"/' in text and 'robot ws/install/setup.bash' in text
    assert str(Path.home()) not in text
    assert 'unset FASTRTPS_DEFAULT_PROFILES_FILE' in text


def test_shell_path_suffix_cannot_execute_substitution(tmp_path):
    marker = tmp_path/'should-not-exist'
    name = '~/space $(touch '+str(marker)+') `false`'
    command = 'printf %s '+cli.robot_shell_path(name)
    result = subprocess.run(['bash', '-c', command], capture_output=True, text=True, check=True)
    assert result.stdout.endswith(name[2:]) and not marker.exists()


def test_runtime_configuration_preserves_lock_and_resolves_paths_only_on_robot(monkeypatch, tmp_path):
    # Restore globals after this isolated configuration exercise.
    for name in ('FACTORY_ROOT', 'LOG_ROOT', 'NETWORK', 'TARGET', 'PROFILE'):
        monkeypatch.setattr(rt, name, getattr(rt, name))
    monkeypatch.setenv('HOME', str(tmp_path/'robot-home'))
    rt.configure(dict(factory_ip='192.0.2.10'))
    assert rt.FACTORY_ROOT == tmp_path/'robot-home/jy_exe'
    assert rt.TARGET == ('192.0.2.10', 43893)
    assert rt.LEDGER.name == '.lite3-recovery-attempts'
    assert rt.LOCK == '/tmp/lite3-native-commission.lock'
    assert all(str(rt.FACTORY_ROOT) in path for path in rt.PROFILE['files'])
    assert list(rt.PROFILE['files'].values()) == list(PROFILE['files'].values())


def test_bootstrap_carries_configuration_but_not_ssh_credentials():
    options = dict(mode='inspect', duration=1., config=dict(factory_ip='192.0.2.10'))
    code = cli.bootstrap(options)
    assert '.configure(' in code and '192.0.2.10' in code
    assert 'ssh_key' not in repr(options)
    compile(code, '<bootstrap>', 'exec')


def test_all_published_python_supports_robot_python38_syntax():
    root = Path(__file__).resolve().parents[1]
    for path in root.rglob('*.py'):
        if not any(part in ('build', 'install', '__pycache__') for part in path.parts):
            ast.parse(path.read_text(), filename=str(path), feature_version=(3, 8))


def test_receiver_helper_help_needs_neither_ros_nor_robot():
    path = Path(__file__).resolve().parents[1]/'scripts/receiver.py'
    result = subprocess.run([sys.executable, str(path), '--help'], capture_output=True, text=True)
    assert result.returncode == 0 and 'build' in result.stdout
