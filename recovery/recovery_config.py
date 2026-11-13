"""Public defaults and private configuration; never contains credentials."""
import ipaddress
import json
from pathlib import Path
import re


PROFILE = {
    'name': 'lite3-deeprcs-20250117-static-analysis',
    'motion_validated': False,
    'files': {
        'bin/jy_exe': '72b0bb6d9d9dc0448df9659879177ea4eec6ba4921fb31d212a0ad1e2fa57fe6',
        'lib/ras_lib/libdeepras.so': '8e47ebea2539bfa544a2be0e230a39bf946a512cbda1cb0f5483140b9bddd0dc',
    },
}
DEFAULTS = dict(ssh_host=None, ssh_key=None, ros_setup='/opt/ros/foxy/setup.bash',
    robot_workspace='~/lite3_recovery_ws', dds_profile=None, ros_domain_id=83,
    factory_root='~/jy_exe', factory_ip=None, factory_port=43893, telemetry_port=43897)


def validate_config(values):
    if not isinstance(values, dict) or set(values)-set(DEFAULTS):
        raise ValueError('Configuration must be an object containing only documented keys')
    result = dict(DEFAULTS, **values)
    host = result['ssh_host']
    if host is not None and (not isinstance(host, str) or
            not re.fullmatch(r'(?:[A-Za-z0-9_.-]+@)?[A-Za-z0-9_][A-Za-z0-9_.-]*', host)
            or host.startswith('-')):
        raise ValueError('Invalid SSH host or alias')
    key = result['ssh_key']
    if key is not None and (not isinstance(key, str) or not key or any(c in key for c in '\x00\r\n')):
        raise ValueError('Invalid SSH key path')
    for name in ('ros_setup', 'robot_workspace', 'factory_root', 'dds_profile'):
        value = result[name]
        if name == 'dds_profile' and value is None:
            continue
        if (not isinstance(value, str) or not value or any(c in value for c in '\x00\r\n')
                or not (value.startswith('/') or value.startswith('~/'))):
            raise ValueError(name+' must be an absolute robot path or start with ~/')
    for name in ('factory_port', 'telemetry_port', 'ros_domain_id'):
        maximum = 232 if name == 'ros_domain_id' else 65535
        minimum = 0 if name == 'ros_domain_id' else 1
        if type(result[name]) is not int or not minimum <= result[name] <= maximum:
            raise ValueError('Invalid '+name)
    if result['factory_port'] == result['telemetry_port']:
        raise ValueError('Command and receive ports must differ')
    if result['factory_ip'] is not None:
        try:
            address = ipaddress.IPv4Address(result['factory_ip'])
        except (ValueError, TypeError) as exc:
            raise ValueError('factory_ip must be an IPv4 address') from exc
        if address.is_unspecified or address.is_multicast or address == ipaddress.IPv4Address('255.255.255.255'):
            raise ValueError('factory_ip must identify one robot interface')
        result['factory_ip'] = str(address)
    return result


def load_config(path=None, host=None, key=None):
    values = {} if path is None else json.loads(Path(path).read_text(encoding='utf8'))
    result = validate_config(values)
    if host is not None:
        result['ssh_host'] = host
    if key is not None:
        result['ssh_key'] = key
    return validate_config(result)
