import hashlib
import importlib
import math
from pathlib import Path
import struct
import sys
from types import ModuleType, SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

from lite3_recovery_telemetry.protocol import HEADER, STATE, decode_state
from lite3_recovery_telemetry.profile import FILES
from recovery_config import PROFILE


def packet(battery=90., ultrasound=(math.nan, math.inf)):
    values = [99, 0, 179., 0., 0., .1, .2, .3, 0., 0., 9.81,
              0., 0., .33, 0., 0., 0., 0., 0., 0.,
              0, False, 0, 0, battery, 0, False, True, *ultrasound]
    return HEADER.pack(0x0901, 200, 1)+STATE.pack(*values)


def test_receiver_and_recovery_use_identical_known_firmware_fingerprints():
    assert FILES == PROFILE['files']


def test_decoder_preserves_inversion_and_battery_without_needing_sonar():
    state = decode_state(packet())
    assert state.rpy_deg == (179., 0., 0.) and state.battery == 90.
    assert math.isnan(state.ultrasound[0])


@pytest.mark.parametrize('data', [packet()[:-1], packet()+b'0', HEADER.pack(0x906, 200, 1)+packet()[12:],
    HEADER.pack(0x901, 199, 1)+packet()[12:], HEADER.pack(0x901, 200, 0)+packet()[12:]])
def test_decoder_rejects_wrong_abi(data):
    with pytest.raises(ValueError):
        decode_state(data)


def test_decoder_rejects_invalid_boolean_and_nonfinite_pose():
    raw = bytearray(packet())
    raw[12+180] = 3
    with pytest.raises(ValueError):
        decode_state(raw)
    raw = bytearray(packet())
    struct.pack_into('<d', raw, 12+8, math.nan)
    with pytest.raises(ValueError):
        decode_state(raw)


@pytest.fixture
def receiver(monkeypatch, tmp_path):
    world = SimpleNamespace(now=100., queue=[], sockets=[], publishers={}, occupied=False)
    root = tmp_path/'factory'
    files = {'bin/jy_exe': b'fake-controller', 'lib/ras_lib/libdeepras.so': b'fake-library'}
    for name, data in files.items():
        path = root/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    (root/'conf').mkdir()
    (root/'conf/network.toml').write_text('ip="192.0.2.10"\nlocal_port=43893\n')
    world.root = root
    world.parameters = dict(bind_ip='192.0.2.10', source_ip='192.0.2.10', receive_port=43897,
                            factory_root=str(root))
    class Publisher:
        def __init__(self):
            self.messages = []
        def publish(self, message):
            self.messages.append(message)
    class Node:
        def __init__(self, name):
            self.name = name
        def declare_parameter(self, key, value):
            pass
        def get_parameter(self, key):
            return SimpleNamespace(value=world.parameters[key])
        def create_publisher(self, kind, topic, qos):
            world.publishers[topic] = Publisher()
            return world.publishers[topic]
        def create_timer(self, *args):
            pass
        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 'robot-local-stamp'))
        def destroy_node(self):
            pass
    class Imu:
        def __init__(self):
            self.header = SimpleNamespace()
            self.orientation = SimpleNamespace()
            self.orientation_covariance = [0.]*9
            self.angular_velocity_covariance = [0.]*9
            self.linear_acceleration_covariance = [0.]*9
    class Array:
        def __init__(self):
            self.header = SimpleNamespace()
    class Status(SimpleNamespace):
        OK, ERROR = 0, 2
    classes = {'rclpy': {}, 'rclpy.node': dict(Node=Node),
        'rclpy.qos': dict(qos_profile_sensor_data=object()),
        'sensor_msgs': {}, 'sensor_msgs.msg': dict(Imu=Imu),
        'diagnostic_msgs': {}, 'diagnostic_msgs.msg': dict(DiagnosticArray=Array,
            DiagnosticStatus=Status, KeyValue=SimpleNamespace)}
    for name, values in classes.items():
        module = ModuleType(name)
        module.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.delitem(sys.modules, 'lite3_recovery_telemetry.receiver', raising=False)
    module = importlib.import_module('lite3_recovery_telemetry.receiver')
    monkeypatch.setattr(module, 'FILES', {name: hashlib.sha256(data).hexdigest() for name, data in files.items()})
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: world.now))
    class Socket:
        def __init__(self):
            self.closed = False
            world.sockets.append(self)
        def bind(self, address):
            if world.occupied:
                raise OSError('Already occupied')
            self.address = address
        def setblocking(self, value):
            pass
        def recvfrom(self, size):
            if not world.queue:
                raise BlockingIOError()
            return world.queue.pop(0)
        def close(self):
            self.closed = True
        def sendto(self, *args):
            pytest.fail('Receiver must NEVER send factory UDP commands')
    monkeypatch.setattr(module.socket, 'socket', lambda *a: Socket())
    world.module = module
    yield world
    sys.modules.pop('lite3_recovery_telemetry.receiver', None)


def test_receive_only_node_publishes_required_imu_and_diagnostics(receiver):
    world = receiver
    node = world.module.StateReceiver()
    world.queue.append((packet(), ('192.0.2.10', 1234)))
    node.receive()
    node.report()
    imu = world.publishers['/lite3/imu'].messages[-1]
    assert imu.header.frame_id == 'base_link' and imu.header.stamp == 'robot-local-stamp'
    assert abs(imu.orientation.x) > .99
    status = world.publishers['/lite3/native/diagnostics'].messages[-1].status[0]
    values = {item.key: item.value for item in status.values}
    assert values['protocol_verified'] == 'True' and values['clock_ready'] == 'False'
    assert values['valid_packets'] == '1' and values['basic_state'] == '99'
    assert set(world.publishers) == {'/lite3/imu', '/lite3/native/diagnostics'}
    node.destroy_node()
    assert world.sockets[0].closed


def test_receiver_rejects_foreign_or_malformed_data_and_reports_staleness(receiver):
    world = receiver
    node = world.module.StateReceiver()
    world.queue += [(packet(), ('192.0.2.11', 1234)), (packet()[:-1], ('192.0.2.10', 1234))]
    node.receive()
    node.report()
    assert node.foreign == node.invalid == 1 and node.valid == 0
    assert not world.publishers['/lite3/imu'].messages
    assert world.publishers['/lite3/native/diagnostics'].messages[-1].status[0].level == 2


def test_receiver_fingerprint_mismatch_never_opens_socket(receiver):
    (receiver.root/'bin/jy_exe').write_bytes(b'unknown-firmware')
    with pytest.raises(ValueError, match='fingerprint'):
        receiver.module.StateReceiver()
    assert not receiver.sockets


def test_receiver_rejects_wrong_interface_or_command_port(receiver):
    receiver.parameters['receive_port'] = 43893
    with pytest.raises(ValueError, match='fingerprint'):
        receiver.module.StateReceiver()
    assert not receiver.sockets


def test_occupied_port_is_not_stolen_and_socket_is_closed(receiver):
    receiver.occupied = True
    with pytest.raises(OSError):
        receiver.module.StateReceiver()
    assert receiver.sockets[0].closed


def test_receiver_package_has_no_gateway_entrypoint():
    root = Path(__file__).resolve().parents[1]/'ros2_ws/src/lite3_recovery_telemetry'
    xml = ET.parse(root/'package.xml').getroot()
    assert xml.findtext('name') == 'lite3_recovery_telemetry'
    setup = (root/'setup.py').read_text()
    assert 'native_state =' in setup and 'gateway' not in setup
    receiver_source = (root/'lite3_recovery_telemetry/receiver.py').read_text()
    assert '.sendto(' not in receiver_source and 'SO_REUSEADDR' not in receiver_source
