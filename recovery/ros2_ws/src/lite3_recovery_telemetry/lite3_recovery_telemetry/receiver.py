"""Receive-only telemetry: no factory command socket, gateway or clock writer.

Modified from the native bridge design: only IMU and native diagnostics remain.
See THIRD_PARTY_NOTICES.md for the extracted decoder's MIT attribution.
"""
import hashlib
import ipaddress
import math
from pathlib import Path
import re
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu
from .profile import FILES
from .protocol import decode_state


def verified_profile(root, bind_ip, receive_port):
    """Do not mark a decoder successful merely because its layout parses."""
    root = Path(root).expanduser()
    try:
        if any(hashlib.sha256((root/name).read_bytes()).hexdigest() != digest
               for name, digest in FILES.items()):
            return False
        config = (root/'conf/network.toml').read_text()
        ip = re.search(r"(?m)^\s*ip\s*=\s*['\"]([^'\"]+)['\"]", config)
        command = re.search(r'(?m)^\s*local_port\s*=\s*(\d+)', config)
        if not ip or not command or ip[1] != bind_ip or int(command[1]) == receive_port:
            return False
    except OSError:
        return False
    return True


def orientation(rpy_deg):
    roll, pitch, yaw = (math.radians(v)/2 for v in rpy_deg)
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


class StateReceiver(Node):
    def __init__(self):
        super().__init__('lite3_recovery_native_state')
        for key, value in [('bind_ip', ''), ('source_ip', ''), ('receive_port', 43897),
                           ('factory_root', '~/jy_exe')]:
            self.declare_parameter(key, value)
        p = lambda name: self.get_parameter(name).value
        for name in ('bind_ip', 'source_ip'):
            address = ipaddress.IPv4Address(p(name))
            if address.is_unspecified or address.is_multicast:
                raise ValueError('Use an explicit robot '+name)
        if type(p('receive_port')) is not int or not 1 <= p('receive_port') <= 65535:
            raise ValueError('Invalid receive port')
        self.verified = verified_profile(p('factory_root'), p('bind_ip'), p('receive_port'))
        if not self.verified:
            raise ValueError('Firmware/network fingerprint mismatch; receive-only adapter refused')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # Exclusive bind: never share or steal the existing native receiver.
            self.sock.bind((p('bind_ip'), p('receive_port')))
            self.sock.setblocking(False)
        except Exception:
            self.sock.close()
            raise
        self.source_ip = p('source_ip')
        self.last, self.state = -math.inf, None
        self.valid = self.invalid = self.foreign = 0
        self.last_rejection = ''
        self.imu = self.create_publisher(Imu, '/lite3/imu', qos_profile_sensor_data)
        self.diag = self.create_publisher(DiagnosticArray, '/lite3/native/diagnostics', 1)
        self.create_timer(.01, self.receive)
        self.create_timer(.05, self.report)

    def receive(self):
        latest = None
        for _ in range(256):
            try:
                packet, peer = self.sock.recvfrom(65535)
            except BlockingIOError:
                break
            if peer[0] != self.source_ip:
                self.foreign += 1
                continue
            if packet[:4] != b'\x01\x09\x00\x00':
                continue
            try:
                latest = decode_state(packet)
                self.valid += 1
            except ValueError as exc:
                self.invalid += 1
                self.last_rejection = str(exc)
        if latest is None:
            return
        self.last, self.state = time.monotonic(), latest
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'
        for key, value in zip(('x', 'y', 'z', 'w'), orientation(latest.rpy_deg)):
            setattr(imu.orientation, key, value)
        imu.orientation_covariance = [.001, 0., 0., 0., .001, 0., 0., 0., .003]
        imu.angular_velocity_covariance[0] = imu.linear_acceleration_covariance[0] = -1.
        self.imu.publish(imu)

    def report(self):
        age = time.monotonic()-self.last
        values = dict(valid_packets=self.valid, rejected_packets=self.invalid,
            foreign_packets=self.foreign, last_rejection=self.last_rejection,
            age_s=age, protocol_verified=self.verified, clock_ready=False, clock_correction=None)
        if self.state is not None:
            values.update(basic_state=self.state.basic, gait=self.state.gait, battery_raw=self.state.battery)
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(name='lite3/native', hardware_id='Lite3',
            level=DiagnosticStatus.OK if age <= .2 else DiagnosticStatus.ERROR,
            message='robot-local telemetry; no cross-host correction' if age <= .2 else 'native telemetry stale')
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        message.status = [status]
        self.diag.publish(message)

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = StateReceiver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
