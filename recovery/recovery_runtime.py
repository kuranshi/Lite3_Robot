"""Robot-local ROS observer and explicitly requested supervised recovery trial."""
import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import select
import socket
import subprocess
import time
from recovery_config import validate_config

from recovery_core import (PROFILE, TARGET, FallDetector, Sample, ManualConfirmation,
                           one_shot, telemetry_reasons, packets, plan, normalize_imu_stamp,
                           MAX_AGE)

WAIT = 'Robot is lose power, wait a moment to press [STAND UP BUTTON] to recover the posture!!!'
SUCCESS = 'Robot recovering posture succeed'
FACTORY_ROOT = Path('~/jy_exe').expanduser()
LOG_ROOT = FACTORY_ROOT/'log'
NETWORK = FACTORY_ROOT/'conf/network.toml'
LOCK = '/tmp/lite3-native-commission.lock'
LEDGER = Path('~/.lite3-recovery-attempts').expanduser()


def configure(values):
    """Expand robot paths only AFTER the source has arrived on the robot."""
    global FACTORY_ROOT, LOG_ROOT, NETWORK, TARGET, PROFILE
    config = validate_config(values)
    if config['factory_ip'] is None:
        raise ValueError('Configure factory_ip from the robot network.toml')
    FACTORY_ROOT = Path(config['factory_root']).expanduser()
    LOG_ROOT = FACTORY_ROOT/'log'
    NETWORK = FACTORY_ROOT/'conf/network.toml'
    TARGET = (config['factory_ip'], config['factory_port'])
    from recovery_config import PROFILE as template
    PROFILE = dict(template, files={str(FACTORY_ROOT/name): digest
                                   for name, digest in template['files'].items()})


def system_check():
    hashes, reasons = {}, []
    for relative, expected in PROFILE['files'].items():
        name = str(FACTORY_ROOT/relative)
        try:
            hashes[name] = hashlib.sha256(Path(name).read_bytes()).hexdigest()
        except OSError:
            hashes[name] = None
        if hashes[name] != expected:
            reasons.append('firmware_fingerprint_mismatch:'+name)
    try:
        config = NETWORK.read_text()
        ip = re.search(r"(?m)^\s*ip\s*=\s*['\"]([^'\"]+)['\"]", config)
        port = re.search(r'(?m)^\s*local_port\s*=\s*(\d+)', config)
        if not ip or not port or (ip[1], int(port[1])) != TARGET:
            reasons.append('factory_endpoint_mismatch')
    except OSError:
        reasons.append('factory_network_config_unreadable')
    result = subprocess.run(['ps', '-eo', 'pid=,args='], capture_output=True, text=True, check=True)
    factory, conflicting = [], []
    for line in result.stdout.splitlines():
        fields = line.strip().split(maxsplit=1)
        if len(fields) != 2:
            continue
        pid, command = fields
        words = command.split()
        if words[0] in (str(FACTORY_ROOT/'bin/jy_exe'), str(FACTORY_ROOT/'bin/backup/deeprcs')):
            factory.append(int(pid))
        names = {Path(word).name for word in words[:3]}
        if names.intersection({'rl_deploy', 'lite3_transfer', 'native_gateway'}):
            conflicting.append(int(pid))
    if len(factory) != 1:
        reasons.append('need_one_factory_controller')
    if conflicting:
        reasons.append('conflicting_controller_process')
    return dict(hashes=hashes, factory_pids=factory, conflicting_pids=conflicting,
                target=list(TARGET), reasons=reasons, motion_validated=False)


def parse_log_time(line):
    match = re.match(r'\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d)\.(\d{3})\.(\d{3})\]', line)
    if not match:
        return None
    # Interpret on the robot, where these local timestamps were generated.
    return datetime.datetime.strptime(match[1]+'.'+match[2]+match[3], '%Y-%m-%d %H:%M:%S.%f').timestamp()


class FactoryLog:
    """Read current boot's dated log, ignoring historical recovery successes.

    Execution requires a waiting event actually observed AFTER this observer
    started. Starting the tool on an already-fallen robot never accepts an old
    log line as an authorization. This is intentionally stricter than inspect.
    """
    def __init__(self, root=None):
        self.root = LOG_ROOT if root is None else Path(root)
        self.path = None
        self.identity = None
        self.offset = 0
        self.partial = b''
        self.waiting = None
        self.wait_since = None
        self.estop_latched = False
        self.stop_generation = 0
        self.fault_latched = False
        self.started = False
        self.recovery_started = False
        self.direction_seen = None
        self.succeeded = False
        self.failed = False

    def poll(self, now, wall):
        paths = list(self.root.glob('*/deeprcs.*.log'))
        if not paths:
            raise RuntimeError('Factory event log not found')
        path = max(paths, key=lambda p: p.stat().st_mtime_ns)
        stat = path.stat()
        identity = (str(path), stat.st_dev, stat.st_ino)
        if identity != self.identity or stat.st_size < self.offset:
            self.waiting = self.wait_since = None
            if self.started:
                raise RuntimeError('Factory log changed or was truncated; restart observation')
            self.path, self.identity, self.offset = path, identity, stat.st_size
            self.started = True
            return []
        with path.open('rb') as handle:
            handle.seek(self.offset)
            chunk = handle.read(131072)
        if len(chunk) == 131072:
            self.waiting = None
            raise RuntimeError('Factory log reader fell behind')
        self.offset += len(chunk)
        pieces = (self.partial+chunk).split(b'\n')
        self.partial = pieces.pop()
        events = []
        for raw in pieces:
            line = raw.decode('utf8', 'replace').replace('\x00', '')
            stamp = parse_log_time(line)
            if stamp is None or not -.05 <= wall-stamp <= 2.:
                continue
            if '[ERROR' in line or '[FATAL' in line or 'recovering posture failed' in line:
                self.fault_latched = True
                self.failed = True
                self.waiting = None
                events.append('factory_fault')
            elif 'Soft Emergency Pressed' in line:
                self.estop_latched = True
                self.stop_generation += 1
                self.waiting = None
                events.append('soft_emergency_observed')
            elif WAIT in line:
                self.waiting = hashlib.sha256((repr(identity)+line).encode()).hexdigest()
                self.wait_since = now
                self.recovery_started = False
                self.direction_seen = None
                self.succeeded = False
                events.append('factory_waiting_for_recovery')
            elif 'Robot is recovering posture...' in line:
                self.waiting = None
                self.recovery_started = True
                events.append('factory_recovery_started')
            elif 'Recover Left!!!' in line or 'Recover Right!!!' in line:
                self.direction_seen = 'left' if 'Recover Left!!!' in line else 'right'
                events.append('factory_direction_'+self.direction_seen)
            elif SUCCESS in line:
                self.waiting = None
                self.succeeded = True
                events.append('factory_reported_success')
            elif any(word in line for word in ('stand_cmd', 'stand_down', 'Stand up action',
                       'Stand down action', 'Switching to force control', 'JY is Moving',
                       'Robot is lose control', 'Robot is waitting to stand up')):
                self.waiting = None
                events.append('factory_state_changed')
        return events

    def reasons(self, now, acknowledgement=None):
        result = []
        if not self.waiting or self.wait_since is None or not 0 <= now-self.wait_since <= 30.:
            result.append('need_new_factory_recovery_wait_event')
        # An explicit interactive acknowledgement only scopes THIS recovery
        # request. The latch stays set, and no estop-clear packet is generated.
        if self.estop_latched and acknowledgement != (self.waiting, self.stop_generation):
            result.append('soft_emergency_observed')
        if self.fault_latched:
            result.append('factory_fault_observed')
        return result

    def confirmation_context(self, direction, now):
        reasons = self.reasons(now)
        blocked = [r for r in reasons if r != 'soft_emergency_observed']
        if blocked:
            return None
        return (self.waiting, direction, self.stop_generation, self.estop_latched, self.wait_since+30.)


class OperatorInput:
    """Bounded JSON-line input over SSH stdin; EOF cancels pending requests."""
    def __init__(self, fd=0):
        self.fd = fd
        self.buffer = b''

    def poll(self):
        if not select.select([self.fd], [], [], 0)[0]:
            return []
        data = os.read(self.fd, 4096)
        if not data:
            raise RuntimeError('Operator input channel closed; pending confirmation cancelled')
        self.buffer += data
        if len(self.buffer) > 4096:
            raise RuntimeError('Operator input exceeded protocol limit')
        lines = self.buffer.split(b'\n')
        self.buffer = lines.pop()
        messages = []
        for line in lines:
            try:
                message = json.loads(line)
            except (ValueError, UnicodeDecodeError):
                message = None
            messages.append(message)
        return messages


class Observer:
    def __init__(self, hold_s=1.):
        import rclpy
        from sensor_msgs.msg import Imu
        from diagnostic_msgs.msg import DiagnosticArray
        from rclpy.qos import qos_profile_sensor_data
        self.rclpy = rclpy
        rclpy.init(args=[])
        self.node = rclpy.create_node('lite3_recovery_observer')
        self.detector = FallDetector(hold_s=hold_s)
        self.native, self.native_received = {}, -math.inf
        self.last_native_counter = -1
        self.valid_imu = 0
        self.invalid_imu = 0
        self.candidate_events = 0
        self.imu_source_ns = None
        self.imu_clock_mode = None
        self.imu_clock_uncertainty = 0.
        self.imu_timing_error = 'No IMU source stamp received'
        self.imu_sub = self.node.create_subscription(Imu, '/lite3/imu', self.imu, qos_profile_sensor_data)
        self.diag_sub = self.node.create_subscription(DiagnosticArray, '/lite3/native/diagnostics', self.diag, 1)

    def imu(self, msg):
        now = time.monotonic()
        if msg.header.frame_id != 'base_link' or msg.orientation_covariance[0] < 0:
            self.invalid_imu += 1
            self.detector.history.clear()
            self.detector.candidate = False
            return
        q = msg.orientation
        stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        # Both observer and receiver run on the robot. Undo the optional PC
        # offset before checking source age and advancing stamps. Do not start
        # a navigation stack or publish a fabricated global clock correction.
        try:
            stamp, mode, uncertainty = normalize_imu_stamp(stamp, time.time(), self.native)
            if stamp <= self.detector.last_stamp:
                return  # Duplicate or out-of-order input never refreshes age.
            event = self.detector.update(Sample(stamp, now, (q.x, q.y, q.z, q.w)), now)
            self.imu_source_ns, self.imu_clock_mode = stamp, mode
            self.imu_clock_uncertainty = uncertainty
            self.imu_timing_error = None
            self.valid_imu += 1
            self.candidate_events += int(event)
        except (ValueError, TypeError) as exc:
            self.imu_timing_error = str(exc)
            self.invalid_imu += 1
            self.detector.history.clear()
            self.detector.candidate = False

    def diag(self, msg):
        for status in msg.status:
            if status.name != 'lite3/native':
                continue
            native = {v.key: v.value for v in status.values}
            self.native = native
            try:
                counter = int(native.get('valid_packets', '-1'))
            except ValueError:
                return
            if counter > self.last_native_counter:
                self.last_native_counter = counter
                self.native_received = time.monotonic()

    def spin(self, duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=min(.01, max(0., end-time.monotonic())))

    def snapshot(self):
        now = time.monotonic()
        source_age = None if self.imu_source_ns is None else time.time()-self.imu_source_ns*1e-9
        source_valid = (source_age is not None and math.isfinite(source_age)
            and -self.imu_clock_uncertainty <= source_age <= MAX_AGE-self.imu_clock_uncertainty)
        local_verified = (source_valid and self.imu_clock_mode == 'robot_local'
                          and self.native.get('clock_ready') == 'False')
        reasons = telemetry_reasons(self.detector, self.native, self.native_received, now,
                                    local_clock_verified=local_verified)
        if not source_valid:
            reasons.append('imu_source_timestamp_stale_or_invalid')
        if self.node.count_publishers('/lite3/imu') != 1 or self.node.count_publishers('/lite3/native/diagnostics') != 1:
            reasons.append('need_exactly_one_native_telemetry_publisher')
        return dict(tilt_deg=self.detector.last_tilt, fallen_candidate=self.detector.candidate,
                    valid_imu_callbacks=self.valid_imu, invalid_imu_callbacks=self.invalid_imu,
                    candidate_events=self.candidate_events, native=self.native, reasons=reasons,
                    imu_clock_mode=self.imu_clock_mode, imu_source_age_s=source_age,
                    imu_timing_error=self.imu_timing_error)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def reserve(event):
    LEDGER.mkdir(mode=0o700, exist_ok=True)
    with (LEDGER/event).open('x') as handle:
        handle.write('Reserved before sending; never automatically retry this event.\n')
        handle.flush()
        os.fsync(handle.fileno())
    directory = os.open(str(LEDGER), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def run(options):
    if options.get('mode') == 'auto':
        from recovery_auto import run_auto
        return run_auto(options)
    started = time.monotonic()
    def emit(event, **data):
        print(json.dumps(dict(event=event, elapsed_s=round(time.monotonic()-started, 3), **data),
                         allow_nan=False, ensure_ascii=False), flush=True)

    execute = options.get('execute', False)
    interactive = options.get('interactive', False)
    if execute and (options['mode'] != 'manual' or not interactive or not all(options.get(k) for k in
            ('operator_ready', 'clearance_confirmed'))):
        raise RuntimeError('Real trials need an interactive supervised manual session')
    if interactive and options['mode'] != 'manual':
        raise RuntimeError('Interactive recovery is available only in manual mode')
    observer = lock = sock = None
    sent = False
    try:
        system = system_check()
        emit('system', **system)
        if options['mode'] == 'manual':
            emit('request_plan', direction=options['direction'], requests=plan(options['direction']),
                 packet_hex=[p.hex() for p in packets(options['direction'])],
                 dry_run=not execute, motion_validated=False)
        if execute and system['reasons']:
            raise RuntimeError('System check refused: '+','.join(system['reasons']))
        observer = Observer()
        factory_log = FactoryLog()
        factory_log.poll(time.monotonic(), time.time())
        emit('observing', interactive=interactive, execute=execute,
             waiting_for_new_factory_event=True, sends_on_startup=False)
        confirmation = ManualConfirmation()
        operator = OperatorInput() if interactive else None
        if execute:
            lock = open(LOCK, 'a')
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        last_report, previous_candidates = -math.inf, 0
        deadline = time.monotonic()+options['duration']

        def pause(seconds):
            observer.spin(seconds)
            for item in factory_log.poll(time.monotonic(), time.time()):
                emit(item)

        def check_ready(accepted):
            pause(.02)
            pending = operator.poll()
            if any(m == {'action': 'cancel'} for m in pending):
                raise RuntimeError('Operator cancelled before sending')
            for _ in pending:
                emit('confirmation_rejected', error='Confirmation was already consumed')
            now = time.monotonic()
            snapshot = observer.snapshot()
            acknowledgement = (accepted.waiting_event, accepted.stop_generation)
            reasons = snapshot['reasons']+factory_log.reasons(now, acknowledgement)
            if now >= accepted.expires_at:
                reasons.append('operator_confirmation_expired')
            if (factory_log.waiting, factory_log.stop_generation) != acknowledgement:
                reasons.append('factory_waiting_event_or_soft_stop_changed')
            if reasons:
                raise RuntimeError('Recovery refused: '+','.join(reasons))
            return factory_log.waiting

        while time.monotonic() < deadline:
            pause(.02)
            snapshot = observer.snapshot()
            reasons = system['reasons']+snapshot['reasons']+factory_log.reasons(time.monotonic())
            if observer.candidate_events != previous_candidates:
                previous_candidates = observer.candidate_events
                emit('fall_candidate_alert', automatic_action=False, tilt_deg=snapshot['tilt_deg'])
            if time.monotonic()-last_report >= 1.:
                emit('observation', **snapshot, factory_reasons=factory_log.reasons(time.monotonic()),
                     execute=execute, automatic_action=False)
                last_report = time.monotonic()
            if not interactive:
                continue

            now = time.monotonic()
            context = factory_log.confirmation_context(options['direction'], now)
            previous = confirmation.offer
            offered = confirmation.sync(context, now, bool(system['reasons']+snapshot['reasons']))
            if previous and previous is not confirmation.offer:
                emit('confirmation_invalidated', nonce=previous.nonce,
                     reason='readiness_changed_or_confirmation_expired')
            if offered:
                emit('confirmation_ready', **offered.public(now), dry_run=not execute,
                     instruction='确认机器狗已倒地、所选方向空地和急停条件仍满足后，输入显示的完整口令；CANCEL 退出。')
            messages = operator.poll()
            if any(m == {'action': 'cancel'} for m in messages):
                raise RuntimeError('Operator cancelled; no additional commands')
            accepted = None
            for message in messages:
                try:
                    accepted = confirmation.consume(message,
                        (factory_log.waiting, factory_log.stop_generation), time.monotonic())
                except ValueError as exc:
                    emit('confirmation_rejected', error=str(exc))
                    continue
                break
            if accepted is None:
                continue
            emit('operator_confirmed', waiting_event=accepted.waiting_event,
                 direction=accepted.direction, acknowledged_soft_stop=accepted.resume_soft_stop,
                 dry_run=not execute)
            check_ready(accepted)
            if not execute:
                emit('confirmation_dry_run_complete', direction=accepted.direction,
                     sent_motion_packets=False, motion_validated=False,
                     requires_new_confirmation_for_real_trial=True)
                return 0
            if execute:
                # Recheck the disk profile and process ownership just before the
                # only transaction, then refresh ROS callbacks before each send.
                current = system_check()
                if current['reasons'] or current['factory_pids'] != system['factory_pids']:
                    raise RuntimeError('Controller or firmware changed during observation')
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.bind((TARGET[0], 0))
                def transmit(data):
                    nonlocal sent
                    sent = True
                    if sock.sendto(data, TARGET) != len(data):
                        raise OSError('Short UDP send; result unknown, no retry')
                one_shot(options['direction'], lambda: check_ready(accepted), reserve, transmit, pause, emit)
                end = time.monotonic()+10.
                upright_since = None
                while time.monotonic() < end:
                    pause(.02)
                    snapshot = observer.snapshot()
                    if factory_log.failed or factory_log.stop_generation != accepted.stop_generation:
                        raise RuntimeError('Factory reported fault/emergency during recovery')
                    if any(m == {'action': 'cancel'} for m in operator.poll()):
                        raise RuntimeError('Operator stopped observation; factory action may continue')
                    if factory_log.direction_seen and factory_log.direction_seen != options['direction']:
                        raise RuntimeError('Factory direction differs from requested direction')
                    fresh = 0 <= time.monotonic()-observer.detector.last_received <= .2
                    upright = fresh and observer.detector.last_tilt is not None and observer.detector.last_tilt <= 20.
                    upright_since = (time.monotonic() if upright_since is None else upright_since) if upright else None
                    if (factory_log.recovery_started and factory_log.succeeded
                            and factory_log.direction_seen == options['direction']
                            and upright_since is not None and time.monotonic()-upright_since >= .5):
                        emit('factory_success_and_upright_observed', standing_verified=False, physical_confirmation_required=True)
                        return 0
                raise RuntimeError('Recovery outcome unknown after 10 seconds; no retry, check robot')
        snapshot = observer.snapshot()
        reasons = system['reasons']+snapshot['reasons']+factory_log.reasons(time.monotonic())
        emit('finished', sent_motion_packets=sent, execute=execute,
             ready_for_trial=not reasons, reasons=reasons, snapshot=snapshot,
             motion_validated=False)
        return 2 if execute or observer.valid_imu == 0 else 0
    except (Exception, KeyboardInterrupt) as exc:
        emit('stopped', error=str(exc) or 'interrupted', sent_motion_packets=sent,
             retry_allowed=False, factory_action_may_continue=sent)
        return 2
    finally:
        # No hidden stop/mode/heartbeat/stand packet on exit. Once requested,
        # recovery is executed autonomously by the factory controller.
        if sock is not None:
            sock.close()
        if lock is not None:
            lock.close()
        if observer is not None:
            observer.close()
