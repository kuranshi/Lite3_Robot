import datetime
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import pytest

from recovery_core import FallDetector, Sample, packets, plan, telemetry_reasons, one_shot, tilt_deg
from recovery_runtime import FactoryLog, WAIT, SUCCESS, parse_log_time
import lite3_recovery
import recovery_runtime


def q(roll, yaw=0):
    r, y = math.radians(roll)/2, math.radians(yaw)/2
    return (math.sin(r)*math.cos(y), math.sin(r)*math.sin(y),
            math.cos(r)*math.sin(y), math.cos(r)*math.cos(y))


def feed(detector, roll=100, start=0, n=61):
    events = []
    for i in range(n):
        now = start+i*.02
        events.append(detector.update(Sample(int(now*1e9)+1, now, q(roll)), now))
    return events


def test_direction_and_toggle_exact_wire_bytes():
    assert packets('left')[0].hex() == '31010121f7ceffff00000000'
    assert packets('right')[0].hex() == '310101210931000000000000'
    assert packets('left')[1].hex() == '020201210000000000000000'
    assert packets('left')[1] == packets('right')[1]
    with pytest.raises(ValueError):
        plan('auto')


@pytest.mark.parametrize('roll', [0, 10, 20, 45, 64])
def test_normal_or_moderate_tilt_never_requests_recovery(roll):
    detector = FallDetector()
    assert not any(feed(detector, roll))
    assert not detector.candidate


@pytest.mark.parametrize('roll', [80, 100, 180, -100, -180])
def test_side_or_inverted_candidate_once(roll):
    detector = FallDetector()
    assert sum(feed(detector, roll)) == 1
    assert detector.candidate
    assert not any(feed(detector, roll, start=1.22))


def test_yaw_does_not_hide_inversion_or_quaternion_sign():
    assert tilt_deg(q(180, 90)) == pytest.approx(180)
    assert tilt_deg(tuple(-x for x in q(100))) == pytest.approx(100)


@pytest.mark.parametrize('orientation', [(0,0,0,0), (0,0,0,2), (math.nan,0,0,1), (0,0,1)])
def test_invalid_orientation_rejected(orientation):
    with pytest.raises(ValueError):
        tilt_deg(orientation)


def test_duplicates_stale_and_time_gap_do_not_complete_hold():
    detector = FallDetector()
    assert not any(feed(detector, n=25))
    for i in range(50):
        now = .5+i*.02
        assert not detector.update(Sample(1, now, q(100)), now)
    assert not detector.candidate
    assert not detector.update(Sample(3_000_000_000, 1., q(100)), 3.)
    assert not any(feed(detector, start=4., n=30))
    assert not detector.candidate


def test_rotating_robot_does_not_count_as_settled():
    detector = FallDetector()
    for i in range(120):
        now = i*.02
        detector.update(Sample(i+1, now, q(90+i*.2)), now)
    assert not detector.candidate


def test_loss_and_reconnect_does_not_repeat_alert_until_upright():
    detector = FallDetector()
    assert sum(feed(detector)) == 1
    detector.expire(3.)
    assert not any(feed(detector, start=4.))
    assert not any(feed(detector, roll=0, start=5.22, n=120))
    assert sum(feed(detector, start=7.62)) == 1


def test_native_six_is_normal_not_fallen_and_invalid_battery_fails():
    detector = FallDetector()
    feed(detector)
    native = dict(protocol_verified='True', clock_ready='True', age_s='0.01', battery_raw='90', basic_state='6')
    assert 'factory_still_reports_normal_standing' in telemetry_reasons(detector, native, 1.2, 1.2)
    native.update(basic_state='unknown', battery_raw='nan')
    assert 'battery_not_above_20_percent' in telemetry_reasons(detector, native, 1.2, 1.2)
    assert 'native_diagnostics_missing_or_stale' in telemetry_reasons(detector, native, 0, 1.2)


def test_one_shot_exactly_two_no_cleanup_packets():
    sent, reserved, records = [], [], []
    one_shot('right', lambda: 'event1', reserved.append, sent.append, lambda _: None,
             lambda event, **data: records.append(event))
    assert sent == list(packets('right'))
    assert reserved == ['event1']
    assert records.index('attempt_reserved') < records.index('direction_sent')


def test_changed_state_between_packets_prevents_toggle():
    states = iter(['a', 'a', 'b'])
    sent = []
    with pytest.raises(RuntimeError, match='changed before stand'):
        one_shot('left', lambda: next(states), lambda _: None, sent.append, lambda _: None, lambda *a, **k: None)
    assert sent == [packets('left')[0]]


@pytest.mark.parametrize('failure_at', [1, 2])
def test_send_error_keeps_reservation_and_never_retries(failure_at):
    reserved, sent = [], []
    def send(packet):
        sent.append(packet)
        if len(sent) == failure_at:
            raise OSError('unknown delivery')
    with pytest.raises(OSError):
        one_shot('left', lambda: 'a', reserved.append, send, lambda _: None, lambda *a, **k: None)
    assert reserved == ['a']
    assert len(sent) == failure_at


def test_duplicate_attempt_refuses_all_packets(tmp_path, monkeypatch):
    monkeypatch.setattr(recovery_runtime, 'LEDGER', tmp_path/'attempts')
    recovery_runtime.reserve('a')
    sent = []
    with pytest.raises(FileExistsError):
        one_shot('left', lambda: 'a', recovery_runtime.reserve, sent.append, lambda _: None, lambda *a, **k: None)
    assert sent == []


def line(message, wall):
    stamp = datetime.datetime.fromtimestamp(wall).strftime('%Y-%m-%d %H:%M:%S.%f')
    stamp = stamp[:-3]+'.'+stamp[-3:]
    return '['+stamp+'] [INFO  ] [BasicMotion] '+message+'\n'


def log_fixture(tmp_path):
    directory = tmp_path/'2026_1112'
    directory.mkdir()
    path = directory/'deeprcs.2026_1112.log'
    wall = time.time()
    path.write_text(line(WAIT, wall)+line(SUCCESS, wall))
    log = FactoryLog(tmp_path)
    log.poll(0, wall)
    return log, path, wall


def test_only_new_wait_event_counts_and_success_is_not_replayed(tmp_path):
    log, path, wall = log_fixture(tmp_path)
    assert log.waiting is None and not log.succeeded
    with path.open('a') as f:
        f.write(line(WAIT, wall))
    assert log.poll(1, wall) == ['factory_waiting_for_recovery']
    assert not log.reasons(1)
    assert log.reasons(32)


def test_explicit_estop_cannot_be_cleared_by_wait_event(tmp_path):
    log, path, wall = log_fixture(tmp_path)
    with path.open('a') as f:
        f.write(line('Soft Emergency Pressed!!!', wall)+line(WAIT, wall))
    log.poll(1, wall)
    assert 'soft_emergency_observed' in log.reasons(1)


@pytest.mark.parametrize('message', ['stand_cmd....', 'Switching to force control mode', 'Robot is recovering posture...', 'Robot is lose control'])
def test_factory_transition_invalidates_waiting(tmp_path, message):
    log, path, wall = log_fixture(tmp_path)
    with path.open('a') as f:
        f.write(line(WAIT, wall)+line(message, wall))
    log.poll(1, wall)
    assert log.waiting is None


def test_stale_log_and_log_rotation_never_authorize(tmp_path):
    log, path, wall = log_fixture(tmp_path)
    with path.open('a') as f:
        f.write(line(WAIT, wall-5))
    log.poll(1, wall)
    assert log.waiting is None
    path.write_text('')
    with pytest.raises(RuntimeError, match='changed or was truncated'):
        log.poll(2, wall)


def test_new_wait_discards_previous_recovery_result(tmp_path):
    log, path, wall = log_fixture(tmp_path)
    with path.open('a') as f:
        f.write(line('Robot is recovering posture...', wall)+line('Recover Left!!!', wall)+line(SUCCESS, wall))
    log.poll(1, wall)
    assert log.recovery_started and log.succeeded and log.direction_seen == 'left'
    with path.open('a') as f:
        f.write(line(WAIT, wall))
    log.poll(1.1, wall)
    assert log.waiting and not log.recovery_started and not log.succeeded and log.direction_seen is None


def test_parse_log_timestamp_handles_factory_millisecond_microsecond_separator():
    wall = time.time()
    assert abs(parse_log_time(line(WAIT, wall))-wall) < .000002


@pytest.mark.parametrize('argv', [
    ['plan', '--direction', 'left', '--execute'],
    ['watch', '--execute'],
    ['manual', '--direction', 'left', '--execute'],
    ['watch', '--duration', 'nan'],
    ['watch', '--duration', 'inf'],
    ['manual'],
])
def test_invalid_cli_refuses_before_ssh(argv, monkeypatch):
    monkeypatch.setattr(subprocess, 'Popen', lambda *a, **k: pytest.fail('Must not connect'))
    with pytest.raises(SystemExit) as err:
        lite3_recovery.main(argv)
    assert err.value.code == 2


def test_plan_is_completely_offline(monkeypatch, capsys):
    monkeypatch.setattr(subprocess, 'Popen', lambda *a, **k: pytest.fail('Must not connect'))
    assert lite3_recovery.main(['plan', '--direction', 'right']) == 0
    value = json.loads(capsys.readouterr().out)
    assert not value['sends_packets']
    assert not value['profile']['motion_validated']


def test_streamed_bootstrap_does_not_write_or_create_sender():
    script = lite3_recovery.bootstrap({'mode': 'inspect', 'execute': False})
    compile(script, '<bootstrap>', 'exec')
    assert 'sys.dont_write_bytecode = True' in script


def test_passive_runtime_never_creates_command_socket_or_reserves_attempt(monkeypatch, capsys):
    class Observer:
        valid_imu = 10
        candidate_events = 0
        def spin(self, duration):
            time.sleep(duration)
        def snapshot(self):
            return dict(reasons=['need_stable_fallen_orientation'], tilt_deg=0)
        def close(self):
            pass
    class Log:
        def poll(self, *args):
            return []
        def reasons(self, *args):
            return ['need_new_factory_recovery_wait_event']
    monkeypatch.setattr(recovery_runtime, 'Observer', Observer)
    monkeypatch.setattr(recovery_runtime, 'FactoryLog', Log)
    monkeypatch.setattr(recovery_runtime, 'system_check', lambda: dict(reasons=[]))
    monkeypatch.setattr(recovery_runtime.socket, 'socket', lambda *a, **k: pytest.fail('No UDP sender allowed'))
    monkeypatch.setattr(recovery_runtime, 'reserve', lambda *a: pytest.fail('No reservation allowed'))
    assert recovery_runtime.run(dict(mode='manual', execute=False, duration=.05, direction='left')) == 0
    events = [json.loads(s) for s in capsys.readouterr().out.splitlines()]
    assert events[-1]['sent_motion_packets'] is False


def test_runtime_refuses_auto_execution_even_if_called_without_cli():
    with pytest.raises(RuntimeError):
        recovery_runtime.run(dict(mode='watch', execute=True, operator_ready=True,
                                 clearance_confirmed=True, fallen_confirmed=True))


def test_wrong_firmware_stops_before_ros_or_sender(monkeypatch, capsys):
    monkeypatch.setattr(recovery_runtime, 'system_check', lambda: dict(reasons=['firmware_fingerprint_mismatch']))
    monkeypatch.setattr(recovery_runtime, 'Observer', lambda: pytest.fail('Must refuse before ROS'))
    monkeypatch.setattr(recovery_runtime.socket, 'socket', lambda *a, **k: pytest.fail('No sender'))
    result = recovery_runtime.run(dict(mode='manual', direction='left', execute=True, interactive=True,
                                      operator_ready=True, clearance_confirmed=True))
    assert result == 2
    assert json.loads(capsys.readouterr().out.splitlines()[-1])['sent_motion_packets'] is False


def test_existing_audit_refuses_before_ssh(tmp_path, monkeypatch):
    audit = tmp_path/'previous.jsonl'
    audit.write_text('previous trial\n')
    monkeypatch.setattr(subprocess, 'Popen', lambda *a, **k: pytest.fail('No SSH'))
    config = tmp_path/'robot.local.json'
    config.write_text(json.dumps(dict(ssh_host='robot', factory_ip='192.0.2.10')))
    assert lite3_recovery.main(['inspect', '--config', str(config), '--audit', str(audit)]) == 2
    assert audit.read_text() == 'previous trial\n'


def test_live_request_gate_failure_sends_nothing(monkeypatch, tmp_path, capsys):
    class Observer:
        valid_imu = 10
        candidate_events = 0
        def spin(self, duration):
            time.sleep(duration)
        def snapshot(self):
            return dict(reasons=['need_stable_fallen_orientation'], tilt_deg=0)
        def close(self):
            pass
    class Log:
        def poll(self, *args):
            return []
        def reasons(self, *args):
            return ['need_new_factory_recovery_wait_event']
        def confirmation_context(self, *args):
            return None
    monkeypatch.setattr(recovery_runtime, 'Observer', Observer)
    monkeypatch.setattr(recovery_runtime, 'FactoryLog', Log)
    monkeypatch.setattr(recovery_runtime, 'OperatorInput', lambda: type('Input', (), {'poll': lambda self: []})())
    monkeypatch.setattr(recovery_runtime, 'LOCK', str(tmp_path/'sender.lock'))
    monkeypatch.setattr(recovery_runtime, 'system_check', lambda: dict(reasons=[]))
    monkeypatch.setattr(recovery_runtime.socket, 'socket', lambda *a, **k: pytest.fail('No UDP sender allowed'))
    monkeypatch.setattr(recovery_runtime, 'reserve', lambda *a: pytest.fail('No reservation allowed'))
    assert recovery_runtime.run(dict(mode='manual', execute=True, interactive=True, duration=.05, direction='left',
                                    operator_ready=True, clearance_confirmed=True)) == 2
    events = [json.loads(s) for s in capsys.readouterr().out.splitlines()]
    assert events[-1]['sent_motion_packets'] is False
