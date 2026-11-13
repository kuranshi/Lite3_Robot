"""Synthetic robot clock, real detector/log/ledger, fake UDP; no robot access."""
import json
import math
import os
import subprocess
import sys
import threading
from types import SimpleNamespace

import pytest

import lite3_recovery as cli
import recovery_auto as auto
import recovery_runtime as rt
from recovery_core import FallDetector, Sample, packets, telemetry_reasons
from test_recovery import line, q


@pytest.fixture
def robot(monkeypatch, tmp_path):
    world = SimpleNamespace(now=0., wall=946684800., sends=[], send_times=[],
        events=[(.04, 'Soft Emergency Pressed!!!'), (.08, rt.WAIT)],
        source_paused=lambda t: False, bad_native=lambda t: False,
        roll=lambda t: 179., cancel_at=math.inf, disconnect_at=math.inf,
        stop_between=False, fail_send=None, recover_success=True,
        motion_starts=[], completed=set(), second_fall=math.inf, closed=False)
    directory = tmp_path/'2026_1112'
    directory.mkdir()
    logfile = directory/'deeprcs.2026_1112.log'
    logfile.touch()
    def append(text):
        with logfile.open('a') as handle:
            handle.write(line(text, world.wall+world.now))
    def step(seconds):
        world.now += seconds
        for at, message in list(world.events):
            if world.now >= at:
                append(message)
                world.events.remove((at, message))
        for index, start in enumerate(world.motion_starts):
            if world.recover_success and world.now-start >= 4.6 and index not in world.completed:
                append(rt.SUCCESS)
                world.completed.add(index)
    def pose():
        if world.motion_starts:
            latest = len(world.motion_starts)-1
            if latest in world.completed and (latest > 0 or world.now < world.second_fall):
                return 0.
        return world.roll(world.now)
    class Observer:
        def __init__(self, hold_s=1.):
            self.detector = FallDetector(hold_s)
            self.valid_imu = 0
            world.observer = self
        def spin(self, seconds):
            step(seconds)
            if not world.source_paused(world.now):
                self.detector.update(Sample(int(world.now*1e9)+1, world.now, q(pose())), world.now)
                self.valid_imu += 1
        def snapshot(self):
            native = dict(protocol_verified='True', clock_ready='True', age_s='0.001',
                          battery_raw='90', basic_state='6' if pose() <= 20. else '99')
            if world.bad_native(world.now):
                native['clock_ready'] = 'False'
            reasons = telemetry_reasons(self.detector, native, world.now, world.now)
            return dict(reasons=reasons, native=native, tilt_deg=self.detector.last_tilt)
        def close(self):
            world.closed = True
    class Input:
        def poll(self):
            if world.now >= world.disconnect_at:
                raise RuntimeError('Operator input channel closed')
            if world.now >= world.cancel_at:
                return [{'action': 'cancel'}]
            return []
    class Socket:
        def bind(self, target):
            world.bound = target
        def sendto(self, data, target):
            world.sends.append(data)
            world.send_times.append(world.now)
            if world.fail_send == len(world.sends):
                raise OSError('Unknown delivery')
            if len(world.sends) % 2 == 1 and world.stop_between:
                append('Soft Emergency Pressed!!!')
            if len(world.sends) % 2 == 0:
                world.motion_starts.append(world.now)
                append('stand_cmd....')
                append('Robot is recovering posture...')
                append('Recover Left!!!')
            return len(data)
        def close(self):
            pass
    clock = SimpleNamespace(monotonic=lambda: world.now, time=lambda: world.wall+world.now)
    monkeypatch.setattr(rt, 'time', clock)
    real_log = rt.FactoryLog(tmp_path)
    monkeypatch.setattr(rt, 'FactoryLog', lambda: real_log)
    monkeypatch.setattr(rt, 'Observer', Observer)
    monkeypatch.setattr(rt, 'OperatorInput', Input)
    monkeypatch.setattr(rt.socket, 'socket', lambda *a, **k: Socket())
    monkeypatch.setattr(rt, 'LOCK', str(tmp_path/'sender.lock'))
    monkeypatch.setattr(rt, 'LEDGER', tmp_path/'attempts')
    monkeypatch.setattr(rt, 'system_check', lambda: dict(reasons=[], factory_pids=[625]))
    world.options = dict(mode='auto', direction='left', duration=16., fall_policy='any',
                         execute=True, audit_enabled=True)
    world.log = real_log
    world.ledger = tmp_path/'attempts'
    return world


def rows(capsys):
    return [json.loads(s) for s in capsys.readouterr().out.splitlines()]


def test_default_any_soft_stop_sends_once_after_five_seconds_and_rearms(robot, capsys):
    assert rt.run(robot.options) == 0
    assert robot.sends == list(packets('left'))
    # Includes the initial qualification; not 1 second plus another 5.
    assert 5. <= robot.send_times[0] < 5.3
    assert .1 <= robot.send_times[1]-robot.send_times[0] < .2
    events = rows(capsys)
    assert any(r['event'] == 'auto_recovery_complete' for r in events)
    assert any(r['event'] == 'auto_rearmed' for r in events)
    assert any(r['event'] == 'auto_observation' and r['phase'] == 'recovering' for r in events)
    assert robot.log.estop_latched  # Acknowledgement did not clear an estop.
    assert len(list(robot.ledger.iterdir())) == 1 and robot.closed


def test_dry_run_never_opens_socket_or_writes_ledger(robot, monkeypatch, capsys):
    robot.options['execute'] = False
    monkeypatch.setattr(rt.socket, 'socket', lambda *a: pytest.fail('Dry run opened socket'))
    assert rt.run(robot.options) == 0
    assert rows(capsys)[-1]['event'] == 'auto_dry_run_ready'
    assert not robot.sends and not robot.ledger.exists()


def test_natural_only_never_infers_cause_from_absent_stop_log(robot, capsys):
    robot.events = [(.08, rt.WAIT)]
    robot.options.update(execute=False, fall_policy='natural-only')
    assert rt.run(robot.options) == 0
    events = rows(capsys)
    assert any('natural_fall_source_unavailable' in r.get('factory_reasons', []) for r in events)
    assert not any(r['event'] == 'auto_trigger_ready' for r in events)
    assert not robot.sends and not robot.ledger.exists()


@pytest.mark.parametrize('change', [dict(fall_policy='natural-only'), dict(audit_enabled=False),
    dict(direction='right'), dict(interactive=True), dict(duration=float('nan'))])
def test_runtime_rejects_unsupported_or_unaudited_execution(robot, change):
    robot.options.update(change)
    with pytest.raises((RuntimeError, ValueError)):
        rt.run(robot.options)
    assert not robot.sends and not robot.ledger.exists()


def test_rotating_posture_does_not_complete_five_seconds(robot, capsys):
    robot.roll = lambda t: 80.+t*2.
    assert rt.run(robot.options) == 2
    assert not robot.sends
    assert not any(r['event'] == 'auto_trigger_ready' for r in rows(capsys))


def test_upright_interruption_resets_entire_hold(robot, capsys):
    robot.roll = lambda t: 0. if 3. <= t < 3.2 else 179.
    assert rt.run(robot.options) == 0
    assert 8.2 <= robot.send_times[0] < 8.5


def test_imu_gap_resets_entire_hold(robot, capsys):
    robot.source_paused = lambda t: 2. <= t < 2.4
    assert rt.run(robot.options) == 0
    assert 7.4 <= robot.send_times[0] < 7.7


def test_invalid_native_data_resets_hold_even_if_imu_keeps_arriving(robot, capsys):
    robot.bad_native = lambda t: 2. <= t < 2.4
    assert rt.run(robot.options) == 0
    assert 7.4 <= robot.send_times[0] < 7.7


def test_new_stop_during_countdown_restarts_five_seconds(robot, capsys):
    robot.events += [(2., 'Soft Emergency Pressed!!!'), (2.04, rt.WAIT)]
    assert rt.run(robot.options) == 0
    assert 7. <= robot.send_times[0] < 7.4


def test_new_stop_between_packets_blocks_toggle_without_retry(robot, capsys):
    robot.stop_between = True
    assert rt.run(robot.options) == 2
    assert robot.sends == [packets('left')[0]]
    assert len(list(robot.ledger.iterdir())) == 1
    assert rows(capsys)[-1]['sent_motion_packets'] is True


@pytest.mark.parametrize('kind', ['cancel_at', 'disconnect_at'])
def test_cancel_or_disconnect_before_trigger_never_sends(robot, kind, capsys):
    setattr(robot, kind, 4.9)
    assert rt.run(robot.options) == 2
    assert not robot.sends and not robot.ledger.exists()
    assert rows(capsys)[-1]['event'] == 'stopped'


def test_factory_fault_cannot_be_bypassed_by_any_policy(robot, capsys):
    robot.events += [(4., '[ERROR ] [RT-CAN] send error')]
    assert rt.run(robot.options) == 2
    assert not robot.sends and not robot.ledger.exists()
    assert 'fault' in rows(capsys)[-1]['error'].lower()


def test_new_stop_after_stand_send_aborts_monitor_without_second_attempt(robot, capsys):
    robot.events += [(5.5, 'Soft Emergency Pressed!!!'), (5.6, rt.WAIT)]
    assert rt.run(robot.options) == 2
    assert robot.sends == list(packets('left'))
    assert rows(capsys)[-1]['factory_action_may_continue'] is True


def test_controller_change_before_send_refuses_all_packets(robot, monkeypatch, capsys):
    monkeypatch.setattr(rt, 'system_check', lambda: dict(reasons=[],
        factory_pids=[625] if robot.now < 1. else [626]))
    assert rt.run(robot.options) == 2
    assert not robot.sends and not robot.ledger.exists()


def test_factory_wait_is_required_even_with_stable_inversion(robot, capsys):
    robot.events = [(.04, 'Soft Emergency Pressed!!!')]
    assert rt.run(robot.options) == 2
    assert not robot.sends and not robot.ledger.exists()


def test_normal_pose_never_triggers(robot, capsys):
    robot.roll = lambda t: 0.
    assert rt.run(robot.options) == 2
    assert not robot.sends


@pytest.mark.parametrize('missing', ['imu', 'diagnostics'])
def test_no_telemetry_exits_after_five_seconds_without_sender(robot, monkeypatch, capsys, missing):
    if missing == 'imu':
        robot.source_paused = lambda t: True
    else:
        original = rt.Observer.snapshot
        def missing_diagnostics(self):
            snapshot = original(self)
            snapshot['native'] = {}
            return snapshot
        monkeypatch.setattr(rt.Observer, 'snapshot', missing_diagnostics)
    monkeypatch.setattr(rt.socket, 'socket', lambda *a: pytest.fail('No sender without telemetry'))
    assert rt.run(robot.options) == 2
    assert 5. <= robot.now < 5.1
    assert not robot.sends and not robot.ledger.exists()
    result = rows(capsys)
    assert result[-2]['event'] == 'telemetry_unavailable'
    assert result[-1]['sent_motion_packets'] is False


def test_new_wait_during_failed_recovery_is_not_a_second_attempt(robot, capsys):
    robot.recover_success = False
    robot.events += [(7., rt.WAIT)]
    assert rt.run(robot.options) == 2
    assert robot.sends == list(packets('left'))
    assert 'no retry' in rows(capsys)[-1]['error'].lower()


@pytest.mark.parametrize('index', [1, 2])
def test_unknown_send_result_keeps_durable_reservation(robot, capsys, index):
    robot.fail_send = index
    assert rt.run(robot.options) == 2
    assert len(robot.sends) == index and len(list(robot.ledger.iterdir())) == 1
    assert rows(capsys)[-1]['retry_allowed'] is False


def test_persistent_reservation_refusal_prevents_all_sends(robot, monkeypatch, capsys):
    def duplicate(event):
        raise FileExistsError('Already attempted')
    monkeypatch.setattr(rt, 'reserve', duplicate)
    assert rt.run(robot.options) == 2
    assert not robot.sends


def test_second_fall_after_success_and_normal_rearm_gets_one_new_attempt(robot, capsys):
    robot.second_fall = 14.5
    robot.events += [(14.6, rt.WAIT)]
    robot.options['duration'] = 30.
    assert rt.run(robot.options) == 0
    assert robot.sends == list(packets('left'))*2
    assert robot.send_times[2] >= 19.5
    assert len(list(robot.ledger.iterdir())) == 2
    assert rows(capsys)[-1]['successes'] == 2


@pytest.mark.parametrize('args', [
    ['auto', '--execute'],
    ['auto', '--direction', 'right'],
    ['auto', '--interactive'],
    ['auto', '--execute', '--fall-policy', 'natural-only', '--audit', '/tmp/unused-auto.jsonl'],
])
def test_cli_invalid_auto_options_refuse_before_ssh(args, monkeypatch):
    monkeypatch.setattr(cli.subprocess, 'Popen', lambda *a, **k: pytest.fail('No SSH allowed'))
    with pytest.raises(SystemExit):
        cli.main(args)


def test_auto_cli_needs_no_clearance_or_confirmation_flags_and_keeps_control_pipe(monkeypatch, tmp_path):
    captured = {}
    def bootstrap(options):
        captured.update(options)
        return 'pass'
    class Pipe:
        closed = False
        def close(self):
            self.closed = True
    process = SimpleNamespace(stdin=Pipe(), stdout=Pipe(), wait=lambda **kw: 0)
    monkeypatch.setattr(cli, 'bootstrap', bootstrap)
    monkeypatch.setattr(cli.subprocess, 'Popen', lambda *a, **kw: process)
    def stream(proc, audit, interactive, direction, automatic=False):
        assert not proc.stdin.closed and not interactive and automatic and direction == 'left'
        return 0
    monkeypatch.setattr(cli, 'stream_session', stream)
    audit = tmp_path/'auto.jsonl'
    config = tmp_path/'robot.local.json'
    config.write_text(json.dumps(dict(ssh_host='robot', factory_ip='192.0.2.10')))
    assert cli.main(['auto', '--execute', '--config', str(config), '--audit', str(audit)]) == 0
    assert captured['fall_policy'] == 'any' and captured['audit_enabled']
    assert not captured['clearance_confirmed'] and not captured['operator_ready']
    assert not captured['interactive']
    assert json.loads(audit.read_text())['mode'] == 'auto'


def test_streamed_bootstrap_loads_auto_module_without_installing_it():
    code = cli.bootstrap(dict(mode='auto', direction='left', execute=False, duration=10.))
    compile(code, '<auto-bootstrap>', 'exec')
    assert 'recovery_auto' in code and 'sys.modules["recovery_runtime"].run' in code


def test_auto_control_pipe_cancels_without_any_confirmation(monkeypatch, tmp_path, capsys):
    """Real local client/server pipe round trip; no SSH or robot socket."""
    configuration = dict(event='auto_configuration', fall_policy='any', dry_run=False)
    server = ('import sys,json\n'
        f'print(json.dumps({configuration!r}),flush=True)\n'
        'assert json.loads(sys.stdin.readline())=={"action":"cancel"}\n'
        'print(json.dumps(dict(event="stopped",sent_motion_packets=False,error="cancelled")),flush=True)\n')
    ready = threading.Event()
    original = cli.AutoDisplay.handle
    def handle(self, event):
        original(self, event)
        if event.get('event') == 'auto_configuration':
            ready.set()
    monkeypatch.setattr(cli.AutoDisplay, 'handle', handle)
    reader, writer = os.pipe()
    monkeypatch.setattr(cli.sys, 'stdin', SimpleNamespace(fileno=lambda: reader, isatty=lambda: True))
    def type_cancel():
        if ready.wait(2.):
            os.write(writer, b'CANCEL\n')
        os.close(writer)
    thread = threading.Thread(target=type_cancel)
    process = subprocess.Popen([sys.executable, '-c', server], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    path = tmp_path/'auto-control.jsonl'
    try:
        thread.start()
        with path.open('x') as audit:
            assert cli.stream_session(process, audit, False, 'left', automatic=True) == 0
    finally:
        thread.join(timeout=3.)
        os.close(reader)
        if process.poll() is None:
            process.kill()
        process.wait(timeout=2.)
        process.stdin.close()
        process.stdout.close()
    recorded = [json.loads(s) for s in path.read_text().splitlines()]
    assert recorded[1] == dict(event='local_operator_input', message={'action': 'cancel'})
    assert not any(r['event'] == 'confirmation_ready' for r in recorded)
    terminal = capsys.readouterr().out
    assert '真实自动恢复' in terminal and '没有发送运动指令' in terminal
    assert '"event":' not in terminal
