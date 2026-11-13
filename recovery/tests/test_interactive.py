"""Offline tests only: fake telemetry/controllers, pipe input and temporary logs."""
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
from types import SimpleNamespace

import pytest

from recovery_core import ManualConfirmation, packets
from recovery_runtime import FactoryLog, OperatorInput, WAIT, SUCCESS, parse_log_time
from lite3_recovery import TerminalRelay, InteractiveDisplay
import lite3_recovery
import recovery_runtime
from test_recovery import line


def message(offer):
    return dict(action='confirm', nonce=offer.nonce, waiting_event=offer.waiting_event,
                direction=offer.direction, phrase=offer.phrase)


def offer(soft_stop=True):
    confirmation = ManualConfirmation()
    current = confirmation.sync(('event1', 'left', 2, soft_stop, 40.), 10., False)
    return confirmation, current


def test_soft_stop_needs_explicit_event_scoped_phrase():
    confirmation, current = offer()
    assert current.phrase.startswith('RESUME LEFT ')
    assert current.expires_at == 25.
    assert confirmation.consume(message(current), ('event1', 2), 11.) == current
    with pytest.raises(ValueError, match='repeated'):
        confirmation.consume(message(current), ('event1', 2), 11.1)


def test_no_stop_uses_normal_phrase():
    _, current = offer(False)
    assert current.phrase.startswith('RECOVER LEFT ')


@pytest.mark.parametrize('field,value', [('nonce', 'wrong'), ('direction', 'right'),
    ('waiting_event', 'event2'), ('phrase', 'RECOVER LEFT'), ('action', 'cancel')])
def test_changed_confirmation_never_authorizes(field, value):
    confirmation, current = offer()
    body = message(current)
    body[field] = value
    with pytest.raises(ValueError):
        confirmation.consume(body, ('event1', 2), 11.)
    assert confirmation.offer is None


@pytest.mark.parametrize('context', [('event2', 2), ('event1', 3), (None, 2)])
def test_new_wait_or_new_stop_revokes_previous_intent(context):
    confirmation, current = offer()
    with pytest.raises(ValueError, match='changed'):
        confirmation.consume(message(current), context, 11.)


@pytest.mark.parametrize('now', [25., 30., float('nan'), float('inf')])
def test_expired_or_invalid_confirmation_time_refused(now):
    confirmation, current = offer()
    with pytest.raises(ValueError, match='expired'):
        confirmation.consume(message(current), ('event1', 2), now)


def test_blocked_readiness_invalidates_offer_and_cannot_be_preconfirmed():
    confirmation, current = offer()
    confirmation.sync(('event1', 'left', 2, True, 40.), 11., True)
    with pytest.raises(ValueError, match='No current'):
        confirmation.consume(message(current), ('event1', 2), 12.)
    new = confirmation.sync(('event1', 'left', 2, True, 40.), 13., False)
    assert new.nonce != current.nonce


def test_expiry_refresh_has_new_nonce_and_wait_deadline_caps_lifetime():
    confirmation, current = offer()
    assert confirmation.sync(('event1', 'left', 2, True, 40.), 12., False) is None
    new = confirmation.sync(('event1', 'left', 2, True, 40.), 25., False)
    assert new.nonce != current.nonce and new.expires_at == 40.
    assert confirmation.sync(('event1', 'left', 2, True, 40.), 40., False) is None
    assert confirmation.offer is None


def test_terminal_relay_rejects_early_input_wrong_phrase_and_replay():
    relay = TerminalRelay('left', lambda: 10.)
    _, current = offer()
    with pytest.raises(ValueError):
        relay.submit(current.phrase)
    relay.receive(dict(event='confirmation_ready', **current.public(10.)))
    with pytest.raises(ValueError):
        relay.submit('left')
    relay.receive(dict(event='confirmation_ready', **current.public(10.)))
    assert relay.submit(current.phrase) == message(current)
    with pytest.raises(ValueError):
        relay.submit(current.phrase)


def test_terminal_relay_tracks_invalidation_direction_and_deadline():
    clock = [10.]
    relay = TerminalRelay('left', lambda: clock[0])
    _, current = offer()
    public = dict(event='confirmation_ready', **current.public(10.))
    relay.receive(dict(public, direction='right'))
    assert relay.offer is None
    relay.receive(public)
    relay.receive(dict(event='confirmation_invalidated', nonce=current.nonce))
    assert relay.offer is None
    relay.receive(public)
    clock[0] = 25.
    with pytest.raises(ValueError):
        relay.submit(current.phrase)
    assert relay.submit('CANCEL') == {'action': 'cancel'}


def test_operator_pipe_handles_partial_frames_malformed_json_and_disconnect():
    reader, writer = os.pipe()
    try:
        channel = OperatorInput(reader)
        os.write(writer, b'{"action":')
        assert channel.poll() == []
        os.write(writer, b'"cancel"}\nbad json\n')
        assert channel.poll() == [{'action': 'cancel'}, None]
        os.close(writer)
        writer = None
        with pytest.raises(RuntimeError, match='closed'):
            channel.poll()
    finally:
        os.close(reader)
        if writer is not None:
            os.close(writer)


def test_interactive_cli_cannot_be_confirmed_by_pipe(monkeypatch, tmp_path):
    monkeypatch.setattr(lite3_recovery.sys.stdin, 'isatty', lambda: False)
    monkeypatch.setattr(lite3_recovery.subprocess, 'Popen', lambda *a, **k: pytest.fail('Must not connect'))
    with pytest.raises(SystemExit):
        lite3_recovery.main(['manual', '--direction', 'left', '--interactive'])
    with pytest.raises(SystemExit):
        lite3_recovery.main(['manual', '--direction', 'left', '--execute', '--operator-ready',
                             '--clearance-confirmed', '--audit', str(tmp_path/'never.jsonl')])
    assert not (tmp_path/'never.jsonl').exists()


def test_terminal_stream_round_trip_preserves_one_time_confirmation(monkeypatch, tmp_path, capsys):
    """Real local process/pipe round trip, with a fake server and NO network."""
    _, current = offer()
    public = dict(event='confirmation_ready', **current.public(10.), dry_run=True)
    expected = message(current)
    server = (
        'import sys,json\n'
        f'raw=json.dumps({public!r})+"\\n"\n'
        'sys.stdout.write(raw[:20]);sys.stdout.flush()\n'
        'sys.stdout.write(raw[20:]);sys.stdout.flush()\n'
        'reply=json.loads(sys.stdin.readline())\n'
        f'assert reply=={expected!r},reply\n'
        'print(json.dumps(dict(event="confirmation_dry_run_complete",sent_motion_packets=False)),flush=True)\n'
    )
    ready = threading.Event()
    class Relay(TerminalRelay):
        def receive(self, event):
            super().receive(event)
            if self.offer:
                ready.set()
    monkeypatch.setattr(lite3_recovery, 'TerminalRelay', Relay)
    reader, writer = os.pipe()
    monkeypatch.setattr(lite3_recovery.sys, 'stdin', SimpleNamespace(fileno=lambda: reader))
    def type_phrase():
        if ready.wait(2.):
            os.write(writer, (current.phrase+'\n').encode())
        os.close(writer)
    thread = threading.Thread(target=type_phrase)
    process = subprocess.Popen([sys.executable, '-c', server], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    audit_path = tmp_path/'audit.jsonl'
    try:
        thread.start()
        with audit_path.open('x') as audit:
            assert lite3_recovery.stream_session(process, audit, True, 'left') == 0
    finally:
        thread.join(timeout=3.)
        os.close(reader)
        if process.poll() is None:
            process.kill()
        process.wait(timeout=2.)
        if not process.stdin.closed:
            process.stdin.close()
        process.stdout.close()
    rows = [json.loads(s) for s in audit_path.read_text().splitlines()]
    assert rows[1] == dict(event='local_operator_input', message=expected)
    assert rows[-1]['sent_motion_packets'] is False
    terminal = capsys.readouterr().out
    assert '现在可在下面输入' in terminal
    assert current.phrase in terminal
    assert '"event":' not in terminal  # Raw JSON remains in the audit only.


def test_confirmation_stays_visible_during_continuous_telemetry(capsys):
    display = InteractiveDisplay('left')
    _, current = offer()
    display.handle(dict(event='confirmation_ready', **current.public(10.), dry_run=False))
    prompt = capsys.readouterr().out
    assert '真实动作请求' in prompt and current.phrase in prompt
    assert '当前终端' in prompt and 'Enter' in prompt
    for _ in range(200):
        display.handle(dict(event='observation', reasons=['soft_emergency_observed'],
                            factory_reasons=[], tilt_deg=179.))
    assert capsys.readouterr().out == ''
    assert display.waiting_nonce == current.nonce


def test_repeated_waiting_status_does_not_scroll_terminal(capsys):
    clock = [10.]
    display = InteractiveDisplay('left', lambda: clock[0])
    event = dict(event='observation', reasons=['need_stable_fallen_orientation'], factory_reasons=[])
    for i in range(100):
        clock[0] = 10.+i
        display.handle(event)
    assert capsys.readouterr().out.count('[等待]') == 1


def test_expiry_clearly_revokes_visible_phrase(capsys):
    display = InteractiveDisplay('left')
    _, current = offer()
    display.handle(dict(event='confirmation_ready', **current.public(10.), dry_run=False))
    capsys.readouterr()
    display.handle(dict(event='confirmation_invalidated', nonce=current.nonce))
    assert display.waiting_nonce is None
    assert '口令已失效' in capsys.readouterr().out


def test_fault_visible_immediately_even_while_prompt_is_shown(capsys):
    display = InteractiveDisplay('left')
    _, current = offer()
    display.handle(dict(event='confirmation_ready', **current.public(10.), dry_run=False))
    capsys.readouterr()
    display.handle(dict(event='factory_fault'))
    assert display.waiting_nonce is None
    assert '原厂错误记录' in capsys.readouterr().out


@pytest.mark.parametrize('sent', [False, True])
def test_session_end_does_not_claim_factory_motion_was_stopped(capsys, sent):
    display = InteractiveDisplay('left')
    display.handle(dict(event='stopped', sent_motion_packets=sent))
    text = capsys.readouterr().out
    assert ('原厂动作可能继续' in text) if sent else ('没有发送动作' in text)


def test_synthetic_factory_sequence_requires_new_confirmation_after_each_transition(tmp_path):
    directory = tmp_path/'2026_1112'
    directory.mkdir()
    path = directory/'deeprcs.2026_1112.log'
    path.touch()
    entries = []
    baseline = Path(__file__).parent/'fixtures'/'factory_sequence.txt'
    for raw in baseline.read_text().splitlines():
        stamp = parse_log_time(raw)
        if stamp is not None:
            entries.append((stamp, raw))
    start = entries[0][0]
    log, confirmation = FactoryLog(tmp_path), ManualConfirmation()
    log.poll(0., start)
    offers = []
    for stamp, raw in entries:
        with path.open('a') as handle:
            handle.write(raw+'\n')
        now = stamp-start
        log.poll(now, stamp)
        current = confirmation.sync(log.confirmation_context('left', now), now, False)
        if current:
            assert 'soft_emergency_observed' in log.reasons(now)
            assert current.resume_soft_stop and current.phrase.startswith('RESUME LEFT ')
            assert not log.reasons(now, (current.waiting_event, current.stop_generation))
            offers.append(current)
    assert len(offers) == 2 and offers[0].nonce != offers[1].nonce
    assert offers[0].waiting_event != offers[1].waiting_event
    assert confirmation.offer is None  # APP action consumed the factory waiting state.
    assert log.succeeded and log.direction_seen == 'left'
    assert not log.confirmation_context('left', entries[-1][0]-start)


@pytest.fixture
def runtime_simulator(monkeypatch, tmp_path):
    """Production runtime/log/parser, simulated operator and telemetry, fake UDP."""
    world = SimpleNamespace(now=1000., wall=946684800., started=1000., offer=None,
        accepted=False, confirm=False, cancel=False, stop_after_confirm=False,
        stop_after_direction=False, stopped=False, sends=[], soft_sent=False, wait_sent=False)
    directory = tmp_path/'2026_1112'
    directory.mkdir()
    logfile = directory/'deeprcs.2026_1112.log'
    logfile.touch()
    real_log = FactoryLog(tmp_path)
    world.log = real_log
    clock = SimpleNamespace(monotonic=lambda: world.now, time=lambda: world.wall+world.now-world.started)
    monkeypatch.setattr(recovery_runtime, 'time', clock)
    def append(text):
        with logfile.open('a') as handle:
            handle.write(line(text, clock.time()))
    def step(seconds):
        world.now += seconds
        if not world.soft_sent and world.now-world.started >= .04:
            append('Soft Emergency Pressed!!!')
            world.soft_sent = True
        if not world.wait_sent and world.now-world.started >= .08:
            append(WAIT)
            world.wait_sent = True
        if world.accepted and world.stop_after_confirm and not world.stopped:
            append('Soft Emergency Pressed!!!')
            append(WAIT)
            world.stopped = True
        if world.sends and world.stop_after_direction and not world.stopped:
            append('Soft Emergency Pressed!!!')
            world.stopped = True
    class Observer:
        valid_imu = 10
        candidate_events = 1
        detector = SimpleNamespace(last_received=1000., last_tilt=179.03845691261057)
        def spin(self, seconds):
            step(seconds)
            self.detector.last_received = world.now
            if len(world.sends) == 2:
                self.detector.last_tilt = 0.
        def snapshot(self):
            return dict(reasons=[], tilt_deg=self.detector.last_tilt)
        def close(self):
            pass
    class Confirmation(ManualConfirmation):
        def sync(self, *args):
            new = super().sync(*args)
            if new:
                world.offer = new
            return new
    class Input:
        def poll(self):
            if world.offer and world.confirm and not world.accepted:
                world.accepted = True
                return [message(world.offer)]+([{'action': 'cancel'}] if world.cancel else [])
            return []
    class Socket:
        def bind(self, target):
            world.bound = target
        def sendto(self, data, target):
            world.sends.append(data)
            if len(world.sends) == 2:
                append('stand_cmd....')
                append('Robot is recovering posture...')
                append('Recover Left!!!')
                append(SUCCESS)
            return len(data)
        def close(self):
            pass
    monkeypatch.setattr(recovery_runtime, 'Observer', Observer)
    monkeypatch.setattr(recovery_runtime, 'FactoryLog', lambda: real_log)
    monkeypatch.setattr(recovery_runtime, 'ManualConfirmation', Confirmation)
    monkeypatch.setattr(recovery_runtime, 'OperatorInput', Input)
    monkeypatch.setattr(recovery_runtime, 'LOCK', str(tmp_path/'sender.lock'))
    monkeypatch.setattr(recovery_runtime, 'LEDGER', tmp_path/'attempts')
    monkeypatch.setattr(recovery_runtime, 'system_check', lambda: dict(reasons=[], factory_pids=[625]))
    monkeypatch.setattr(recovery_runtime.socket, 'socket', lambda *a, **k: Socket())
    world.options = dict(mode='manual', direction='left', duration=2., interactive=True,
                         execute=True, operator_ready=True, clearance_confirmed=True)
    world.ledger = tmp_path/'attempts'
    return world


def events(capsys):
    return [json.loads(s) for s in capsys.readouterr().out.splitlines()]


def test_runtime_ready_without_operator_sends_nothing(runtime_simulator, capsys):
    world = runtime_simulator
    assert recovery_runtime.run(world.options) == 2
    rows = events(capsys)
    assert any(r['event'] == 'confirmation_ready' for r in rows)
    assert not world.sends and not world.ledger.exists()


def test_runtime_interactive_dry_run_acknowledges_stop_but_never_sends(runtime_simulator, capsys):
    world = runtime_simulator
    world.confirm = True
    world.options['execute'] = False
    assert recovery_runtime.run(world.options) == 0
    rows = events(capsys)
    assert rows[-1]['event'] == 'confirmation_dry_run_complete'
    assert any(r.get('acknowledged_soft_stop') is True for r in rows)
    assert not world.sends and not world.ledger.exists()
    assert world.log.estop_latched  # The acknowledgement never clears factory/observer protection.


def test_runtime_confirmed_fake_trial_sends_exactly_one_pair(runtime_simulator, capsys):
    world = runtime_simulator
    world.confirm = True
    assert recovery_runtime.run(world.options) == 0
    assert world.sends == list(packets('left'))
    assert len(list(world.ledger.iterdir())) == 1
    assert events(capsys)[-1]['event'] == 'factory_success_and_upright_observed'


def test_runtime_new_soft_stop_after_confirmation_sends_nothing(runtime_simulator, capsys):
    world = runtime_simulator
    world.confirm = world.stop_after_confirm = True
    assert recovery_runtime.run(world.options) == 2
    assert not world.sends and not world.ledger.exists()
    assert events(capsys)[-1]['event'] == 'stopped'


def test_runtime_new_stop_between_packets_prevents_toggle(runtime_simulator, capsys):
    world = runtime_simulator
    world.confirm = world.stop_after_direction = True
    assert recovery_runtime.run(world.options) == 2
    assert world.sends == [packets('left')[0]]
    assert len(list(world.ledger.iterdir())) == 1
    assert events(capsys)[-1]['sent_motion_packets'] is True


def test_runtime_cancel_wins_over_confirmation_in_same_batch(runtime_simulator, capsys):
    world = runtime_simulator
    world.confirm = world.cancel = True
    assert recovery_runtime.run(world.options) == 2
    assert not world.sends and not world.ledger.exists()
    assert events(capsys)[-1]['event'] == 'stopped'
