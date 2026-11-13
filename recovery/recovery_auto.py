"""Opt-in automatic recovery. Imported on the robot; never starts on import."""
import json
import math

import recovery_runtime as rt
from recovery_core import MAX_AGE, one_shot, packets, plan


HOLD_S = 5.
UPRIGHT_S = 2.


def policy_reasons(policy, factory):
    if policy == 'any':
        return []
    if policy == 'natural-only':
        # There is no verified natural/manual cause field on the current
        # receiver. Absence of a stop log must NEVER count as natural fall.
        return ['natural_fall_source_unavailable']
    raise ValueError('Unknown fall policy')


def recovery_reasons(observer, factory, policy, now):
    acknowledgement = ((factory.waiting, factory.stop_generation)
                       if policy == 'any' else None)
    return (observer.snapshot()['reasons']+factory.reasons(now, acknowledgement)
            +policy_reasons(policy, factory))


def upright_ready(observer, snapshot, now):
    # A horizontal body alone may be prone rather than normally standing.
    allowed = {'need_stable_fallen_orientation', 'factory_still_reports_normal_standing'}
    return (not [r for r in snapshot['reasons'] if r not in allowed]
            and 0 <= now-observer.detector.last_received <= MAX_AGE
            and observer.detector.last_tilt is not None
            and observer.detector.last_tilt <= 20.
            and snapshot.get('native', {}).get('basic_state') == '6')


def run_auto(options):
    """Five seconds of settled posture, one transaction per fall episode.

    'any' scopes acknowledgement of an already observed soft stop to this
    transaction. A NEW stop still invalidates an in-flight transaction. No
    estop-clear, mode, heartbeat, velocity or cleanup command is generated.
    """
    execute = options.get('execute', False)
    policy = options.get('fall_policy', 'any')
    policy_reasons(policy, None)
    if options.get('interactive'):
        raise RuntimeError('Auto mode does not use manual confirmation')
    if options.get('direction') != 'left':
        raise RuntimeError('Auto mode currently supports the validated left direction only')
    if execute and policy == 'natural-only':
        raise RuntimeError('Natural-only execution unavailable: fall cause source is not verified')
    if execute and not options.get('audit_enabled'):
        raise RuntimeError('Auto execution requires a new local audit file')
    duration = options.get('duration', 10.)
    if not math.isfinite(duration) or not 1 <= duration <= 300:
        raise ValueError('Auto duration must be between 1 and 300 seconds')

    started = rt.time.monotonic()
    def emit(event, **data):
        print(json.dumps(dict(event=event, elapsed_s=round(rt.time.monotonic()-started, 3),
                              **data), allow_nan=False, ensure_ascii=False), flush=True)

    observer = lock = sock = None
    sent = False
    attempts = successes = 0
    try:
        system = rt.system_check()
        emit('system', **system)
        emit('auto_configuration', fall_policy=policy, stable_fallen_s=HOLD_S,
             direction='left', dry_run=not execute, environment_detection=False,
             soft_stop_can_be_followed_by_recovery=policy == 'any',
             natural_cause_detection_available=False)
        emit('request_plan', direction='left', requests=plan('left'),
             packet_hex=[p.hex() for p in packets('left')], dry_run=not execute)
        if execute and system['reasons']:
            raise RuntimeError('System check refused: '+','.join(system['reasons']))
        observer = rt.Observer(hold_s=HOLD_S)
        factory = rt.FactoryLog()
        factory.poll(rt.time.monotonic(), rt.time.time())
        operator = rt.OperatorInput()  # EOF / CANCEL cancels before either send.
        if execute:
            lock = open(rt.LOCK, 'a')
            rt.fcntl.flock(lock, rt.fcntl.LOCK_EX | rt.fcntl.LOCK_NB)
        emit('observing', automatic=True, execute=execute, fall_policy=policy,
             waiting_for_new_factory_event=True, sends_on_startup=False)
        deadline = rt.time.monotonic()+duration
        generation = factory.stop_generation
        last_report = -math.inf
        waiting_for_rearm = False
        rearm_since = None

        def control():
            messages = operator.poll()
            if any(m == {'action': 'cancel'} for m in messages):
                raise RuntimeError('Operator cancelled; factory action may continue if already requested')
            for _ in messages:
                emit('auto_input_ignored', instruction='Auto mode accepts CANCEL only')

        def pause(seconds):
            nonlocal generation
            observer.spin(seconds)
            for item in factory.poll(rt.time.monotonic(), rt.time.time()):
                emit(item)
            if factory.stop_generation != generation:
                generation = factory.stop_generation
                observer.detector.reset_stability()
                emit('auto_countdown_reset', reason='new_soft_emergency', stop_generation=generation)

        def report(snapshot, phase, reasons):
            nonlocal last_report
            now = rt.time.monotonic()
            if now-last_report >= 1.:
                history = observer.detector.history
                held = max(0., now-history[0][0]) if history else 0.
                emit('auto_observation', **snapshot, phase=phase, execute=execute,
                     factory_reasons=reasons, stable_fallen_s=round(min(HOLD_S, held), 3),
                     remaining_s=round(max(0., HOLD_S-held), 3), fall_policy=policy)
                last_report = now

        while rt.time.monotonic() < deadline:
            pause(.02)
            control()
            now = rt.time.monotonic()
            snapshot = observer.snapshot()
            if now-started >= 5. and (observer.valid_imu == 0 or not snapshot.get('native')):
                emit('telemetry_unavailable', valid_imu_callbacks=observer.valid_imu,
                     hint='未收到有效IMU或原厂诊断；请按 recovery/README.md 启动只读遥测接收器。')
                raise RuntimeError('No usable IMU/native telemetry after 5 seconds; auto session stopped without a recovery request')
            if factory.fault_latched:
                raise RuntimeError('Factory fault latched; automatic recovery locked')
            if waiting_for_rearm:
                ready = upright_ready(observer, snapshot, now)
                rearm_since = (now if rearm_since is None else rearm_since) if ready else None
                report(snapshot, 'waiting_for_normal_standing', [])
                if rearm_since is not None and now-rearm_since >= UPRIGHT_S:
                    waiting_for_rearm = False
                    rearm_since = None
                    observer.detector.reset_stability()
                    emit('auto_rearmed', upright_normal_standing_s=UPRIGHT_S)
                continue
            reasons = system['reasons']+recovery_reasons(observer, factory, policy, now)
            # Invalid diagnostics or loss of native readiness cannot accumulate
            # the hold across an unseen interval. Missing factory WAIT does not
            # itself reset valid IMU history; it remains a send-time gate.
            invalid = [r for r in snapshot['reasons'] if r not in {
                'need_stable_fallen_orientation', 'factory_still_reports_normal_standing'}]
            if invalid or 'factory_still_reports_normal_standing' in snapshot['reasons']:
                observer.detector.reset_stability()
            report(snapshot, 'observing', reasons)
            if reasons:
                continue
            event, accepted_generation = factory.waiting, factory.stop_generation
            emit('auto_trigger_ready', waiting_event=event, stop_generation=accepted_generation,
                 fall_policy=policy, stable_fallen_s=HOLD_S, dry_run=not execute)
            if not execute:
                emit('auto_dry_run_ready', sent_motion_packets=False,
                     persistent_attempt_created=False, requires_execute_for_motion=True)
                return 0

            def check_ready():
                pause(.02)
                control()
                now = rt.time.monotonic()
                reasons = recovery_reasons(observer, factory, policy, now)
                if (factory.waiting, factory.stop_generation) != (event, accepted_generation):
                    reasons.append('factory_waiting_event_or_soft_stop_changed')
                if reasons:
                    raise RuntimeError('Auto recovery refused: '+','.join(reasons))
                return event

            current = rt.system_check()
            if current['reasons'] or current['factory_pids'] != system['factory_pids']:
                raise RuntimeError('Controller or firmware changed during observation')
            if sock is None:
                sock = rt.socket.socket(rt.socket.AF_INET, rt.socket.SOCK_DGRAM)
                sock.bind((rt.TARGET[0], 0))
            def transmit(data):
                nonlocal sent
                sent = True
                if sock.sendto(data, rt.TARGET) != len(data):
                    raise OSError('Short UDP send; result unknown, no retry')
            one_shot('left', check_ready, rt.reserve, transmit, pause, emit)
            attempts += 1
            outcome_deadline = rt.time.monotonic()+10.
            upright_since = None
            while rt.time.monotonic() < outcome_deadline:
                pause(.02)
                control()
                now = rt.time.monotonic()
                snapshot = observer.snapshot()
                report(snapshot, 'recovering', [])
                if factory.failed or factory.stop_generation != accepted_generation:
                    raise RuntimeError('Factory reported fault/emergency during recovery; no retry')
                if factory.direction_seen and factory.direction_seen != 'left':
                    raise RuntimeError('Factory recovery direction mismatch; no retry')
                ready = upright_ready(observer, snapshot, now)
                upright_since = (now if upright_since is None else upright_since) if ready else None
                if (factory.recovery_started and factory.succeeded and factory.direction_seen == 'left'
                        and upright_since is not None and now-upright_since >= UPRIGHT_S):
                    successes += 1
                    emit('auto_recovery_complete', native_normal_standing_observed=True,
                         upright_normal_standing_s=UPRIGHT_S, successes=successes)
                    waiting_for_rearm = True
                    break
            else:
                raise RuntimeError('Recovery outcome unknown after 10 seconds; no retry, check robot')
        emit('finished', execute=execute, sent_motion_packets=sent,
             attempts=attempts, successes=successes, retry_allowed=False)
        return 0 if successes or not execute else 2
    except (Exception, KeyboardInterrupt) as exc:
        emit('stopped', error=str(exc) or 'interrupted', sent_motion_packets=sent,
             retry_allowed=False, factory_action_may_continue=sent)
        return 2
    finally:
        if sock is not None:
            sock.close()
        if lock is not None:
            lock.close()
        if observer is not None:
            observer.close()
