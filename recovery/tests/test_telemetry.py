"""Clock/data availability regressions; no ROS installation or robot required."""
import math
from types import SimpleNamespace

import pytest

from recovery_core import FallDetector, normalize_imu_stamp
from recovery_runtime import Observer
import recovery_runtime as rt
from lite3_recovery import AutoDisplay
from test_recovery import q


WALL = 946684800.


def test_robot_local_imu_needs_no_pc_clock_alignment():
    stamp = round((WALL-.01)*1e9)
    normalized, mode, uncertainty = normalize_imu_stamp(stamp, WALL, {'clock_ready': 'False'})
    assert normalized == stamp and mode == 'robot_local' and uncertainty == 0.


def test_aligned_stamp_is_normalized_with_bounded_uncertainty():
    stamp = round((WALL+19.43-.01)*1e9)
    normalized, mode, uncertainty = normalize_imu_stamp(stamp, WALL,
        dict(clock_ready='True', clock_correction='(100., 19.43, .003)'))
    assert normalized*1e-9 == pytest.approx(WALL-.01)
    assert mode == 'aligned_to_computer' and uncertainty == .003


@pytest.mark.parametrize('age', [-.01, .201, 5.])
def test_local_future_or_stale_stamp_never_bypasses_source_age(age):
    with pytest.raises(ValueError):
        normalize_imu_stamp(round((WALL-age)*1e9), WALL, dict(clock_ready='False'))


@pytest.mark.parametrize('correction', ['None', '[]', '(1,2)', 'bad',
    "(1., float('nan'), .001)", '(1., 20., -.001)', '(1., 20., .021)'])
def test_malformed_alignment_never_falls_back_to_unverified_time(correction):
    with pytest.raises(ValueError):
        normalize_imu_stamp(round(WALL*1e9), WALL,
            dict(clock_ready='True', clock_correction=correction))


def test_uncertainty_is_subtracted_from_max_source_age():
    with pytest.raises(ValueError):
        normalize_imu_stamp(round((WALL+10.-.19)*1e9), WALL,
            dict(clock_ready='True', clock_correction='(1., 10., .02)'))


def test_missing_clock_mode_does_not_assume_robot_local_time():
    with pytest.raises(ValueError):
        normalize_imu_stamp(round(WALL*1e9), WALL, {})


@pytest.fixture
def observer(monkeypatch):
    clock = SimpleNamespace(now=10.)
    monkeypatch.setattr(rt, 'time', SimpleNamespace(monotonic=lambda: clock.now,
                                                   time=lambda: WALL+clock.now))
    obs = Observer.__new__(Observer)
    obs.detector = FallDetector(5.)
    obs.native = dict(protocol_verified='True', clock_ready='False',
        age_s='.001', battery_raw='90', basic_state='99')
    obs.native_received = clock.now
    obs.last_native_counter = 0
    obs.valid_imu = obs.invalid_imu = obs.candidate_events = 0
    obs.imu_source_ns = obs.imu_clock_mode = None
    obs.imu_clock_uncertainty = 0.
    obs.imu_timing_error = None
    obs.node = SimpleNamespace(count_publishers=lambda topic: 1)
    def receive(at, offset=0., age=.005, aligned=False):
        clock.now = at
        obs.native_received = at
        obs.native['clock_ready'] = 'True' if aligned else 'False'
        obs.native['clock_correction'] = str((at, offset, .001)) if aligned else 'None'
        source = round((WALL+at+offset-age)*1e9)
        sec, ns = divmod(source, 1000000000)
        x, y, z, w = q(179.)
        msg = SimpleNamespace(header=SimpleNamespace(frame_id='base_link',
            stamp=SimpleNamespace(sec=sec, nanosec=ns)), orientation_covariance=[.001],
            orientation=SimpleNamespace(x=x, y=y, z=z, w=w))
        obs.imu(msg)
        return msg
    return obs, clock, receive


def test_real_observer_accumulates_five_seconds_with_clock_ready_false(observer):
    obs, clock, receive = observer
    for i in range(260):
        receive(10.+i*.02)
    snapshot = obs.snapshot()
    assert obs.detector.candidate and obs.candidate_events == 1
    assert snapshot['imu_clock_mode'] == 'robot_local'
    assert snapshot['imu_source_age_s'] == pytest.approx(.005, abs=1e-6)
    assert not snapshot['reasons'] and obs.valid_imu == 260


def test_normalized_stamps_stay_monotonic_when_alignment_mode_changes(observer):
    obs, clock, receive = observer
    for i in range(260):
        aligned = int(i/50) % 2 == 0
        receive(10.+i*.02, offset=19.43 if aligned else 0., aligned=aligned)
    assert obs.detector.candidate and obs.valid_imu == 260
    assert not obs.snapshot()['reasons']


def test_same_source_stamp_cannot_refresh_callback_freshness(observer):
    obs, clock, receive = observer
    msg = receive(10.)
    clock.now = 10.1
    obs.imu(msg)
    assert obs.valid_imu == 1 and obs.detector.last_received == 10.
    clock.now = 10.21
    obs.imu(msg)
    reasons = obs.snapshot()['reasons']
    assert obs.valid_imu == 1 and not obs.detector.candidate
    assert 'imu_source_timestamp_stale_or_invalid' in reasons


def test_source_age_and_callback_age_cannot_add_up_to_more_than_200ms(observer):
    obs, clock, receive = observer
    receive(10., age=.15)
    clock.now = 10.1
    obs.native_received = 10.1
    assert 'imu_source_timestamp_stale_or_invalid' in obs.snapshot()['reasons']


def test_local_clock_does_not_bypass_native_freshness_or_protocol(observer):
    obs, clock, receive = observer
    receive(10.)
    obs.native_received = -math.inf
    obs.native['protocol_verified'] = 'False'
    reasons = obs.snapshot()['reasons']
    assert 'native_diagnostics_missing_or_stale' in reasons
    assert 'native_protocol_not_verified' in reasons


def test_missing_imu_display_reports_data_problem_not_a_zero_second_fall(capsys):
    display = AutoDisplay()
    display.handle(dict(event='auto_observation', valid_imu_callbacks=0, tilt_deg=None,
        reasons=['need_stable_fallen_orientation'], factory_reasons=[], stable_fallen_s=0., phase='observing'))
    output = capsys.readouterr().out
    assert '尚未收到IMU' in output and '稳定倒地 0/5 秒' not in output
