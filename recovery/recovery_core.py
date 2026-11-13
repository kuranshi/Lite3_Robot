"""Pure recovery planning/detection. No network, ROS, or robot side effects."""
from dataclasses import dataclass
import ast
import math
import secrets
import struct


from recovery_config import PROFILE

TARGET = (None, 43893)  # Unconfigured: a real robot endpoint must be supplied.
HEADER = struct.Struct('<IiI')
MAX_AGE = .2


def plan(direction):
    if direction not in ('left', 'right'):
        raise ValueError('Choose left or right explicitly; no automatic direction selection')
    value = -12553 if direction == 'left' else 12553
    return [
        {'purpose': 'set_recovery_direction', 'code': 0x21010131, 'value': value, 'type': 0},
        {'purpose': 'toggle_stand_request_once', 'code': 0x21010202, 'value': 0, 'type': 0},
    ]


def packets(direction):
    return tuple(HEADER.pack(p['code'], p['value'], p['type']) for p in plan(direction))


def quaternion(value):
    q = tuple(float(v) for v in value)
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError('Invalid orientation')
    norm = math.sqrt(sum(v*v for v in q))
    if abs(norm-1.) > .002:
        raise ValueError('Orientation must be a unit quaternion')
    return tuple(v/norm for v in q)


def tilt_deg(q):
    x, y, _, _ = quaternion(q)
    return math.degrees(math.acos(max(-1., min(1., 1.-2.*(x*x+y*y)))))


def angle_deg(a, b):
    return math.degrees(2*math.acos(min(1., abs(sum(x*y for x, y in zip(a, b))))))


@dataclass(frozen=True)
class Sample:
    stamp_ns: int                 # ROS source stamp: duplicates never refresh history.
    received: float               # Local monotonic callback time.
    orientation: tuple


class FallDetector:
    """Conservative posture candidate, NOT a classifier of estop/fault causes.

    A >=65 degree body tilt must persist for hold_s and vary <=5 degrees.
    Manual observation defaults to one second; automatic recovery uses five.
    Quaternion tilt handles a fully inverted body and is independent of yaw.
    No velocity/acceleration units are inferred from undocumented telemetry.
    """
    def __init__(self, hold_s=1.):
        if not math.isfinite(hold_s) or hold_s < 1.:
            raise ValueError('Posture hold must be finite and at least one second')
        self.hold_s = hold_s
        self.history = []
        self.last_stamp = -1
        self.last_received = -math.inf
        self.last_tilt = None
        self.candidate = False
        self.announced = False
        self.upright_since = None

    def reset_stability(self):
        self.history.clear()
        self.candidate = False
        self.upright_since = None

    def expire(self, now):
        if not 0 <= now-self.last_received <= MAX_AGE:
            self.history.clear()
            self.candidate = False
            self.upright_since = None

    def update(self, sample, now):
        self.expire(now)
        if (not math.isfinite(now) or not math.isfinite(sample.received)
                or not 0 <= now-sample.received <= MAX_AGE
                or sample.stamp_ns <= self.last_stamp):
            return False
        q = quaternion(sample.orientation)
        self.last_stamp = sample.stamp_ns
        self.last_received = sample.received
        self.last_tilt = tilt_deg(q)
        if self.last_tilt < 65.:
            self.history.clear()
            self.candidate = False
            if self.last_tilt <= 20.:
                self.upright_since = now if self.upright_since is None else self.upright_since
                if now-self.upright_since >= 2.:
                    self.announced = False
            else:
                self.upright_since = None
            return False
        self.upright_since = None
        if self.history and angle_deg(self.history[0][1], q) > 5.:
            self.history.clear()
        self.history.append((now, q))
        while len(self.history) > 1 and now-self.history[1][0] >= self.hold_s:
            self.history.pop(0)
        self.candidate = len(self.history) >= 10 and now-self.history[0][0] >= self.hold_s
        event = self.candidate and not self.announced
        if event:
            self.announced = True
        return event


def normalize_imu_stamp(stamp_ns, wall, native):
    """Undo the verified receiver's optional PC offset, on the SAME robot.

    The receiver always stamps with the robot ROS wall clock, and only adds
    computer_minus_robot while clock_ready is True. Cross-host alignment is
    unnecessary for this robot-local observer; source age is still bounded.
    Returns robot wall nanoseconds, timing mode and worst-case uncertainty.
    """
    ready = native.get('clock_ready')
    if ready not in ('True', 'False'):
        raise ValueError('Native clock mode unavailable')
    offset, uncertainty = 0., 0.
    mode = 'robot_local'
    if ready == 'True':
        try:
            correction = ast.literal_eval(native['clock_correction'])
            if not isinstance(correction, (tuple, list)) or len(correction) != 3:
                raise ValueError('Invalid correction shape')
            measured, offset, uncertainty = map(float, correction)
            if not all(math.isfinite(v) for v in (measured, offset, uncertainty)):
                raise ValueError('Nonfinite correction')
            if not 0 <= uncertainty <= .02:
                raise ValueError('Excessive clock uncertainty')
        except (KeyError, ValueError, TypeError, SyntaxError) as exc:
            raise ValueError('Native clock correction invalid') from exc
        mode = 'aligned_to_computer'
    if not isinstance(stamp_ns, int) or stamp_ns <= 0 or not math.isfinite(wall):
        raise ValueError('Invalid IMU source stamp')
    normalized = stamp_ns-round(offset*1e9)
    age = wall-normalized*1e-9
    if not -uncertainty <= age <= MAX_AGE-uncertainty:
        raise ValueError('IMU source stamp stale, future or clock transition')
    return normalized, mode, uncertainty


def telemetry_reasons(detector, native, native_received, now, local_clock_verified=False):
    detector.expire(now)
    reasons = []
    if not detector.candidate:
        reasons.append('need_stable_fallen_orientation')
    if not 0 <= now-native_received <= MAX_AGE:
        reasons.append('native_diagnostics_missing_or_stale')
    if native.get('protocol_verified') != 'True':
        reasons.append('native_protocol_not_verified')
    if native.get('clock_ready') != 'True' and not local_clock_verified:
        reasons.append('native_clock_not_ready')
    try:
        age, battery = float(native['age_s']), float(native['battery_raw'])
        if not 0 <= age <= MAX_AGE:
            reasons.append('native_state_stale')
        if not 20 < battery <= 100:
            reasons.append('battery_not_above_20_percent')
    except (KeyError, ValueError, TypeError):
        reasons.append('native_diagnostics_invalid')
    # basic_state=6 is known to mean NORMAL STANDING on this firmware. Do not
    # import another model's "6=estop" mapping to authorize recovery.
    if native.get('basic_state') == '6':
        reasons.append('factory_still_reports_normal_standing')
    return reasons


@dataclass(frozen=True)
class ConfirmationOffer:
    nonce: str
    waiting_event: str
    direction: str
    stop_generation: int
    resume_soft_stop: bool
    expires_at: float

    @property
    def phrase(self):
        verb = 'RESUME' if self.resume_soft_stop else 'RECOVER'
        return f'{verb} {self.direction.upper()} {self.nonce}'

    def public(self, now):
        return dict(nonce=self.nonce, waiting_event=self.waiting_event,
                    direction=self.direction, stop_generation=self.stop_generation,
                    resume_soft_stop=self.resume_soft_stop, phrase=self.phrase,
                    expires_in_s=max(0., self.expires_at-now))


class ManualConfirmation:
    """Short-lived, single-use operator intent tied to a current waiting event.

    This does not classify a stop or clear any factory protection. Only the
    exact explicit operator response can acknowledge the observed soft stop,
    and that acknowledgement cannot carry into a different event/generation.
    """
    def __init__(self):
        self.offer = None

    def sync(self, context, now, blocked):
        if blocked or context is None:
            self.offer = None
            return None
        event, direction, generation, soft_stop, waiting_deadline = context
        if (not math.isfinite(now) or not math.isfinite(waiting_deadline)
                or now >= waiting_deadline):
            self.offer = None
            return None
        plan(direction)
        previous = self.offer
        if (previous is not None and previous.waiting_event == event
                and previous.direction == direction and previous.stop_generation == generation
                and previous.resume_soft_stop == soft_stop and now < previous.expires_at):
            return None
        self.offer = ConfirmationOffer(secrets.token_hex(4), event, direction,
                                       generation, soft_stop, min(now+15., waiting_deadline))
        return self.offer

    def consume(self, message, context, now):
        offer, self.offer = self.offer, None  # Even rejected responses cannot be replayed.
        if offer is None:
            raise ValueError('No current confirmation offer; early or repeated input rejected')
        if not math.isfinite(now) or now >= offer.expires_at:
            raise ValueError('Confirmation expired')
        if context != (offer.waiting_event, offer.stop_generation):
            raise ValueError('Factory waiting event or soft stop changed')
        expected = dict(action='confirm', nonce=offer.nonce, waiting_event=offer.waiting_event,
                        direction=offer.direction, phrase=offer.phrase)
        if message != expected:
            raise ValueError('Confirmation does not match this event, direction and phrase')
        return offer


def one_shot(direction, check_ready, reserve_attempt, transmit, pause, record):
    """Two allowlisted datagrams; never retry the non-idempotent stand toggle.

    check_ready returns an immutable waiting-event identity. The caller must
    reserve it durably before the first send, including partial-send failures.
    No cleanup stand/mode/estop/velocity commands are generated on any path.
    """
    data = packets(direction)
    event = check_ready()
    reserve_attempt(event)
    record('attempt_reserved', waiting_event=event, direction=direction)
    if check_ready() != event:
        raise RuntimeError('Factory waiting event changed before direction request')
    transmit(data[0])
    record('direction_sent', delivery_acknowledged=False)
    pause(.10)
    if check_ready() != event:
        raise RuntimeError('Factory waiting event changed before stand request')
    # Once attempted, a send exception also leaves the durable reservation in
    # place: lack of a send result is not proof the robot did not receive it.
    record('stand_send_attempt', retry_allowed=False)
    transmit(data[1])
    record('stand_sent', delivery_acknowledged=False)
