"""Transport and odometry math shared by ROS nodes and regression tests."""
import math


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(payload):
    raw = payload.encode('ascii')
    return raw + f'*{crc16(raw):04X}\n'.encode('ascii')


def unframe(raw):
    body, check = raw.rstrip(b'\r\n').rsplit(b'*', 1)
    if len(check) != 4 or int(check, 16) != crc16(body):
        raise ValueError('checksum mismatch')
    return body.decode('ascii').split(',')


def counter_delta(new, old):
    return ((new - old + 2**31) % 2**32) - 2**31


def integrate(x, y, yaw, dl, dr, track):
    distance, angle = (dl + dr) / 2.0, (dr - dl) / track
    scale = 1.0 if abs(angle) < 1e-9 else math.sin(angle / 2.0) / (angle / 2.0)
    x += distance * scale * math.cos(yaw + angle / 2.0)
    y += distance * scale * math.sin(yaw + angle / 2.0)
    yaw = math.atan2(math.sin(yaw + angle), math.cos(yaw + angle))
    return x, y, yaw, distance, angle


class MotionGate:
    """A lost lease or sensor fault disarms until an explicit off->on request."""
    def __init__(self):
        self.armed = False
        self.requested = False
        self.lease_at = -math.inf
        self.command_at = -math.inf
        self.v = self.w = 0.0

    def permit(self, enable, now, healthy):
        if enable and not self.requested:
            self.armed = healthy
            self.command_at = -math.inf  # Require a fresh command after arming.
        self.requested = enable
        self.lease_at = now
        if not enable:
            self.armed = False

    def command(self, v, w, now):
        if not math.isfinite(v) or not math.isfinite(w):
            self.armed = False
            self.v = self.w = 0.0
            return
        self.v, self.w, self.command_at = v, w, now

    def output(self, now, healthy):
        if now - self.lease_at > 0.4 or not healthy:
            self.armed = False
        if not self.armed or now - self.command_at > 0.3:
            return 0.0, 0.0
        scale = max(1.0, abs(self.v) / 0.18, abs(self.w) / 0.6)
        return self.v / scale, self.w / scale
