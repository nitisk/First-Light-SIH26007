"""Wi-Fi UDP warnings and telemetry. Only delivered, fresh packets are used."""
import json
import math
import socket
import time

class PeerCache:
    def __init__(self, ident, timeout=1.0, allow_mock=False):
        self.ident, self.timeout = ident, timeout
        self.allow_mock = allow_mock
        self.peers = {}

    def ingest(self, raw, now, wall):
        try:
            if len(raw) > 4096:
                return False
            p = json.loads(raw)
            if not isinstance(p, dict) or p.get('type') != 'first-light-v1':
                return False
            if type(p.get('mock', False)) is not bool or (p.get('mock', False) and not self.allow_mock):
                return False
            for key in ('distance_m','speed_mps','target_mps','ttc_s','left_pwm','right_pwm'):
                value = p.get(key)
                if value is not None and (type(value) not in (int, float) or not math.isfinite(value)):
                    return False
            ident, timestamp, seq = p['vehicle_id'], p['timestamp'], p['seq']
            if not isinstance(ident, str) or not 1 <= len(ident) <= 40 or ident == self.ident:
                return False
            if type(timestamp) not in (float, int) or not math.isfinite(timestamp) or abs(wall-timestamp) > self.timeout:
                return False
            if type(seq) is not int or not 0 <= seq <= 0xffffffff:
                return False
            if type(p['warning']) is not bool or type(p['fog']) is not bool:
                return False
            if not isinstance(p['zone'], str) or not 1 <= len(p['zone']) <= 40:
                return False
            if p['state'] not in ('WAITING','DISARMED','CRUISE','CAUTION','STOP','FAULT','PARKED'):
                return False
            old = self.peers.get(ident)
            if old and timestamp <= old[1]['timestamp']:
                return False
            if len(self.peers) >= 32 and ident not in self.peers:
                return False
            self.peers[ident] = (now, p)
            return True
        except (ValueError, KeyError, TypeError, UnicodeDecodeError, RecursionError, OverflowError):
            return False

    def fresh(self, now):
        self.peers = {key: value for key, value in self.peers.items() if 0 <= now-value[0] <= self.timeout}
        return [p for _, p in self.peers.values()]

    def warning(self, now, zone):
        return any(p['zone'] == zone and (p['warning'] or p['fog']) for p in self.fresh(now))

class Link:
    def __init__(self, ident, peer_port=5005, destination='255.255.255.255', center=None, timeout=1.0, allow_mock=False):
        self.cache = PeerCache(ident, timeout, allow_mock)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(('', peer_port))
        self.sock.setblocking(False)
        self.destination = (destination, peer_port)
        self.center = (center, 5006) if center else None
        self.errors = 0

    def receive(self, now):
        self.cache.fresh(now)
        for _ in range(32):
            try:
                raw, _ = self.sock.recvfrom(4097)
            except BlockingIOError:
                break
            except OSError:
                self.errors += 1
                break
            self.cache.ingest(raw, now, time.time())

    def send(self, packet):
        raw = json.dumps(packet, allow_nan=False).encode('utf-8')
        for dest in (self.destination, self.center):
            if dest:
                try:
                    self.sock.sendto(raw, dest)
                except OSError:
                    self.errors += 1

    def close(self):
        self.sock.close()
