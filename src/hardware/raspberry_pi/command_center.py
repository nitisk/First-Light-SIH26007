"""Read-only fleet receiver; never sends motor commands."""
import argparse
import json
from pathlib import Path
import socket
import time
from .network import PeerCache

def main():
    parser = argparse.ArgumentParser(description='Hardware fleet telemetry receiver')
    parser.add_argument('--port', type=int, default=5006)
    parser.add_argument('--log', default='results/hardware/fleet.jsonl')
    args = parser.parse_args()
    cache = PeerCache('COMMAND', 2.0, allow_mock=True)
    path = Path(args.log)
    path.parent.mkdir(parents=True, exist_ok=True)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock, path.open('a', encoding='utf-8') as log:
        sock.bind(('', args.port))
        sock.settimeout(.5)
        print(f'Command center listening on UDP {args.port}; Ctrl+C to stop', flush=True)
        last_display = 0
        try:
            while True:
                now = time.monotonic()
                cache.fresh(now)
                try:
                    raw, _ = sock.recvfrom(4097)
                    if cache.ingest(raw, time.monotonic(), time.time()):
                        log.write(raw.decode('utf-8').strip()+'\n')
                        log.flush()
                except socket.timeout:
                    pass
                if now-last_display >= 1:
                    rows = [f"{p['vehicle_id']} {p['state']} {'MOCK' if p.get('mock') else 'HARDWARE'}" for p in cache.fresh(now)]
                    print(' | '.join(rows) if rows else 'No fresh vehicle telemetry', flush=True)
                    last_display = now
        except KeyboardInterrupt:
            return 0

if __name__ == '__main__':
    raise SystemExit(main())
