"""Run using python src/main.py hardware --mock, or --port PORT --arm."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time
from .network import Link
from .protocol import parse_sample
from .safety import Config, Controller
from .transport import MockTransport, SerialTransport

def main():
    parser = argparse.ArgumentParser(description='First Light tabletop rover controller')
    parser.add_argument('--config', default=str(Path(__file__).resolve().parents[1] / 'config.json'))
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--mock', action='store_true')
    source.add_argument('--port', help='/dev/ttyACM0 on Pi or COM3 on Windows')
    parser.add_argument('--arm', action='store_true', help='Enable physical motor commands')
    parser.add_argument('--duration', type=float, default=0, help='Seconds; 0 runs until Ctrl+C')
    parser.add_argument('--network', action='store_true', help='Enable Wi-Fi V2V, disabled by default')
    parser.add_argument('--broadcast', default='255.255.255.255')
    parser.add_argument('--center', help='Command-center laptop IP; implies --network')
    parser.add_argument('--log', default='results/hardware/telemetry.jsonl')
    args = parser.parse_args()
    if not 0 <= args.duration < 86400:
        parser.error('duration must be between 0 and 86400 seconds')
    cfg = Config.load(args.config)
    control = Controller(cfg)
    transport = MockTransport() if args.mock else SerialTransport(args.port)
    link = None
    try:
        if args.network or args.center:
            link = Link(cfg.vehicle_id, destination=args.broadcast, center=args.center, timeout=cfg.peer_timeout_s, allow_mock=args.mock)
        path = Path(args.log)
        path.parent.mkdir(parents=True, exist_ok=True)
        start = time.monotonic()
        duration = args.duration or (14 if args.mock else float('inf'))
        seq, previous_state = 0, None
        print('MOCK SENSOR REPLAY (no hardware)' if args.mock else f'USB {args.port}: ' + ('ARMED' if args.arm else 'MONITOR ONLY'), flush=True)
        with path.open('w', encoding='utf-8') as log:
            while time.monotonic()-start < duration:
                tick = time.monotonic()
                raw = transport.read()
                sample = None
                if raw:
                    try:
                        sample = parse_sample(raw)
                    except ValueError:
                        pass  # Missing/invalid data always commands zero; watchdog also runs on Uno.
                now = time.monotonic()
                if link:
                    link.receive(now)
                warning = link.cache.warning(now, cfg.zone) if link else False
                output = control.update(sample, now, armed=args.arm or args.mock, peer_warning=warning)
                transport.write(seq, output.left_pwm, output.right_pwm)
                packet = dict(type='first-light-v1', vehicle_id=cfg.vehicle_id, zone=cfg.zone,
                              timestamp=time.time(), seq=seq, mock=args.mock,
                              fog=sample.fog if sample else False,
                              warning=output.state in ('STOP','FAULT') or (sample.fog if sample else False),
                              distance_m=sample.distance_m if sample else None,
                              **asdict(output))
                # Do not rebroadcast peer-only caution as a new warning: avoids warning feedback loops.
                log.write(json.dumps(packet, allow_nan=False)+'\n')
                log.flush()
                if link:
                    link.send(packet)
                if output.state != previous_state:
                    print(f'{now-start:5.1f}s {output.state}: {output.reason} | PWM {output.left_pwm}/{output.right_pwm}', flush=True)
                    previous_state = output.state
                seq = (seq+1) & 0xffffffff
                time.sleep(max(0, .1-(time.monotonic()-tick)))
        return 2 if control.fault else 0
    except KeyboardInterrupt:
        return 0
    finally:
        try:
            transport.close()
        finally:
            if link:
                link.close()

if __name__ == '__main__':
    raise SystemExit(main())
