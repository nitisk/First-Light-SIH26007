"""Repository launcher; simulation imports remain isolated from hardware code."""
from pathlib import Path
import runpy
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'simulation'
    if len(sys.argv) > 1 and mode in ('hardware', 'command-center', 'simulation'):
        sys.argv.pop(1)
    else:
        mode = 'simulation'
    if mode == 'hardware':
        from hardware.raspberry_pi.rover_controller import main as run
        return run()
    if mode == 'command-center':
        from hardware.raspberry_pi.command_center import main as run
        return run()
    sys.path.insert(0, str(ROOT / 'simulation'))
    runpy.run_path(str(ROOT / 'simulation' / 'main.py'), run_name='__main__')
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
