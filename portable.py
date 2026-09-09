"""Launcher for the optional self-contained Windows x64 runtime."""
from pathlib import Path
import os
import runpy
import sys

ROOT=Path(__file__).resolve().parent
os.chdir(ROOT)
handle=os.add_dll_directory(str(ROOT/'runtime'))
if (ROOT/'runtime'/'extra').is_dir():
    sys.path.insert(0,str(ROOT/'runtime'/'extra'))
sys.path.insert(0,str(ROOT/'src'))
sys.path.insert(0,str(ROOT/'src'/'simulation'))
sys.argv=['main.py',*sys.argv[1:]]
runpy.run_path(str(ROOT/'src'/'main.py'),run_name='__main__')
