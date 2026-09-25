"""Run every analysis script, save the figures and write results.txt.

    python3 analysis/run_all.py
"""
import runpy
import sys
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

buffer = StringIO()
scripts = sorted(HERE.glob("[0-9][0-9]_*.py"))
for script in scripts:
    print(f"running {script.name} ...", file=sys.stderr)
    with redirect_stdout(buffer):
        runpy.run_path(str(script), run_name="__main__")
(HERE / "results.txt").write_text(buffer.getvalue(), encoding="utf-8")
print(buffer.getvalue())
print(f"{len(scripts)} analyses done -> analysis/results.txt, figures in docs/images/analysis/", file=sys.stderr)
