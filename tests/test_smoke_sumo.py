"""Increment 1 smoke test: SUMO loads the network and demand and moves traffic."""

import subprocess

from cadence.paths import R1_DIR, sumo_bin, sumo_env
from cadence.simulation import demand as d


def test_sumo_runs_s0_demand(tmp_path, spec):
    files = d.generate("S0", 0, tmp_path, spec)
    begin = d.scenario_window("S0", spec).start
    summary = tmp_path / "summary.xml"
    res = subprocess.run(
        [sumo_bin("sumo"), "-n", str(R1_DIR / "r1.net.xml"), "-r", str(files["routes"]),
         "-b", str(begin), "-e", str(begin + 900), "--time-to-teleport", "-1",
         "--collision.action", "warn", "--summary-output", str(summary), "--no-step-log", "true"],
        capture_output=True, text=True, env=sumo_env())
    assert res.returncode == 0, res.stderr
    assert "Teleport" not in res.stderr
    assert "Collision" not in res.stderr
    text = summary.read_text()
    last = text.strip().splitlines()[-2]
    arrived = int(last.split('arrived="')[1].split('"')[0])
    assert arrived > 100
