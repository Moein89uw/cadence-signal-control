"""Validate the corridor queue model against SUMO and write the report.

    python experiments/validate_prediction.py            # all stages
    python experiments/validate_prediction.py --stage collect|calibrate|evaluate|report

Stages:
1. collect: one SUMO run per scenario records true states every 5 min after
   warm-up, per-second queues and system counts, signal states and SUMO's
   own discharge times (runs/validation/<S>/truth.pkl).
2. calibrate: grid over the two permissive-left gap parameters on S1 only,
   scored on the I2 shared-lane queues.
3. evaluate: 5 min predictions from every start in every scenario, with and
   without first-in-first-out shared lanes.
4. report: docs/validation/prediction_model_m1.md, figures and a JSON of
   all numbers.

Needs matplotlib for the figures (pip install -e ".[report]").
"""

from __future__ import annotations

import argparse
import json
import statistics as st
from pathlib import Path

from cadence.paths import R1_DIR, REPO_ROOT, load_spec
from cadence.prediction.queue_model import ModelParams
from cadence.prediction.validation import collect_truth, ensure_routes, load_truth, predict_and_compare, save_truth
from cadence.simulation import demand as dm
from cadence.simulation.replay import in_fresh_process
from cadence.simulation.runner import net_lane_lengths

RUNS = REPO_ROOT / "runs" / "validation"
DOCS = REPO_ROOT / "docs" / "validation"
HORIZON_S = 300.0
GRID_BEFORE = (0.0, 1.0, 2.0)
GRID_AFTER = (0.0, 1.0, 2.0, 3.0)
SHARED = (("I2", "SB"), ("I2", "NB"))
COLORS = {"s1": "#2a78d6", "s2": "#eb6834", "s3": "#1baf7a"}
SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"


def _collect(scenario: str) -> str:
    work = RUNS / scenario
    truth = collect_truth(scenario, work, horizon_s=HORIZON_S)
    save_truth(truth, work / "truth.pkl")
    return str(work / "truth.pkl")


def stage_collect() -> None:
    for sc in dm.SCENARIOS:
        print("collect", sc, in_fresh_process(_collect, sc), flush=True)


def _truths():
    return {sc: ensure_routes(load_truth(RUNS / sc / "truth.pkl"), RUNS / sc / f"{sc}_r00.rou.xml") for sc in dm.SCENARIOS}


def _evaluate(truth, params, spec, nl):
    return [predict_and_compare(truth, t0, HORIZON_S, spec, nl, params) for t0 in truth.starts]


def _shared_mae(results) -> float:
    errs = [abs(c.queue_model[k] - c.queue_sumo.get(k, 0)) for c in results for k in SHARED]
    return sum(errs) / len(errs)


def _shared_bias(results) -> float:
    errs = [c.queue_model[k] - c.queue_sumo.get(k, 0) for c in results for k in SHARED]
    return sum(errs) / len(errs)


def stage_calibrate() -> dict:
    spec = load_spec()
    nl = net_lane_lengths(R1_DIR / "r1.net.xml")
    truth = load_truth(RUNS / "S1" / "truth.pkl")
    grid = []
    for b in GRID_BEFORE:
        for a in GRID_AFTER:
            res = _evaluate(truth, ModelParams(gap_before_s=b, gap_after_s=a), spec, nl)
            grid.append({"gap_before_s": b, "gap_after_s": a, "shared_queue_mae": round(_shared_mae(res), 3),
                         "shared_queue_bias": round(_shared_bias(res), 3),
                         "vt_mean_abs_rel_error": round(st.mean(abs(c.vt_rel_error) for c in res), 5)})
            print(grid[-1], flush=True)
    best = min(grid, key=lambda g: (g["shared_queue_mae"], g["vt_mean_abs_rel_error"]))
    out = {"grid": grid, "chosen": best}
    (RUNS / "calibration.json").write_text(json.dumps(out, indent=2))
    return out


def _summary(results) -> dict:
    rel = [c.vt_rel_error for c in results]
    abs_rel = sorted(abs(r) for r in rel)
    groups = {"mainline": lambda k: k[1] in ("EB", "WB"),
              "side (own lanes)": lambda k: k[1] in ("NB", "SB") and k not in SHARED,
              "I2 shared lanes": lambda k: k in SHARED}
    q = {}
    for name, sel in groups.items():
        e = [c.queue_model[k] - c.queue_sumo.get(k, 0) for c in results for k in c.queue_model if sel(k)]
        q[name] = {"mae": round(sum(abs(x) for x in e) / len(e), 2), "bias": round(sum(e) / len(e), 2)}
    sd = [sum(n for m, n in c.discharge_model.items() if m[:2] == k) - sum(n for m, n in c.discharge_sumo.items() if m[:2] == k)
          for c in results for k in SHARED]
    q["I2 shared lanes"]["discharge_mae"] = round(sum(abs(x) for x in sd) / len(sd), 2)
    q["I2 shared lanes"]["discharge_bias"] = round(sum(sd) / len(sd), 2)
    d = [c.discharge_model[m] - c.discharge_sumo[m] for c in results for m in set(c.discharge_model) | set(c.discharge_sumo)]
    tot = [(sum(c.discharge_model.values()) - sum(c.discharge_sumo.values())) / max(1, sum(c.discharge_sumo.values()))
           for c in results]
    return {
        "starts": len(results),
        "vehicle_time_rel_error": {"bias_pct": round(100 * st.mean(rel), 2), "mean_abs_pct": round(100 * st.mean(abs_rel), 2),
                                   "p90_abs_pct": round(100 * abs_rel[int(0.9 * (len(abs_rel) - 1))], 2),
                                   "max_abs_pct": round(100 * abs_rel[-1], 2)},
        "mean_queue": q,
        "discharge_per_movement_mae": round(sum(abs(x) for x in d) / len(d), 2),
        "discharge_total_bias_pct": round(100 * st.mean(tot), 2),
    }


def stage_evaluate(chosen: dict) -> dict:
    spec = load_spec()
    nl = net_lane_lengths(R1_DIR / "r1.net.xml")
    truths = _truths()
    out = {"params": chosen, "horizon_s": HORIZON_S, "scenarios": {}, "points": {}}
    for sc, truth in truths.items():
        out["scenarios"][sc] = {}
        out["points"][sc] = {}
        for fifo in (True, False):
            p = ModelParams(gap_before_s=chosen["gap_before_s"], gap_after_s=chosen["gap_after_s"], fifo=fifo)
            res = _evaluate(truth, p, spec, nl)
            name = "fifo" if fifo else "no_fifo"
            out["scenarios"][sc][name] = _summary(res)
            out["points"][sc][name] = [
                {"t0": c.t0, "vt_rel_error": c.vt_rel_error,
                 "shared": {f"{k[0]} {k[1]}": [round(c.queue_model[k], 2), round(c.queue_sumo.get(k, 0), 2)] for k in SHARED},
                 "shared_discharge": {f"{k[0]} {k[1]}": [sum(n for m, n in c.discharge_model.items() if m[:2] == k),
                                                         sum(n for m, n in c.discharge_sumo.items() if m[:2] == k)]
                                      for k in SHARED}}
                for c in res]
        print(sc, json.dumps(out["scenarios"][sc]), flush=True)
    (RUNS / "evaluation.json").write_text(json.dumps(out, indent=2))
    return out


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def _style(ax):
    ax.set_facecolor(SURFACE)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=9)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def _example_series(truth, t0, spec, nl, chosen, approach=("I2", "NB")):
    from cadence.prediction.queue_model import CorridorQueueModel
    from cadence.prediction.validation import initialize, signal_lookup

    series = {}
    for fifo in (True, False):
        m = CorridorQueueModel(ModelParams(gap_before_s=chosen["gap_before_s"], gap_after_s=chosen["gap_after_s"],
                                           fifo=fifo), spec)
        m.set_signals(signal_lookup(truth.state_changes, spec))
        initialize(m, truth, t0, HORIZON_S, spec, nl)
        m.run(t0 + HORIZON_S, record_queues_every=1)
        series["fifo" if fifo else "no_fifo"] = [(t - t0, q[approach]) for t, q in m.queue_count]
    series["sumo"] = [(s - t0, truth.queues[s][approach]) for s in range(int(t0) + 1, int(t0 + HORIZON_S) + 1)]
    return series


def stage_report(calib: dict, ev: dict) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    DOCS.mkdir(parents=True, exist_ok=True)
    figs = DOCS / "figures"
    figs.mkdir(exist_ok=True)
    plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRID, "text.color": INK, "axes.labelcolor": INK2})

    # Figure 1: vehicle-time error by scenario, FIFO vs no FIFO (dot strip)
    fig, ax = plt.subplots(figsize=(7.2, 3.6), facecolor=SURFACE)
    _style(ax)
    for j, (name, col, label) in enumerate((("fifo", COLORS["s1"], "Model with FIFO shared lanes"),
                                            ("no_fifo", COLORS["s2"], "Model without FIFO"))):
        for k, sc in enumerate(dm.SCENARIOS):
            ys = [100 * p["vt_rel_error"] for p in ev["points"][sc][name]]
            xs = [k + (j - 0.5) * 0.28 + (n % 5 - 2) * 0.025 for n in range(len(ys))]
            ax.scatter(xs, ys, s=22, color=col, edgecolors=SURFACE, linewidths=1.2, label=label if k == 0 else None, zorder=3)
    ax.axhline(0, color=INK2, linewidth=1)
    ax.set_xticks(range(len(dm.SCENARIOS)), dm.SCENARIOS)
    ax.set_ylabel("Vehicle-time error over 5 min (%)")
    ax.legend(frameon=False, fontsize=9, loc="upper left")
    fig.tight_layout()
    f1 = figs / "vehicle_time_error.png"
    fig.savefig(f1, dpi=160, facecolor=SURFACE)
    plt.close(fig)

    # Figure 2: I2 shared-lane end queues, predicted vs SUMO
    fig, ax = plt.subplots(figsize=(4.8, 4.4), facecolor=SURFACE)
    _style(ax)
    top = 1
    for name, col, label in (("fifo", COLORS["s1"], "With FIFO"), ("no_fifo", COLORS["s2"], "Without FIFO")):
        xs, ys = [], []
        for sc in dm.SCENARIOS:
            for p in ev["points"][sc][name]:
                for model_q, sumo_q in p["shared"].values():
                    xs.append(sumo_q); ys.append(model_q)
        top = max(top, max(xs), max(ys))
        ax.scatter(xs, ys, s=22, color=col, edgecolors=SURFACE, linewidths=1.2, label=label, zorder=3, alpha=0.9)
    ax.plot([0, top + 2], [0, top + 2], color=INK2, linewidth=1, linestyle=(0, (4, 3)))
    ax.set_xlim(-1, top + 2); ax.set_ylim(-1, top + 2)
    ax.set_xlabel("SUMO mean queue over 5 min (veh)")
    ax.set_ylabel("Predicted mean queue (veh)")
    ax.legend(frameon=False, fontsize=9, loc="upper left")
    fig.tight_layout()
    f2 = figs / "shared_lane_queues.png"
    fig.savefig(f2, dpi=160, facecolor=SURFACE)
    plt.close(fig)

    # Figure 3: example horizon with head-of-line blocking at I2 NB
    spec = load_spec()
    nl = net_lane_lengths(R1_DIR / "r1.net.xml")
    truths = _truths()
    cands = []
    for sc in dm.SCENARIOS:
        for pf, pn in zip(ev["points"][sc]["fifo"], ev["points"][sc]["no_fifo"]):
            for k in ("I2 NB", "I2 SB"):
                fifo_q, sumo_q = pf["shared"][k]
                if 8 <= sumo_q <= 25:
                    cands.append((abs(pn["shared"][k][0] - sumo_q) - abs(fifo_q - sumo_q), sc, pf, k))
    _, sc_ex, p_ex, k_ex = max(cands, key=lambda x: x[0])
    approach = tuple(k_ex.split())
    series = _example_series(truths[sc_ex], p_ex["t0"], spec, nl, ev["params"], approach)
    fig, ax = plt.subplots(figsize=(7.2, 3.6), facecolor=SURFACE)
    _style(ax)
    items = (("sumo", COLORS["s3"], "SUMO"), ("fifo", COLORS["s1"], "Model with FIFO"),
             ("no_fifo", COLORS["s2"], "Model without FIFO"))
    ends = []
    for name, col, label in items:
        xs, ys = zip(*series[name])
        ax.plot(xs, ys, color=col, linewidth=2, label=label)
        ends.append([ys[-1], label])
    # direct end labels, nudged apart so they never overlap
    ends.sort()
    span = max(max(y for _, y in v) for v in series.values()) or 1
    for k in range(1, len(ends)):
        ends[k][0] = max(ends[k][0], ends[k - 1][0] + 0.07 * span)
    for y, label in ends:
        ax.annotate(label, (HORIZON_S, y), xytext=(6, 0), textcoords="offset points", fontsize=8, color=INK2, va="center")
    name = {"NB": "northbound", "SB": "southbound"}[approach[1]]
    ax.set_xlabel(f"Seconds after {int(p_ex['t0'] // 3600):02d}:{int(p_ex['t0'] % 3600 // 60):02d} ({sc_ex})")
    ax.set_ylabel(f"I2 {name} queue (veh)")
    ax.set_xlim(0, HORIZON_S * 1.3)
    ax.legend(frameon=False, fontsize=9, loc="lower left", bbox_to_anchor=(0, 1.0), ncol=3)
    fig.tight_layout()
    f3 = figs / "shared_lane_example.png"
    fig.savefig(f3, dpi=160, facecolor=SURFACE)
    plt.close(fig)

    (DOCS / "prediction_model_m1.json").write_text(json.dumps({"calibration": calib, "evaluation": {
        k: v for k, v in ev.items() if k != "points"}, "points": ev["points"]}, indent=2))
    md = _markdown(calib, ev, (sc_ex, p_ex, k_ex), _spillback(truths))
    path = DOCS / "prediction_model_m1.md"
    path.write_text(md, encoding="utf-8")
    return path


def _pct(x: float) -> str:
    return "0.0" if abs(x) < 0.05 else f"{x:+.1f}"


def _spillback(truths) -> dict:
    """How often the I2 side streets fill their storage, and the largest entry backlog."""
    from collections import Counter

    out = {}
    for sc, t in truths.items():
        full = sum(1 for q in t.queues.values() if max(q[("I2", "NB")], q[("I2", "SB")]) >= 36)
        backlog = Counter()
        for t0 in t.starts:
            for stream, n in Counter(v.split(".")[0] for v in t.snapshots[t0]["pending"]).items():
                backlog[stream] = max(backlog[stream], n)
        out[sc] = {"share_of_time_i2_side_full": round(full / len(t.queues), 3),
                   "max_backlog_by_entry": dict(backlog)}
    return out


def _markdown(calib, ev, example, spill) -> str:
    sc = ev["scenarios"]
    lines = ["# Prediction model validation, Milestone 1", ""]
    lines += ["Generated by `python experiments/validate_prediction.py`. All numbers are in "
              "`prediction_model_m1.json`.", ""]
    f = {k: sc[k]["fifo"]["vehicle_time_rel_error"] for k in sc}
    n = {k: sc[k]["no_fifo"]["vehicle_time_rel_error"] for k in sc}
    lines += ["## Findings", "",
              f"1. **Vehicle-time over a 5 minute horizon is predicted within about 1 % in the loaded scenarios.** With "
              f"FIFO shared lanes the bias is {_pct(f['S1']['bias_pct'])} % (S1), {_pct(f['S2']['bias_pct'])} % (S2) and "
              f"{_pct(f['S3']['bias_pct'])} % (S3), with mean absolute errors of {f['S1']['mean_abs_pct']:.1f}, "
              f"{f['S2']['mean_abs_pct']:.1f} and {f['S3']['mean_abs_pct']:.1f} %. In light traffic (S0) the model "
              f"under-predicts by {abs(f['S0']['bias_pct']):.1f} %, because SUMO vehicles take longer than free-flow time "
              "between signals (acceleration, slower leaders, turning speeds).",
              f"2. **FIFO shared lanes are necessary.** Without them the model under-predicts vehicle-time by "
              f"{abs(n['S1']['bias_pct']):.1f} to {abs(n['S2']['bias_pct']):.1f} % in S1 to S3 and over-predicts I2 "
              f"side-street throughput by {sc['S3']['no_fifo']['mean_queue']['I2 shared lanes']['discharge_bias']:.0f} to "
              f"{sc['S2']['no_fifo']['mean_queue']['I2 shared lanes']['discharge_bias']:.0f} vehicles per approach per "
              "5 minutes; with them, throughput bias is within about 2 vehicles.",
              f"3. **The FIFO model still over-states I2 shared-lane queues** by about "
              f"{sc['S3']['fifo']['mean_queue']['I2 shared lanes']['bias']:.0f} to "
              f"{sc['S2']['fifo']['mean_queue']['I2 shared lanes']['bias']:.0f} vehicles on average: its permissive "
              "left-turners find gaps less readily than SUMO's. About 2 vehicles of every queue difference is "
              "definitional (see below). Individual 5 minute queues are noisy because one left-turner's wait for a gap "
              "decides whether the lane flows.",
              f"4. **Baseline finding for review: I2's side streets are oversaturated in S2.** An I2 side-street queue is at "
              f"or near its full 40-vehicle storage {100 * spill['S2']['share_of_time_i2_side_full']:.0f} % of the time, "
              f"and up to {max(spill['S2']['max_backlog_by_entry'].get(k, 0) for k in ('N2', 'S2'))} vehicles wait "
              "outside the network at an I2 side-street entry. Document Zero intends the I4 eastbound lane drop to be "
              "the bottleneck. The cause is the Milestone 1 combination of one shared lane, a permissive left with no "
              "end-of-phase sneakers, and splits sized from flow ratios that ignore left-turn impedance.", ""]
    lines += ["## What was tested", "",
              "The corridor queue model (`src/cadence/prediction/queue_model.py`) is the simplified model an optimizer "
              "would roll forward. Every 5 minutes after the 15 minute warm-up, in each scenario, it was initialized from "
              "SUMO's true state (every vehicle's position, speed and remaining route, and the insertion backlog) and "
              "predicted the next 5 minutes, given the true future entries and the signal states SUMO actually showed. "
              "Its errors are therefore structural: what the simplification costs, before any estimation error.", "",
              "Two versions are compared: the model with first-in-first-out (FIFO) shared lanes, where a vehicle that "
              "cannot go blocks the lane behind it, and the same model without FIFO, where each movement in a shared "
              "lane queues as if it had its own lane.", "",
              "Measures: system vehicle-time over the 5 minutes (backlog plus network, the quantity in the cost J); the "
              "mean queue on each approach over the 5 minutes; and the number of vehicles discharged by each movement.", "",
              "Queues are not defined identically: SUMO's are halted vehicles (speed below 0.1 m/s) on the approach, "
              "while the model's are all vehicles that have reached the stop line and not yet left, including those "
              "moving off at the start of green. This alone makes model queues about 2 vehicles longer on busy "
              "approaches (the mainline bias in the table).", ""]
    lines += ["## Calibration", "",
              "Only the two permissive-left gap parameters were fitted, on S1, scored on the I2 shared-lane queues; "
              "S0, S2 and S3 are out of sample. Saturation headway (2.0 s), start-up lost time (2 s), yellow use (2 s) "
              "and free-flow speed are fixed at their Document Zero or textbook values.", "",
              "| No opposing departure in the last (s) | No opposing arrival in the next (s) | I2 mean queue MAE (veh) | "
              "I2 mean queue bias (veh) | Vehicle-time mean abs. error |", "| --- | --- | --- | --- | --- |"]
    for g in calib["grid"]:
        mark = " **(chosen)**" if g == calib["chosen"] else ""
        lines.append(f"| {g['gap_before_s']} | {g['gap_after_s']} | {g['shared_queue_mae']}{mark} | "
                     f"{g['shared_queue_bias']:+.2f} | {100 * g['vt_mean_abs_rel_error']:.2f} % |")
    lines += ["", "## Results", "",
              "| Scenario | Model | Starts | Vehicle-time bias | Mean abs. error | 90th pct abs. error | "
              "Mean queue MAE (veh): mainline | side, own lanes | I2 shared lanes | I2 shared-lane throughput bias (veh) | "
              "Discharges per movement MAE (veh) |",
              "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for sc, by in ev["scenarios"].items():
        for name, label in (("fifo", "FIFO"), ("no_fifo", "no FIFO")):
            r = by[name]
            v = r["vehicle_time_rel_error"]
            q = r["mean_queue"]
            lines.append(f"| {sc} | {label} | {r['starts']} | {v['bias_pct']:+.2f} % | {v['mean_abs_pct']:.2f} % | "
                         f"{v['p90_abs_pct']:.2f} % | {q['mainline']['mae']} | {q['side (own lanes)']['mae']} | "
                         f"{q['I2 shared lanes']['mae']} | {q['I2 shared lanes']['discharge_bias']:+.1f} | "
                         f"{r['discharge_per_movement_mae']} |")
    lines += ["", "![Vehicle-time error by scenario](figures/vehicle_time_error.png)", "",
              "*Each dot is one 5 minute prediction. Error = (model − SUMO) / SUMO.*", "",
              "## Shared-lane head-of-line blocking", "",
              "I2's side streets have one lane for left, through and right traffic, and the left turn is permissive. "
              "In SUMO a left-turner waiting for a gap holds up everything behind it. The figure compares the mean I2 "
              "side-street queue over each 5 minute horizon. Points along the top are horizons in which the 40-vehicle "
              "side-street storage was full.", "",
              "![I2 shared-lane queues](figures/shared_lane_queues.png)", "",
              f"*Each dot is one approach (I2 northbound or southbound) at one start time; the dashed line is perfect "
              f"prediction.* Below, the horizon (among those where the side street was not full) where FIFO made the "
              f"largest difference ({example[0]}, I2 {example[2].split()[1]}).", "",
              "![Example of head-of-line blocking](figures/shared_lane_example.png)", ""]
    lines += ["## I2 side-street spillback by scenario", "",
              "| Scenario | Share of time an I2 side-street queue is 36 vehicles or more | Largest entry backlog (vehicles, by entry) |",
              "| --- | --- | --- |"]
    for k, v in spill.items():
        b = ", ".join(f"{e} {m}" for e, m in sorted(v["max_backlog_by_entry"].items())) or "none"
        lines.append(f"| {k} | {100 * v['share_of_time_i2_side_full']:.0f} % | {b} |")
    lines += ["", "In S3 the backlog at I3 northbound (stream S3 and the pulse) is the intended Document Zero disruption. "
              "Options for the I2 side streets, for review: size the I2 side splits for the shared lane's real capacity, "
              "lower the Milestone 1 I2 cross-street volumes within the Document Zero range, or restore end-of-phase "
              "left turns once the junction-clearing issue in corridor_r1.md is solved another way.", ""]
    lines += ["## Known structural limits", "",
              "- Point queues: vehicles join a queue when they reach the stop line, not when they reach the back of the "
              "queue, so delay upstream of long queues is shifted in time.",
              "- One saturation headway (2.0 s) for every lane and movement; turning movements in SUMO are slower.",
              "- Free-flow speed is the posted speed for every vehicle; SUMO drivers vary their desired speed.",
              "- Permissive lefts use a fixed gap rule; SUMO's gap acceptance depends on the approaching vehicles' "
              "speeds and distances.",
              "- Left-turn bay overflow is represented by a left-turner blocking the adjacent through lane once it "
              "cannot enter the bay; the exact position where it waits is not modeled.", ""]
    return "\n".join(lines) + "\n"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage", choices=("all", "collect", "calibrate", "evaluate", "report"), default="all")
    args = ap.parse_args()
    RUNS.mkdir(parents=True, exist_ok=True)
    if args.stage in ("all", "collect"):
        stage_collect()
        if args.stage == "collect":
            return
    calib = stage_calibrate() if args.stage in ("all", "calibrate") else json.loads((RUNS / "calibration.json").read_text())
    if args.stage in ("all", "calibrate", "evaluate"):
        ev = stage_evaluate(calib["chosen"])
    else:
        ev = json.loads((RUNS / "evaluation.json").read_text())
    if args.stage in ("all", "report", "evaluate", "calibrate"):
        print(stage_report(calib, ev))


if __name__ == "__main__":
    main()
