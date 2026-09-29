#!/usr/bin/env python3
"""
FAST-LIVO2 trajectory evaluation & plotting
===========================================

Evaluate one or more estimated trajectories per sequence against a reference
(ground truth), compute ATE / RPE (translation part, SE(3) Umeyama alignment),
draw evo-style RPE-coloured 3D trajectory plots, and write a comparison table
with the relative change of every run w.r.t. a baseline run.

Typical use — two sequences, each with an "Original" (upstream) and a
"Modified" run, evaluated against total-station ground truth:

    python3 tool/eval_trajectory.py --out tool/eval_results/report \
        --seq Tunnel4 tool/eval_results/tunnel4_gt.txt \
              Original=tool/eval_results/tunnel4_original.txt \
              Modified=tool/eval_results/tunnel4_modified.txt \
        --seq Tunnel5 tool/eval_results/tunnel5_gt.txt \
              Original=tool/eval_results/tunnel5_original.txt \
              Modified=tool/eval_results/tunnel5_modified.txt \
        --baseline Original

Each --seq takes:  <sequence name>  <reference tum file>  <label>=<est tum file> ...

Outputs (in --out):
    <seq>_<label>_RPE.png        RPE-coloured 3D trajectory (one per run)
    <seq>_<label>_ATE.png        ATE-coloured 3D trajectory (one per run)
    grid_RPE.png                 all RPE plots tiled (rows = sequences, cols = runs)
    metrics.csv                  one row per (sequence, run)
    metrics.md                   markdown table incl. relative change vs --baseline
    metrics_summary.txt          human-readable dump

All trajectory files are TUM format: "t x y z qx qy qz qw" per line.

Position-only ground truth (prism / total station, quaternion written as
0 0 0 1) is detected automatically. In that case the estimate's orientations
are discarded before computing RPE, so RPE becomes the pure position drift
|(p_est_j - p_est_i) - (p_ref_j - p_ref_i)| over `delta` metres of path.
Without this, evo expresses the estimate's relative translation in the
estimate's body frame and the reference's in the world frame, which is
meaningless (errors approach 2*delta on the return leg of an out-and-back
tunnel run). Force the behaviour with --position_only.
"""

import argparse
import copy
import csv
import io
import os
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                     # noqa: E402
from matplotlib import cm                           # noqa: E402
from matplotlib.colors import Normalize             # noqa: E402
from matplotlib.ticker import MaxNLocator           # noqa: E402
from mpl_toolkits.mplot3d.art3d import Line3DCollection  # noqa: E402

from evo.core import metrics, sync                  # noqa: E402
from evo.core.metrics import PoseRelation, Unit     # noqa: E402
from evo.core.trajectory import PoseTrajectory3D    # noqa: E402
from evo.tools import file_interface                # noqa: E402


# --------------------------------------------------------------------------- #
# metrics
# --------------------------------------------------------------------------- #

def is_position_only(traj, tol=1e-6):
    """True if every orientation is the identity quaternion."""
    q = traj.orientations_quat_wxyz
    return bool(np.all(np.abs(np.abs(q[:, 0]) - 1.0) < tol) and
                np.all(np.abs(q[:, 1:]) < tol))


def strip_orientation(traj):
    q = np.tile([1.0, 0.0, 0.0, 0.0], (traj.num_poses, 1))
    return PoseTrajectory3D(traj.positions_xyz, q, traj.timestamps)


def shift_time(traj, dt):
    return PoseTrajectory3D(traj.positions_xyz, traj.orientations_quat_wxyz,
                            traj.timestamps + dt)


def evaluate(ref_file, est_file, rpe_delta=0.2, max_diff=0.05,
             correct_scale=False, position_only=None, t_offset=0.0):
    """
    Load ref/est, associate by timestamp, SE(3)-align est to ref, compute
    ATE + RPE on the translation part.

    position_only: None = auto-detect from the reference orientations.
    t_offset:      seconds added to the estimate's timestamps before association.

    Returns dict with the aligned trajectories, evo Result objects, per-pose
    error arrays for plotting, and a flat `stats` dict.
    """
    traj_ref = file_interface.read_tum_trajectory_file(ref_file)
    traj_est = file_interface.read_tum_trajectory_file(est_file)
    if t_offset:
        traj_est = shift_time(traj_est, t_offset)

    if position_only is None:
        position_only = is_position_only(traj_ref)

    ref, est = sync.associate_trajectories(traj_ref, traj_est, max_diff=max_diff)
    if ref.num_poses < 10:
        raise RuntimeError(
            f"only {ref.num_poses} associated poses between\n  {ref_file}\n  {est_file}\n"
            f"(ref {traj_ref.timestamps[0]:.2f}-{traj_ref.timestamps[-1]:.2f}, "
            f"est {traj_est.timestamps[0]:.2f}-{traj_est.timestamps[-1]:.2f}, "
            f"max_diff={max_diff})")

    est_aligned = copy.deepcopy(est)
    est_aligned.align(ref, correct_scale=correct_scale)   # Umeyama on positions

    if position_only:
        ref = strip_orientation(ref)
        est_aligned = strip_orientation(est_aligned)

    ape = metrics.APE(PoseRelation.translation_part)
    ape.process_data((ref, est_aligned))
    ape_res = ape.get_result()

    rpe = metrics.RPE(PoseRelation.translation_part, delta=rpe_delta,
                      delta_unit=Unit.meters, all_pairs=False)
    rpe.process_data((ref, est_aligned))
    rpe_res = rpe.get_result()

    # per-pose error arrays for colouring the trajectory
    n = ref.num_poses
    ate_per_pose = ape_res.np_arrays["error_array"]
    rpe_err = rpe_res.np_arrays["error_array"]
    # evo picks pairs (i, j) along the path; delta_ids are the j indices.
    # Assign each pair's error to pose j and hold it until the next pair.
    rpe_per_pose = np.full(n, np.nan)
    rpe_per_pose[np.asarray(rpe.delta_ids, dtype=int)] = rpe_err
    idx = np.arange(n)
    valid = ~np.isnan(rpe_per_pose)
    rpe_per_pose = np.interp(idx, idx[valid], rpe_per_pose[valid])

    def flat(prefix, res):
        return {f"{prefix}_{k}": float(v) for k, v in res.stats.items()}

    stats = {
        "position_only": position_only,
        "t_offset_s": t_offset,
        "n_ref_total": traj_ref.num_poses,
        "n_est_total": traj_est.num_poses,
        "n_assoc": n,
        "n_rpe_pairs": len(rpe_err),
        "duration_s": float(ref.timestamps[-1] - ref.timestamps[0]),
        "path_len_ref_m": float(ref.path_length),
        "path_len_est_m": float(est_aligned.path_length),
    }
    stats.update(flat("ate", ape_res))
    stats.update(flat("rpe", rpe_res))

    return {"ref": ref, "est": est_aligned, "ape": ape_res, "rpe": rpe_res,
            "ate_per_pose": ate_per_pose, "rpe_per_pose": rpe_per_pose,
            "stats": stats}


# --------------------------------------------------------------------------- #
# plotting
# --------------------------------------------------------------------------- #

def _fit_axes(ax, pts, min_ratio=0.25, margin=0.05, zoom=1.0):
    """
    Limits = data bounding box (+margin); box aspect proportional to the data
    extents so a long tunnel is drawn long, but no axis thinner than
    `min_ratio` of the longest so z stays readable.
    """
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    span = np.maximum(hi - lo, 1e-6)
    pad = span * margin
    ax.set_xlim3d(lo[0] - pad[0], hi[0] + pad[0])
    ax.set_ylim3d(lo[1] - pad[1], hi[1] + pad[1])
    ax.set_zlim3d(lo[2] - pad[2], hi[2] + pad[2])
    aspect = span / span.max()
    aspect = np.maximum(aspect, min_ratio)
    ax.set_box_aspect(tuple(aspect), zoom=zoom)
    # few ticks on the short axes, otherwise labels pile up
    for axis, a in zip((ax.xaxis, ax.yaxis, ax.zaxis), aspect):
        axis.set_major_locator(MaxNLocator(nbins=8 if a > 0.6 else 4))


def auto_azim(pts):
    """
    Azimuth that puts the trajectory's principal horizontal direction across
    the screen (left→right), so a long straight tunnel is drawn long instead of
    end-on.
    """
    xy = pts[:, :2] - pts[:, :2].mean(axis=0)
    _, _, vt = np.linalg.svd(xy, full_matrices=False)
    ang = np.degrees(np.arctan2(vt[0, 1], vt[0, 0]))   # principal axis angle
    return ang - 90.0


def draw_error_3d(fig, ax, ref, est, err_per_pose, title, ref_label="reference",
                  cmap="jet", elev=28, azim=None, cb_shrink=0.7, cb_pad=0.12,
                  lw=2.0, cb_fraction=0.15, min_ratio=0.25, zoom=1.0):
    """
    Draw an evo-style error-coloured trajectory into an existing 3D axes:
    reference dashed grey, estimate as line segments coloured by
    `err_per_pose` (one value per estimate pose; segment i takes the value of
    pose i), plus a colorbar.  azim=None → chosen from the trajectory's
    principal direction.
    """
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.set_facecolor("#eaeaf2")
        pane.set_edgecolor("white")
    ax.grid(True, color="white", linewidth=0.8)

    r = ref.positions_xyz
    ax.plot(r[:, 0], r[:, 1], r[:, 2], "--", color="0.45", lw=1.2, label=ref_label)

    p = est.positions_xyz
    e = np.asarray(err_per_pose, dtype=float)
    assert len(e) == len(p), "err_per_pose must have one entry per estimate pose"
    segs = np.stack([p[:-1], p[1:]], axis=1)

    norm = Normalize(vmin=e.min(), vmax=e.max())
    lc = Line3DCollection(segs, cmap=cmap, norm=norm, linewidths=lw)
    lc.set_array(e[:-1])
    ax.add_collection3d(lc)

    sm = cm.ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array(e)
    cb = fig.colorbar(sm, ax=ax, shrink=cb_shrink, pad=cb_pad, aspect=25,
                      fraction=cb_fraction)
    cb.set_ticks([e.min(), (e.min() + e.max()) / 2, e.max()])
    cb.ax.set_yticklabels([f"{v:.3f}" for v in cb.get_ticks()], fontsize=8)

    ax.set_xlabel("x (m)", fontsize=9, labelpad=4)
    ax.set_ylabel("y (m)", fontsize=9, labelpad=4)
    ax.zaxis.set_rotate_label(False)
    ax.set_zlabel("z (m)", fontsize=9, labelpad=4, rotation=90)
    ax.tick_params(labelsize=7)
    ax.set_title(title, fontsize=9.5, pad=6)
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    allpts = np.vstack([r, p])
    ax.view_init(elev=elev, azim=auto_azim(allpts) if azim is None else azim)
    _fit_axes(ax, allpts, min_ratio=min_ratio, zoom=zoom)
    return cb


def plot_error_3d(ref, est, err_per_pose, title, suptitle=None,
                  ref_label="reference", cmap="jet", elev=28, azim=None,
                  figsize=(7.2, 6.4)):
    """Single-panel figure wrapper around draw_error_3d."""
    fig = plt.figure(figsize=figsize, dpi=150)
    ax = fig.add_subplot(111, projection="3d")
    draw_error_3d(fig, ax, ref, est, err_per_pose, title, ref_label=ref_label,
                  cmap=cmap, elev=elev, azim=azim)
    if suptitle:
        fig.suptitle(suptitle, fontsize=11, fontweight="bold", y=0.985)
    fig.tight_layout()
    return fig


# --------------------------------------------------------------------------- #
# paper-style outputs: (a)(b)(c) panel figure + metric table
# --------------------------------------------------------------------------- #

def setup_cjk_font():
    """Prepend an available CJK font to the sans-serif fallback list."""
    import matplotlib.font_manager as fm
    have = {f.name for f in fm.fontManager.ttflist}
    for cand in ("Noto Sans CJK SC", "Noto Sans CJK JP", "Source Han Sans SC",
                 "WenQuanYi Micro Hei", "WenQuanYi Zen Hei", "Microsoft YaHei",
                 "SimHei", "AR PL UMing CN", "Droid Sans Fallback"):
        if cand in have:
            plt.rcParams["font.sans-serif"] = [cand] + list(plt.rcParams["font.sans-serif"])
            plt.rcParams["axes.unicode_minus"] = False
            return cand
    return None


def plot_paper_panels(panels, captions, ref_label="reference", elev=28, azim=None,
                      panel_w=6.2, panel_h=5.6, title_fn=None):
    """
    panels:   list of (ref, est, err_per_pose) — one per sequence, drawn left→right
    captions: list of caption strings, prefixed with (a) (b) (c) ... automatically
    """
    n = len(panels)
    fig = plt.figure(figsize=(panel_w * n, panel_h), dpi=200)
    for i, ((ref, est, err), cap) in enumerate(zip(panels, captions)):
        ax = fig.add_subplot(1, n, i + 1, projection="3d")
        draw_error_3d(fig, ax, ref, est, err, title_fn() if title_fn else "",
                      ref_label=ref_label, elev=elev, azim=azim,
                      cb_shrink=0.62, cb_pad=0.06, cb_fraction=0.08, lw=1.8,
                      min_ratio=0.35, zoom=1.15)
        ax.legend(loc="upper right", bbox_to_anchor=(0.92, 0.86), fontsize=8, framealpha=0.9)
        ax.text2D(0.5, -0.06, f"({chr(ord('a') + i)}) {cap}", transform=ax.transAxes,
                  ha="center", va="top", fontsize=12)
    fig.subplots_adjust(left=0.01, right=0.99, top=0.95, bottom=0.10, wspace=0.02)
    return fig


def write_paper_table(rows, seqs, run_labels, display, metric, metric_title, out_base):
    """
    One row per sequence, one column per run.  Writes .md, .tex and .png.
    """
    hdr = [""] + [display.get(r, r) for r in run_labels]
    body = []
    for seq in seqs:
        cells = [seq]
        for r in run_labels:
            row = next((x for x in rows if x["sequence"] == seq and x["run"] == r), None)
            cells.append(f"{row[metric]:.4f}" if row else "–")
        body.append(cells)

    # markdown
    with open(out_base + ".md", "w") as f:
        f.write(f"**{metric_title}**\n\n")
        f.write("| " + " | ".join(hdr) + " |\n|" + "---|" * len(hdr) + "\n")
        for cells in body:
            f.write("| " + " | ".join(cells) + " |\n")

    # latex
    with open(out_base + ".tex", "w") as f:
        f.write("\\begin{table}[htbp]\n\\centering\n")
        f.write(f"\\caption{{{metric_title}}}\n")
        f.write("\\begin{tabular}{l" + "c" * len(run_labels) + "}\n\\toprule\n")
        f.write(" & ".join(hdr) + " \\\\\n\\midrule\n")
        for cells in body:
            f.write(" & ".join(cells) + " \\\\\n")
        f.write("\\bottomrule\n\\end{tabular}\n\\end{table}\n")

    # png
    fig, ax = plt.subplots(figsize=(2.2 + 2.0 * len(run_labels), 0.55 * (len(body) + 2)), dpi=200)
    ax.axis("off")
    ax.set_title(metric_title, fontsize=11, loc="left", pad=8)
    tbl = ax.table(cellText=body, colLabels=hdr, loc="center", cellLoc="center",
                   colLoc="center", edges="horizontal")
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(10.5)
    tbl.scale(1.0, 1.5)
    for (r_, c_), cell in tbl.get_celld().items():
        cell.set_linewidth(0.8 if r_ in (0, len(body)) else 0.0)
        if r_ == 0:
            cell.set_text_props(fontweight="normal")
        if c_ == 0:
            cell.set_text_props(ha="left")
    fig.tight_layout()
    fig.savefig(out_base + ".png", bbox_inches="tight")
    plt.close(fig)


def rpe_title(delta):
    return (f"RPE w.r.t. translation part (m)\n"
            f"for delta = {delta:g} (m) using consecutive pairs\n"
            f"(with SE(3) Umeyama alignment)")


ATE_TITLE = ("APE w.r.t. translation part (m)\n"
             "(with SE(3) Umeyama alignment)")


def tile_pngs(paths, ncols, out_path, pad=10):
    """Tile PNG files into one image (rows filled left→right)."""
    from PIL import Image
    imgs = [Image.open(p).convert("RGB") for p in paths]
    w = max(i.width for i in imgs)
    h = max(i.height for i in imgs)
    nrows = (len(imgs) + ncols - 1) // ncols
    canvas = Image.new("RGB", (ncols * w + (ncols + 1) * pad,
                               nrows * h + (nrows + 1) * pad), "white")
    for k, im in enumerate(imgs):
        rr, cc = divmod(k, ncols)
        x = pad + cc * (w + pad) + (w - im.width) // 2
        y = pad + rr * (h + pad) + (h - im.height) // 2
        canvas.paste(im, (x, y))
    canvas.save(out_path)


# --------------------------------------------------------------------------- #
# reporting
# --------------------------------------------------------------------------- #

METRIC_COLS = [
    ("ate_rmse", "ATE RMSE"),
    ("ate_mean", "ATE mean"),
    ("ate_median", "ATE median"),
    ("ate_std", "ATE std"),
    ("ate_max", "ATE max"),
    ("rpe_rmse", "RPE RMSE"),
    ("rpe_mean", "RPE mean"),
    ("rpe_median", "RPE median"),
    ("rpe_std", "RPE std"),
    ("rpe_max", "RPE max"),
]


def rel_change(new, base):
    """Signed relative change in %, negative = error decreased (better)."""
    if base == 0:
        return float("nan")
    return (new - base) / base * 100.0


def write_csv(rows, path):
    keys = ["sequence", "run"] + [k for k, _ in METRIC_COLS] + \
           ["n_assoc", "duration_s", "path_len_ref_m", "path_len_est_m"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)


def write_markdown(rows, baseline, rpe_delta, path):
    seqs = []
    for r in rows:
        if r["sequence"] not in seqs:
            seqs.append(r["sequence"])

    out = io.StringIO()
    out.write(f"# Trajectory evaluation (translation part, SE(3) Umeyama alignment, "
              f"RPE delta = {rpe_delta:g} m)\n\n")
    out.write("All errors in metres. Relative change = (run − baseline) / baseline; "
              "negative means the error went **down**.\n\n")
    if rows and rows[0].get("position_only"):
        out.write("Reference is position-only (identity orientations): RPE is computed on "
                  "positions only, i.e. the drift of the relative displacement over "
                  f"{rpe_delta:g} m of path.\n\n")
    if rows and rows[0].get("t_offset_s"):
        out.write(f"Estimate timestamps shifted by {rows[0]['t_offset_s']:+.3f} s "
                  "before association.\n\n")

    for seq in seqs:
        srows = [r for r in rows if r["sequence"] == seq]
        base = next((r for r in srows if r["run"] == baseline), None)

        out.write(f"## {seq}\n\n")
        n = srows[0]
        out.write(f"associated poses: {n['n_assoc']}, duration: {n['duration_s']:.1f} s, "
                  f"reference path length: {n['path_len_ref_m']:.2f} m\n\n")

        hdr = ["metric"] + [r["run"] for r in srows]
        if base is not None:
            hdr += [f"Δ {r['run']} vs {baseline}" for r in srows if r["run"] != baseline]
        out.write("| " + " | ".join(hdr) + " |\n")
        out.write("|" + "---|" * len(hdr) + "\n")

        for key, label in METRIC_COLS:
            cells = [label] + [f"{r[key]:.4f}" for r in srows]
            if base is not None:
                for r in srows:
                    if r["run"] == baseline:
                        continue
                    d = rel_change(r[key], base[key])
                    cells.append(f"{d:+.1f} %")
            out.write("| " + " | ".join(cells) + " |\n")
        out.write("\n")

    # compact cross-sequence summary of the headline numbers
    out.write("## Summary (RMSE)\n\n")
    hdr = ["sequence", "run", "ATE RMSE", "RPE RMSE"]
    if baseline:
        hdr += [f"Δ ATE vs {baseline}", f"Δ RPE vs {baseline}"]
    out.write("| " + " | ".join(hdr) + " |\n|" + "---|" * len(hdr) + "\n")
    for seq in seqs:
        srows = [r for r in rows if r["sequence"] == seq]
        base = next((r for r in srows if r["run"] == baseline), None)
        for r in srows:
            cells = [seq, r["run"], f"{r['ate_rmse']:.4f}", f"{r['rpe_rmse']:.4f}"]
            if baseline:
                if base is None or r["run"] == baseline:
                    cells += ["–", "–"]
                else:
                    cells += [f"{rel_change(r['ate_rmse'], base['ate_rmse']):+.1f} %",
                              f"{rel_change(r['rpe_rmse'], base['rpe_rmse']):+.1f} %"]
            out.write("| " + " | ".join(cells) + " |\n")

    with open(path, "w") as f:
        f.write(out.getvalue())
    return out.getvalue()


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #

def parse_seq_arg(tokens):
    if len(tokens) < 3:
        raise argparse.ArgumentTypeError(
            "--seq needs: <name> <ref.tum> <label>=<est.tum> [...]")
    name, ref = tokens[0], tokens[1]
    runs = []
    for tok in tokens[2:]:
        if "=" not in tok:
            raise argparse.ArgumentTypeError(f"bad run spec '{tok}', expected label=file")
        label, path = tok.split("=", 1)
        runs.append((label, path))
    return name, ref, runs


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seq", nargs="+", action="append", required=True,
                    metavar="TOK",
                    help="<name> <ref.tum> <label>=<est.tum> [<label>=<est.tum> ...]; repeatable")
    ap.add_argument("--baseline", default=None,
                    help="run label used as baseline for relative change (default: first run)")
    ap.add_argument("--out", default="eval_output", help="output directory")
    ap.add_argument("--rpe_delta", type=float, default=0.2, help="RPE delta in metres")
    ap.add_argument("--max_diff", type=float, default=0.05,
                    help="max timestamp difference for ref/est association (s)")
    ap.add_argument("--t_offset", type=float, default=0.0,
                    help="seconds added to every estimate's timestamps before association "
                         "(clock offset compensation; applied identically to all runs)")
    ap.add_argument("--position_only", action="store_true",
                    help="force position-only RPE (auto-detected when the reference has "
                         "identity orientations)")
    ap.add_argument("--ref_label", default="reference", help="legend label for the reference")
    ap.add_argument("--no_ate_plots", action="store_true")
    ap.add_argument("--elev", type=float, default=28, help="camera elevation (deg)")
    ap.add_argument("--azim", type=float, default=None,
                    help="camera azimuth (deg); default: auto from trajectory principal axis")
    # paper-style outputs
    ap.add_argument("--paper", action="store_true",
                    help="also write paper-style outputs: one (a)(b)(c)… panel figure per run "
                         "(paper_<run>_RPE.png) and RPE/ATE RMSE tables (paper_table_*.md/.tex/.png)")
    ap.add_argument("--caption", action="append", default=[], metavar="SEQ=TEXT",
                    help="panel caption for a sequence (default '<seq> 序列定位轨迹'); repeatable")
    ap.add_argument("--display", action="append", default=[], metavar="RUN=TEXT",
                    help="table column header for a run label, e.g. Original=FAST-LIVO2 "
                         "Modified=本文方法; repeatable")
    ap.add_argument("--table_title", default=None,
                    help="table title (default: '在 GEODE 数据集序列上 RPE 指标(RMSE)')")
    args = ap.parse_args()

    sequences = [parse_seq_arg(t) for t in args.seq]
    os.makedirs(args.out, exist_ok=True)

    if args.paper:
        font = setup_cjk_font()
        print(f"CJK font: {font or 'none found — Chinese captions will not render'}")

    rows = []
    rpe_pngs = []
    results = {}            # (seq, run) -> evaluate() result, for paper panels
    ncols = max(len(runs) for _, _, runs in sequences)

    for seq_name, ref_file, runs in sequences:
        for label, est_file in runs:
            print(f"[{seq_name} / {label}] {est_file}  vs  {ref_file}")
            res = evaluate(ref_file, est_file, rpe_delta=args.rpe_delta,
                           max_diff=args.max_diff,
                           position_only=True if args.position_only else None,
                           t_offset=args.t_offset)
            st = res["stats"]
            print(f"    assoc={st['n_assoc']}  rpe_pairs={st['n_rpe_pairs']}  "
                  f"position_only={st['position_only']}  "
                  f"ATE rmse={st['ate_rmse']:.4f}  RPE rmse={st['rpe_rmse']:.4f}")

            row = {"sequence": seq_name, "run": label}
            row.update(st)
            rows.append(row)
            results[(seq_name, label)] = res

            sup = f"{seq_name} — {label}"
            fig = plot_error_3d(res["ref"], res["est"], res["rpe_per_pose"],
                                rpe_title(args.rpe_delta), suptitle=sup,
                                ref_label=args.ref_label, elev=args.elev, azim=args.azim)
            p = os.path.join(args.out, f"{seq_name}_{label}_RPE.png")
            fig.savefig(p, bbox_inches="tight")
            plt.close(fig)
            rpe_pngs.append(p)

            if not args.no_ate_plots:
                fig = plot_error_3d(res["ref"], res["est"], res["ate_per_pose"],
                                    ATE_TITLE, suptitle=sup,
                                    ref_label=args.ref_label, elev=args.elev, azim=args.azim)
                fig.savefig(os.path.join(args.out, f"{seq_name}_{label}_ATE.png"),
                            bbox_inches="tight")
                plt.close(fig)

        # pad the row so the grid stays rectangular
        rpe_pngs += [None] * (ncols - len(runs))

    baseline = args.baseline or sequences[0][2][0][0]

    write_csv(rows, os.path.join(args.out, "metrics.csv"))
    md = write_markdown(rows, baseline, args.rpe_delta, os.path.join(args.out, "metrics.md"))
    with open(os.path.join(args.out, "metrics_summary.txt"), "w") as f:
        f.write(md)
    print("\n" + md)

    grid_inputs = [p for p in rpe_pngs if p]
    if len(grid_inputs) > 1:
        try:
            tile_pngs(grid_inputs, ncols, os.path.join(args.out, "grid_RPE.png"))
            print(f"grid -> {os.path.join(args.out, 'grid_RPE.png')}")
        except ImportError:
            print("PIL not available, skipping grid_RPE.png", file=sys.stderr)

    # ---- paper-style outputs -------------------------------------------------
    if args.paper:
        seq_names = [s for s, _, _ in sequences]
        run_labels = []
        for _, _, runs in sequences:
            for lab, _ in runs:
                if lab not in run_labels:
                    run_labels.append(lab)

        captions = dict(kv.split("=", 1) for kv in args.caption)
        display = dict(kv.split("=", 1) for kv in args.display)

        for lab in run_labels:
            panels, caps = [], []
            for seq in seq_names:
                res = results.get((seq, lab))
                if res is None:
                    continue
                panels.append((res["ref"], res["est"], res["rpe_per_pose"]))
                caps.append(captions.get(seq, f"{seq} 序列定位轨迹"))
            if not panels:
                continue
            fig = plot_paper_panels(panels, caps, ref_label=args.ref_label,
                                    elev=args.elev, azim=args.azim,
                                    title_fn=lambda: rpe_title(args.rpe_delta))
            p = os.path.join(args.out, f"paper_{lab}_RPE.png")
            fig.savefig(p, bbox_inches="tight")
            plt.close(fig)
            print(f"paper figure -> {p}")

        title_rpe = args.table_title or "在 GEODE 数据集序列上 RPE 指标(RMSE)"
        write_paper_table(rows, seq_names, run_labels, display, "rpe_rmse", title_rpe,
                          os.path.join(args.out, "paper_table_RPE"))
        title_ate = (args.table_title.replace("RPE", "ATE") if args.table_title
                     else "在 GEODE 数据集序列上 ATE 指标(RMSE)")
        write_paper_table(rows, seq_names, run_labels, display, "ate_rmse", title_ate,
                          os.path.join(args.out, "paper_table_ATE"))
        print(f"paper tables -> {os.path.join(args.out, 'paper_table_{RPE,ATE}.{md,tex,png}')}")

    print(f"\nall outputs in {os.path.abspath(args.out)}")


if __name__ == "__main__":
    main()
