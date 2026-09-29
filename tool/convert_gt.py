#!/usr/bin/env python3
"""
Convert GEODE prism / total-station ground-truth files to clean TUM format for evo.

GEODE GT files (e.g. /home/gh/data/GEODE/Tunneling_tunnel4.txt, Shield_tunnel7.txt)
are 8-column, space-separated:

    timestamp  x  y  z  0 0 0 0

i.e. TUM layout, but the orientation columns are all zero because the GT comes
from a tracked prism (position only). `0 0 0 0` is not a valid quaternion, so
we rewrite it as identity `0 0 0 1`. Positions are kept verbatim (they carry a
large total-station frame offset, e.g. ~(1000, 2000, 11)); use `evo ... -a`
(Umeyama alignment) or `--align_origin` when comparing. Since orientation is
synthetic, evaluate with `--pose_relation trans_part` (or point_distance) only.

Also handled generically (not needed for the current files, but cheap):
  - comment / header lines (start with '#' or contain non-numeric tokens) are skipped
  - comma-separated input is accepted
  - 4-column position-only input (t x y z) is accepted
  - nanosecond timestamps (> 1e12) are converted to seconds
  - a quaternion that is already valid (|q| ~ 1) is kept as-is
  - duplicate / non-monotonic timestamps are dropped with a warning

Usage:
    python3 convert_gt.py                       # converts the two default Tunneling files
    python3 convert_gt.py IN.txt OUT.txt        # convert one file
    python3 convert_gt.py IN.txt OUT.txt --bag-start S --bag-end E   # also report overlap
"""
import argparse
import math
import os
import sys

DEFAULT_JOBS = [
    # (input, output, bag_start, bag_end)  -- bag ranges read from `rosbag info`
    ("/home/gh/data/GEODE/Tunneling_tunnel4.txt",
     "/home/gh/fastlivo2_ws/src/FAST-LIVO2/tool/eval_results/tunnel4_gt.txt",
     1706585253.489917, 1706585485.480262),
    ("/home/gh/data/GEODE/Tunneling_tunnel5.txt",
     "/home/gh/fastlivo2_ws/src/FAST-LIVO2/tool/eval_results/tunnel5_gt.txt",
     1706585926.126528, 1706586149.275601),
]


def _is_number(tok):
    try:
        v = float(tok)
    except ValueError:
        return False
    return math.isfinite(v)


def convert(in_path, out_path, bag_start=None, bag_end=None):
    rows = []          # (t_float, t_str, x_str, y_str, z_str, q_strs)
    n_skipped_header = n_bad = n_nan = 0

    with open(in_path, "r") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                n_skipped_header += 1
                continue
            toks = line.replace(",", " ").split()
            if len(toks) not in (4, 8):
                n_bad += 1
                print(f"  [warn] {in_path}:{lineno}: {len(toks)} columns, skipped")
                continue
            if not all(_is_number(t) for t in toks):
                # a header row such as "timestamp x y z ..." lands here, as do NaN/inf rows
                if any(t.lower() in ("nan", "inf", "-inf") for t in toks):
                    n_nan += 1
                    print(f"  [warn] {in_path}:{lineno}: NaN/inf value, skipped")
                else:
                    n_skipped_header += 1
                continue

            t_str = toks[0]
            t = float(t_str)
            if t > 1e12:                     # nanoseconds -> seconds
                t = t / 1e9
                t_str = f"{t:.9f}"
            x_str, y_str, z_str = toks[1:4]

            q = [float(v) for v in toks[4:8]] if len(toks) == 8 else [0.0, 0.0, 0.0, 0.0]
            qn = math.sqrt(sum(v * v for v in q))
            if abs(qn - 1.0) < 1e-3:
                q_strs = toks[4:8]           # already a valid quaternion: keep verbatim
            else:
                q_strs = ["0", "0", "0", "1"]  # zero / invalid quaternion -> identity
            rows.append((t, t_str, x_str, y_str, z_str, q_strs))

    n_in = len(rows)
    # drop duplicate / backwards timestamps (evo requires strictly increasing time)
    clean, last_t, n_dup, n_back = [], None, 0, 0
    for r in rows:
        if last_t is not None and r[0] == last_t:
            n_dup += 1
            continue
        if last_t is not None and r[0] < last_t:
            n_back += 1
            continue
        clean.append(r)
        last_t = r[0]

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        for _, t_str, x, y, z, q in clean:
            f.write(f"{t_str} {x} {y} {z} {' '.join(q)}\n")

    # ---- summary ---------------------------------------------------------
    ts = [r[0] for r in clean]
    pts = [(float(r[2]), float(r[3]), float(r[4])) for r in clean]
    path_len = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
    dts = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
    gaps = [(ts[i], ts[i + 1], dts[i]) for i in range(len(dts)) if dts[i] > 1.0]
    orient_synthetic = all(r[5] == ["0", "0", "0", "1"] for r in clean)

    print(f"{in_path}\n  -> {out_path}")
    print(f"  rows read={n_in} written={len(clean)} "
          f"(header/comment skipped={n_skipped_header}, bad-cols={n_bad}, nan={n_nan}, "
          f"dup-ts dropped={n_dup}, backwards-ts dropped={n_back})")
    print(f"  time: {ts[0]:.6f} -> {ts[-1]:.6f}  duration={ts[-1]-ts[0]:.2f}s  "
          f"mean rate={len(ts)/(ts[-1]-ts[0]):.2f} Hz  median dt={sorted(dts)[len(dts)//2]:.3f}s")
    print(f"  gaps >1s: {len(gaps)}" + "".join(f"\n     {a:.3f} -> {b:.3f} ({d:.2f}s)" for a, b, d in gaps))
    mn = [min(p[i] for p in pts) for i in range(3)]
    mx = [max(p[i] for p in pts) for i in range(3)]
    print(f"  position min={[round(v,3) for v in mn]} max={[round(v,3) for v in mx]} "
          f"(frame offset ~{[round(v) for v in pts[0]]})")
    print(f"  path length={path_len:.3f} m")
    print(f"  orientation: {'POSITION-ONLY (identity quaternion written; use --pose_relation trans_part)' if orient_synthetic else 'present in input, kept'}")
    if bag_start is not None and bag_end is not None:
        ov = max(0.0, min(ts[-1], bag_end) - max(ts[0], bag_start))
        inside = sum(1 for t in ts if bag_start <= t <= bag_end)
        print(f"  vs bag [{bag_start:.2f}, {bag_end:.2f}]: overlap={ov:.1f}s  "
              f"GT starts {ts[0]-bag_start:+.1f}s rel. bag start, ends {ts[-1]-bag_end:+.1f}s rel. bag end, "
              f"{inside}/{len(ts)} poses inside bag range")
    return len(clean)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", nargs="?", help="GEODE GT file (default: both Tunneling files)")
    ap.add_argument("output", nargs="?", help="output TUM file")
    ap.add_argument("--bag-start", type=float, default=None)
    ap.add_argument("--bag-end", type=float, default=None)
    args = ap.parse_args()

    if args.input and args.output:
        jobs = [(args.input, args.output, args.bag_start, args.bag_end)]
    elif args.input or args.output:
        ap.error("give both INPUT and OUTPUT, or neither for the defaults")
    else:
        jobs = DEFAULT_JOBS

    ok = True
    for in_path, out_path, b0, b1 in jobs:
        if not os.path.isfile(in_path):
            print(f"[error] missing input: {in_path}", file=sys.stderr)
            ok = False
            continue
        convert(in_path, out_path, b0, b1)
        print()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
