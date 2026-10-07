"""Vẽ evidence từ CSV + dữ liệu gốc; CLI --stage baseline/all và --results-dir."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from starter.datasets import list_frames, load_frame
from starter.projection import draw_box2d, overlay_points
from src.calibration_metrics import in_bbox, measure_frame, prepare_frame
from src.run_calibration_qa import make_calibration


def save_image(path, image):
    if not cv2.imwrite(str(path), image):
        raise IOError(f"Cannot save {path}")


def banner(image, title, subtitle=""):
    out = cv2.copyMakeBorder(image, 64, 34, 0, 0, cv2.BORDER_CONSTANT, value=(245, 245, 245))
    cv2.putText(out, title, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, .65, (20, 20, 20), 2, cv2.LINE_AA)
    cv2.putText(out, subtitle, (12, 51), cv2.FONT_HERSHEY_SIMPLEX, .48, (35, 35, 35), 1, cv2.LINE_AA)
    cv2.putText(out, "Sources: KITTI Vision Benchmark Suite / nuScenes (Motional)",
                (12, out.shape[0] - 11), cv2.FONT_HERSHEY_SIMPLEX, .45, (35, 35, 35), 1, cv2.LINE_AA)
    return out


def baseline_figures(figures, out):
    choices = {}
    for frame_id in list_frames("data/kitti_mini"):
        frame = load_frame("data/kitti_mini", frame_id)
        prepared = prepare_frame(frame)
        for entry in prepared["objects"]:
            obj, indices = entry["obj"], entry["indices"]
            n = len(indices)
            if n < 20 or entry["baseline_inside"] / n < .7:
                continue
            category = "near" if obj.location[2] < 15 else "mid" if obj.location[2] < 30 else "far"
            # Chọn theo dữ liệu baseline, không dùng kết quả drift.
            priority = (obj.occluded == 0, obj.truncated < .2, n)
            if category not in choices or priority > choices[category][0]:
                choices[category] = (priority, frame_id, entry["index"])
    if set(choices) != {"near", "mid", "far"}:
        raise RuntimeError(f"Missing baseline distance groups: {choices}")
    selection = []
    for category in ("near", "mid", "far"):
        _, frame_id, obj_index = choices[category]
        frame = load_frame("data/kitti_mini", frame_id)
        prepared = prepare_frame(frame)
        obj = frame["labels"][obj_index]
        vis = overlay_points(frame["image"], prepared["uv"][prepared["fov"]],
                             prepared["cam"][prepared["fov"], 2], radius=1)
        for label in frame["labels"]:
            vis = draw_box2d(vis, label.bbox, color=(100, 180, 100))
        vis = draw_box2d(vis, obj.bbox, color=(0, 255, 255), label=f"{obj.type} z={obj.location[2]:.1f}m")
        raw = draw_box2d(frame["image"], obj.bbox, color=(0, 255, 255), label="Selected GT object")
        image = np.hstack((raw, vis))
        save_image(figures / f"overlay_{category}.png", banner(image,
                   f"KITTI {frame_id} - {category} - camera depth {obj.location[2]:.2f} m",
                   "Left: input image. Right: baseline projection. Yellow: selected box. Depth: near red / far blue (50 m clip)."))
        selection.append(dict(group=category, dataset="kitti", frame_id=frame_id,
                              object_id=obj_index, depth_m=float(obj.location[2])))
    (out / "demo_selection.json").write_text(json.dumps(selection, indent=2) + "\n", encoding="utf-8")
    return selection


def sweep_plot(summary, axis, figures):
    datasets = sorted(summary.dataset.unique())
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for dataset in datasets:
        part = summary[(summary.dataset == dataset) & summary.axis.isin(["baseline", axis])].sort_values("value")
        x = part.value * (100 if axis == "lateral" else 1)
        for ax, metric, title in zip(axes.flat,
                                    ("fov_pct_mean", "retention_pct", "frame_pixel_p50_mean", "object_macro_drop_pp"),
                                    ("Mean frame points in FOV (%)", "Object point retention (%)",
                                     "Mean of frame pixel shift p50 (px)", "Mean object retention drop (pp)")):
            ax.plot(x, part[metric], marker="o", label=dataset)
            ax.set_title(title)
            ax.set_xlabel("Leftward translation (cm)" if axis == "lateral" else f"{axis.title()} drift (deg)")
            ax.grid(alpha=.3)
            ax.legend()
    fig.suptitle("Frozen baseline object membership; >=20 points/object. KITTI and nuScenes labels differ.", fontsize=11)
    fig.savefig(figures / ("translation_vs_metrics.png" if axis == "lateral" else f"{axis}_vs_metrics.png"), dpi=160)
    plt.close(fig)


def failure_figure(row, objects, roots, figures, filename, description):
    dataset, frame_id, axis, value = row.dataset, row.frame_id, row.axis, float(row.value)
    frame = load_frame(roots[dataset], frame_id)
    prepared = prepare_frame(frame)
    calibration = make_calibration(frame["calib"], dataset, axis, value)
    metrics, _, uv, fov = measure_frame(frame, prepared, calibration)
    subset = objects[(objects.dataset == dataset) & (objects.frame_id == frame_id)
                     & (objects.axis == axis) & (objects.value == value) & objects.eligible]
    if subset.empty:
        raise RuntimeError(f"No eligible object for failure {frame_id}")
    focus = subset.sort_values(["retention_drop_pp", "n_object_points"], ascending=False).iloc[0]
    obj = frame["labels"][int(focus.object_id)]
    entry = next(o for o in prepared["objects"] if o["index"] == int(focus.object_id))
    indices = entry["indices"]
    panels, crops = [], []
    h, w = frame["image"].shape[:2]
    x1, y1, x2, y2 = obj.bbox
    margin = max(40, int(max(x2 - x1, y2 - y1) * .5))
    xa, ya, xb, yb = max(0, int(x1) - margin), max(0, int(y1) - margin), min(w, int(x2) + margin), min(h, int(y2) + margin)
    for full_uv, full_fov, label in ((prepared["uv"], prepared["fov"], "Baseline"), (uv, fov, f"{axis}={value:g}")):
        view = frame["image"].copy()
        match = in_bbox(full_uv, full_fov, obj.bbox)
        for index in indices[full_fov[indices]]:
            color = (40, 240, 40) if match[index] else (30, 30, 240)
            cv2.circle(view, tuple(full_uv[index].astype(int)), 2, color, -1)
        view = draw_box2d(view, obj.bbox, color=(0, 255, 255))
        retained = int(match[indices].sum())
        panels.append(cv2.resize(view, (900, round(h * 900 / w))))
        crop = cv2.resize(view[ya:yb, xa:xb], (900, 360))
        crops.append(banner(crop, f"{label}: {retained}/{len(indices)} = {100 * retained / len(indices):.1f}%",
                            f"{obj.type}, camera z={obj.location[2]:.1f}m; green=inside box, red=outside; yellow=GT"))
    assembled = np.vstack((np.hstack(panels), np.hstack(crops)))
    save_image(figures / filename, banner(assembled,
               f"{dataset} {frame_id}: {description}",
               f"All-frame FOV={metrics['fov_pct']:.3f}%; median pixel shift={metrics['pixel_shift_p50']:.2f}px; object points fixed."))
    return dict(filename=filename, dataset=dataset, frame_id=frame_id, axis=axis, value=value,
                object_id=int(focus.object_id), object_depth_m=float(focus.depth_m),
                object_drop_pp=float(focus.retention_drop_pp), object_n_points=int(focus.n_object_points),
                baseline_retention_pct=float(focus.baseline_retention_pct), retention_pct=float(focus.retention_pct),
                frame_fov_pct=float(metrics["fov_pct"]), frame_alignment_drop_pp=float(metrics["alignment_drop_pp"]),
                frame_pixel_p50=float(metrics["pixel_shift_p50"]))


def all_figures(out, figures):
    summary = pd.read_csv(out / "calibration_sweep_summary.csv")
    objects = pd.read_csv(out / "calibration_sweep_objects.csv")
    frames = pd.read_csv(out / "calibration_sweep_frames.csv")
    predictions = pd.read_csv(out / "calibration_sweep_predictions.csv")
    config = json.loads((out / "run_config.json").read_text(encoding="utf-8"))
    roots = {d["dataset"]: d["data_root"] for d in config["datasets"]}
    for axis in ("yaw", "lateral", "pitch", "roll"):
        sweep_plot(summary, axis, figures)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), constrained_layout=True)
    for dataset in sorted(objects.dataset.unique()):
        subset = objects[(objects.dataset == dataset) & objects.axis.isin(["baseline", "yaw"]) & objects.eligible]
        for depth_bin, part in subset.groupby("depth_bin", sort=True):
            means = part.groupby("value").retention_drop_pp.mean()
            ax = axes[0 if dataset == "kitti" else 1]
            ax.plot(means.index, means.values, marker="o", label=depth_bin)
            ax.set(title=f"{dataset}: macro object drop", xlabel="Yaw (deg)", ylabel="Retention drop (pp)")
            ax.grid(alpha=.3)
            ax.legend()
    fig.savefig(figures / "yaw_by_depth.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), constrained_layout=True)
    for ax, (dataset, part) in zip(axes, predictions.groupby("dataset", sort=True)):
        for split, subset in part.groupby("split", sort=True):
            means = subset.groupby("value").alignment_drop_pp.mean()
            ax.plot(means.index, means.values, marker="o", label=f"{split} mean")
        ax.axhline(part.threshold_pp.iloc[0], color="red", linestyle="--", label="train threshold")
        ax.set(title=f"{dataset}: offline GT-based score", xlabel="Yaw (deg)", ylabel="Retention drop (pp)")
        ax.grid(alpha=.3)
        ax.legend()
    fig.suptitle("Controls: 0 and +/-0.5 deg; actionable: |yaw|>=1. Test frames excluded from threshold fitting.", fontsize=10)
    fig.savefig(figures / "alignment_score_threshold.png", dpi=160)
    plt.close(fig)

    failures = []
    geometry = frames[(frames.dataset == "kitti") & (frames.axis == "yaw") & (frames.value.abs() == 3)]
    row = geometry.sort_values("alignment_drop_pp", ascending=False).iloc[0]
    failures.append(failure_figure(row, objects, roots, figures, "fail_01_yaw_drift.png", "Geometry: projected object points leave GT box"))

    baseline = frames[frames.axis == "baseline"][["dataset", "frame_id", "fov_pct"]].rename(columns={"fov_pct": "baseline_fov_pct"})
    joined = frames.merge(baseline, on=["dataset", "frame_id"])
    joined["fov_change_pp"] = (joined.fov_pct - joined.baseline_fov_pct).abs()
    blind = joined[(joined.axis == "yaw") & (joined.value.abs() >= 1) & (joined.fov_change_pp < .5)
                   & (joined.alignment_drop_pp > 5)]
    if not blind.empty:
        row = blind.sort_values("alignment_drop_pp", ascending=False).iloc[0]
        item = failure_figure(row, objects, roots, figures, "fail_02_fov_metric_blind_spot.png", "Metric: small FOV change hides large object mismatch")
        item["fov_change_pp"] = float(row.fov_change_pp)
        failures.append(item)
    misses = predictions[(predictions.split == "test") & predictions.actionable_drift & ~predictions.detected]
    if not misses.empty:
        row = misses.sort_values("pixel_shift_p50", ascending=False).iloc[0]
        item = failure_figure(row, objects, roots, figures, "fail_03_alignment_score_miss.png", "Metric: actionable yaw missed by offline score")
        item["threshold_pp"] = float(row.threshold_pp)
        item["split"] = "test"
        failures.append(item)
    (out / "failure_selection.json").write_text(json.dumps(failures, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(failures, indent=2))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--results-dir", default="results")
    ap.add_argument("--stage", choices=["baseline", "all"], default="all")
    args = ap.parse_args()
    out = Path(args.results_dir)
    figures = out / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    selection = baseline_figures(figures, out)
    print(json.dumps(selection, indent=2))
    if args.stage == "all":
        all_figures(out, figures)


if __name__ == "__main__":
    main()
