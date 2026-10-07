"""Sweep calibration CPU, deterministic, cả KITTI và nuScenes.

python -m src.run_calibration_qa --help
Không ghi vào data/; chỉ CSV/JSON trong output. Không dùng random sampling.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

from starter.datasets import dataset_type, list_frames, load_frame
from starter.projection import perturb_extrinsic
from src.calibration_metrics import CLASSES, MIN_OBJECT_POINTS, measure_frame, prepare_frame


def configurations():
    return ([dict(axis="baseline", value=0., unit="none")]
            + [dict(axis="yaw", value=v, unit="deg") for v in (-3, -2, -1, -.5, .5, 1, 2, 3)]
            + [dict(axis="lateral", value=v, unit="m") for v in (-.10, -.05, -.02, .02, .05, .10)]
            + [dict(axis=axis, value=v, unit="deg") for axis in ("pitch", "roll") for v in (-1, 1)])


def make_calibration(calib, kind, axis, value):
    """Physical axes: roll about forward, pitch about left, yaw about up.

    KITTI: forward=x, left=y. nuScenes: forward=y, left=-x.
    perturb_extrinsic composes drift in the raw LiDAR frame (Tr @ D).
    """
    kwargs = {}
    if axis == "yaw":
        kwargs["yaw_deg"] = value
    elif axis == "roll":
        kwargs["roll_deg" if kind == "kitti" else "pitch_deg"] = value
    elif axis == "pitch":
        kwargs["pitch_deg" if kind == "kitti" else "roll_deg"] = value if kind == "kitti" else -value
    elif axis == "lateral":
        kwargs["t_xyz_m"] = (0, value, 0) if kind == "kitti" else (-value, 0, 0)
    elif axis != "baseline":
        raise ValueError(f"Unknown axis {axis}")
    return perturb_extrinsic(calib, **kwargs)


def summarize(frames, objects):
    records = []
    for (dataset, axis, value, unit), group in frames.groupby(["dataset", "axis", "value", "unit"], sort=True):
        subset = objects[(objects.dataset == dataset) & (objects.axis == axis)
                         & (objects.value == value) & objects.eligible]
        n = int(subset.n_object_points.sum())
        records.append(dict(dataset=dataset, axis=axis, value=value, unit=unit,
                            n_frames=len(group), n_points_mean=group.n_finite.mean(),
                            n_eligible_objects=len(subset), n_object_points=n,
                            fov_pct_mean=group.fov_pct.mean(),
                            retention_pct=100 * subset.n_inside_box.sum() / n if n else np.nan,
                            alignment_drop_pp=100 * (subset.baseline_inside_box.sum()
                                                       - subset.n_inside_box.sum()) / n if n else np.nan,
                            object_macro_retention_pct=subset.retention_pct.mean(),
                            object_macro_drop_pp=subset.retention_drop_pp.mean(),
                            frame_pixel_p50_mean=group.pixel_shift_p50.mean(),
                            frame_pixel_p95_mean=group.pixel_shift_p95.mean()))
    return pd.DataFrame(records)


def calibrate_detector(frames):
    """Offline labeled QA score; test frames never participate in threshold search.

    Tolerated controls: baseline and |yaw|=.5; actionable: |yaw|>=1.
    Score = baseline retention minus perturbed retention (percentage points).
    Select training balanced accuracy; prefer smaller threshold on ties.
    """
    detector, predictions = [], []
    for dataset, group in frames.groupby("dataset", sort=True):
        yaw = group[group.axis.isin(["baseline", "yaw"])].copy()
        yaw = yaw[np.isfinite(yaw.alignment_drop_pp)].copy()
        if yaw.empty:
            continue
        ids = sorted(yaw.frame_id.unique())
        train_ids = ids[::2] if dataset != "nuscenes" else [i for i in ids if i.startswith("scene-0103_")]
        yaw["split"] = np.where(yaw.frame_id.isin(train_ids), "train", "test")
        yaw["actionable_drift"] = yaw.value.abs() >= 1
        train = yaw[yaw.split == "train"]
        if train.empty:
            print(f"No training frames for {dataset}: threshold not fitted", flush=True)
            continue
        values = np.sort(train.alignment_drop_pp.unique())
        thresholds = np.unique(np.r_[0., values - 1e-9, values + 1e-9])
        best = None
        for threshold in thresholds:
            if threshold < 0:
                continue
            pred = train.alignment_drop_pp > threshold
            truth = train.actionable_drift
            tpr = float(pred[truth].mean())
            tnr = float((~pred[~truth]).mean())
            balanced = (tpr + tnr) / 2
            if best is None or balanced > best[0] + 1e-12:
                best = (balanced, float(threshold))
        threshold = best[1]
        yaw["threshold_pp"] = threshold
        yaw["detected"] = yaw.alignment_drop_pp > threshold
        for split, subset in yaw.groupby("split", sort=True):
            truth, pred = subset.actionable_drift, subset.detected
            tp, fn = int((truth & pred).sum()), int((truth & ~pred).sum())
            fp, tn = int((~truth & pred).sum()), int((~truth & ~pred).sum())
            detector.append(dict(dataset=dataset, split=split, threshold_pp=threshold,
                                 tp=tp, fn=fn, fp=fp, tn=tn,
                                 tpr=tp / (tp + fn), fpr=fp / (fp + tn),
                                 balanced_accuracy=.5 * (tp / (tp + fn) + tn / (tn + fp))))
        predictions.append(yaw[["dataset", "frame_id", "axis", "value", "split", "alignment_drop_pp",
                                "threshold_pp", "actionable_drift", "detected", "fov_pct", "pixel_shift_p50"]])
    columns = ["dataset", "frame_id", "axis", "value", "split", "alignment_drop_pp",
               "threshold_pp", "actionable_drift", "detected", "fov_pct", "pixel_shift_p50"]
    return pd.DataFrame(detector, columns=["dataset", "split", "threshold_pp", "tp", "fn", "fp", "tn",
                                           "tpr", "fpr", "balanced_accuracy"]), (
        pd.concat(predictions, ignore_index=True) if predictions else pd.DataFrame(columns=columns))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--data-roots", nargs="+", default=["data/kitti_mini", "data/nuscenes_mini_subset"])
    ap.add_argument("--out-dir", default="results")
    ap.add_argument("--frames", nargs="+", help="Optional explicit frame IDs; use one data-root")
    args = ap.parse_args()
    if args.frames and len(args.data_roots) != 1:
        ap.error("--frames requires exactly one data-root")
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    configs = configurations()
    rows, obj_rows, manifest = [], [], []
    for root in args.data_roots:
        kind = dataset_type(root)
        dataset = "synthetic" if Path(root).name == "synthetic" else kind
        frame_ids = args.frames or list_frames(root)
        manifest.append(dict(data_root=root, dataset=dataset, frames=frame_ids))
        for index, frame_id in enumerate(frame_ids):
            frame = load_frame(root, frame_id)
            prepared = prepare_frame(frame)
            for config in configs:
                metadata = dict(dataset=dataset, frame_id=frame_id, **config)
                calib = make_calibration(frame["calib"], kind, config["axis"], config["value"])
                metrics, objects, _, _ = measure_frame(frame, prepared, calib)
                metrics["timestamp_delta_ms"] = (frame.get("timestamp_camera_us", 0)
                                                  - frame.get("timestamp_lidar_us", 0)) / 1000 if kind == "nuscenes" else np.nan
                rows.append(dict(**metadata, **metrics))
                obj_rows.extend(dict(**metadata, **obj) for obj in objects)
            print(f"{dataset}: {index + 1}/{len(frame_ids)} {frame_id}", flush=True)
    frames, objects = pd.DataFrame(rows), pd.DataFrame(obj_rows)
    summary = summarize(frames, objects)
    detector, predictions = calibrate_detector(frames)
    for name, df in (("frames", frames), ("objects", objects), ("summary", summary),
                     ("detector", detector), ("predictions", predictions)):
        df.to_csv(out / f"calibration_sweep_{name}.csv", index=False, float_format="%.10g")
    config = dict(seed=42, randomness="none: all points and frames", datasets=manifest,
                  configurations=configs, classes=list(CLASSES), min_object_points=MIN_OBJECT_POINTS,
                  min_depth_m=.1, nuScenes_ego_motion=True,
                  object_membership="frozen baseline camera-frame 3D boxes, no padding",
                  depth_bins="camera z of label bottom center: [0,15), [15,30), [30,infinity)",
                  threshold_split="KITTI sorted IDs even-index train / odd-index test; nuScenes day train / night test",
                  positive_definition="abs(yaw)>=1 degree; controls baseline and +/-0.5 degree",
                  nuScenes_label_caveat="2D labels derived from 3D boxes; yaw-only camera boxes approximate full rotation",
                  sensor_axes="KITTI forward=x,left=y; nuScenes forward=y,left=-x; yaw=up(z)")
    (out / "run_config.json").write_text(json.dumps(config, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(summary.to_string(index=False))
    print(detector.to_string(index=False))


if __name__ == "__main__":
    main()
