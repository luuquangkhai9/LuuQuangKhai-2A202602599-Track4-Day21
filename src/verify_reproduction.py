"""Compare ALL deterministic outputs with a fresh run and check scientific invariants.

python -m src.verify_reproduction --reference results --candidate results/reproduced
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pandas as pd


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--reference", default="results")
    ap.add_argument("--candidate", default="results/reproduced")
    ap.add_argument("--write-summary", help="Optional verification JSON path")
    args = ap.parse_args()
    reference, candidate = Path(args.reference), Path(args.candidate)
    names = [f"calibration_sweep_{suffix}.csv" for suffix in
             ("frames", "objects", "summary", "detector", "predictions")]
    verification = {}
    for name in names:
        original = pd.read_csv(reference / name)
        regenerated = pd.read_csv(candidate / name)
        pd.testing.assert_frame_equal(original, regenerated, check_exact=True)
        # LF/CRLF is a Git checkout difference, not a changed number.
        a = (reference / name).read_text(encoding="utf-8").replace("\r\n", "\n")
        b = (candidate / name).read_text(encoding="utf-8").replace("\r\n", "\n")
        assert a == b, f"CSV content mismatch: {name}"
        verification[name] = dict(rows=len(original), normalized_sha256=hashlib.sha256(a.encode()).hexdigest())
        print(f"[PASS] {name}: {len(original)} rows, exact values/content")
    manifest = json.loads((reference / "run_config.json").read_text(encoding="utf-8"))
    rerun_manifest = json.loads((candidate / "run_config.json").read_text(encoding="utf-8"))
    assert manifest == rerun_manifest, "Frame/configuration manifest mismatch"

    frames = pd.read_csv(candidate / names[0])
    objects = pd.read_csv(candidate / names[1])
    expected = len(manifest["configurations"])
    keys = ["dataset", "frame_id", "axis", "value"]
    assert not frames.duplicated(keys).any()
    for dataset in manifest["datasets"]:
        subset = frames[frames.dataset == dataset["dataset"]]
        assert set(subset.frame_id) == set(dataset["frames"])
        assert subset.groupby("frame_id").size().eq(expected).all()
    for field in ("n_points", "n_finite", "n_eligible_objects", "n_object_points", "baseline_inside_box"):
        assert frames.groupby(["dataset", "frame_id"])[field].nunique().eq(1).all(), field
    for field in ("n_object_points", "baseline_inside_box", "eligible", "depth_m"):
        assert objects.groupby(["dataset", "frame_id", "object_id"])[field].nunique().eq(1).all(), field
    assert (frames.n_fov <= frames.n_finite).all()
    assert (frames.n_inside_box <= frames.n_object_points).all()
    assert (objects.n_inside_box <= objects.n_object_points).all()
    assert objects.eligible.eq(objects.n_object_points >= manifest["min_object_points"]).all()
    baseline = frames[frames.axis == "baseline"]
    np.testing.assert_allclose(baseline.pixel_shift_p50, 0, atol=1e-12)
    np.testing.assert_allclose(baseline.pixel_shift_p95, 0, atol=1e-12)
    np.testing.assert_allclose(baseline.alignment_drop_pp.dropna(), 0, atol=1e-12)
    assert frames.loc[frames.n_eligible_objects == 0, "alignment_drop_pp"].isna().all()
    predictions = pd.read_csv(candidate / names[4])
    for _, group in predictions.groupby("dataset"):
        train = set(group.loc[group.split == "train", "frame_id"])
        test = set(group.loc[group.split == "test", "frame_id"])
        assert not (train & test), "Train/test frame leakage"
        assert group.threshold_pp.nunique() == 1
        assert group.detected.eq(group.alignment_drop_pp > group.threshold_pp).all()
    print("[PASS] Full frame/config coverage, frozen denominator, missing-score handling and train/test separation")
    if args.write_summary:
        path = Path(args.write_summary)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(dict(all_outputs_equal=True, configurations_per_frame=expected,
                                        total_frame_configurations=len(frames), outputs=verification,
                                        scientific_invariants="PASS"), indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
