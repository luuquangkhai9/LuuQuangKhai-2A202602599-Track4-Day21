"""Metric QA offline; mọi membership được đóng băng bằng calibration gốc.

Box KITTI: location ở đáy, dimensions=(h,w,l), rotation_y quanh camera y.
nuScenes loader tạo box KITTI xấp xỉ chỉ có yaw; bbox 2D cũng được sinh từ
3D box, không phải nhãn 2D độc lập. Vì vậy không coi retention là accuracy GT.
"""
from __future__ import annotations

import numpy as np

from starter.projection import cam_to_image, velo_to_cam

CLASSES = ("Car", "Pedestrian", "Cyclist", "Bicycle")
MIN_OBJECT_POINTS = 20


def points_in_box(points_cam, obj):
    h, w, length = obj.dimensions
    c, s = np.cos(obj.rotation_y), np.sin(obj.rotation_y)
    R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    local = (points_cam - obj.location) @ R
    eps = 1e-6
    return (np.isfinite(local).all(axis=1)
            & (np.abs(local[:, 0]) <= length / 2 + eps)
            & (local[:, 1] >= -h - eps) & (local[:, 1] <= eps)
            & (np.abs(local[:, 2]) <= w / 2 + eps))


def project_full(points_cam, P2, image_shape):
    """Full UV retains row indices; front-camera points may be outside image.

    Pixel displacement uses baseline-in-FOV points still projectable after drift,
    including points pushed outside the image. Back-camera loss is counted separately.
    """
    _, _, in_fov = cam_to_image(points_cam, P2, image_shape)
    uv = np.full((len(points_cam), 2), np.nan)
    valid = np.isfinite(points_cam).all(axis=1) & (points_cam[:, 2] > .1)
    indices = np.flatnonzero(valid)
    projected = np.column_stack((points_cam[indices], np.ones(len(indices)))) @ P2.T
    safe = np.isfinite(projected).all(axis=1) & (projected[:, 2] > 1e-12)
    indices, projected = indices[safe], projected[safe]
    uv[indices] = projected[:, :2] / projected[:, 2:3]
    projectable = np.isfinite(uv).all(axis=1)
    return uv, in_fov, projectable


def in_bbox(uv, fov, bbox):
    x1, y1, x2, y2 = bbox
    return (fov & (uv[:, 0] >= x1) & (uv[:, 0] <= x2)
            & (uv[:, 1] >= y1) & (uv[:, 1] <= y2))


def prepare_frame(frame):
    points = frame["points"]
    finite = np.isfinite(points[:, :3]).all(axis=1)
    cam = velo_to_cam(points[:, :3], frame["calib"])
    uv, fov, projectable = project_full(cam, frame["calib"].P2, frame["image"].shape)
    objects = []
    for index, obj in enumerate(frame["labels"]):
        if (obj.type not in CLASSES or not np.isfinite(obj.dimensions).all()
                or np.min(obj.dimensions) <= 0 or not np.isfinite(obj.location).all()
                or obj.location[2] <= .1):
            continue
        indices = np.flatnonzero(finite & points_in_box(cam, obj))
        correct = int(in_bbox(uv[indices], fov[indices], obj.bbox).sum())
        objects.append(dict(index=index, obj=obj, indices=indices, baseline_inside=correct))
    return dict(finite=finite, cam=cam, uv=uv, fov=fov, projectable=projectable, objects=objects)


def measure_frame(frame, prepared, calib):
    cam = velo_to_cam(frame["points"][:, :3], calib)
    uv, fov, projectable = project_full(cam, calib.P2, frame["image"].shape)
    paired = prepared["fov"] & projectable
    shifts = np.linalg.norm(uv[paired] - prepared["uv"][paired], axis=1)
    rows = []
    for entry in prepared["objects"]:
        obj, indices = entry["obj"], entry["indices"]
        n = len(indices)
        correct = int(in_bbox(uv[indices], fov[indices], obj.bbox).sum())
        baseline = entry["baseline_inside"]
        rows.append(dict(object_id=entry["index"], class_name=obj.type,
                         depth_m=float(obj.location[2]),
                         depth_bin="near_0_15" if obj.location[2] < 15 else
                         "mid_15_30" if obj.location[2] < 30 else "far_30_plus",
                         occluded=obj.occluded, truncated=obj.truncated,
                         n_object_points=n, eligible=n >= MIN_OBJECT_POINTS,
                         n_inside_box=correct, baseline_inside_box=baseline,
                         retention_pct=100 * correct / n if n else np.nan,
                         baseline_retention_pct=100 * baseline / n if n else np.nan,
                         retention_drop_pp=100 * (baseline - correct) / n if n else np.nan))
    eligible = [r for r in rows if r["eligible"]]
    n_obj_points = sum(r["n_object_points"] for r in eligible)
    n_inside = sum(r["n_inside_box"] for r in eligible)
    n_baseline_inside = sum(r["baseline_inside_box"] for r in eligible)
    n_finite = int(prepared["finite"].sum())
    metrics = dict(n_points=len(fov), n_finite=n_finite, n_fov=int(fov.sum()),
                   fov_pct=100 * fov.sum() / n_finite if n_finite else np.nan,
                   n_pixel_pairs=int(paired.sum()),
                   n_baseline_fov_lost=int((prepared["fov"] & ~fov).sum()),
                   n_baseline_fov_unprojectable=int((prepared["fov"] & ~projectable).sum()),
                   pixel_shift_p50=float(np.median(shifts)) if len(shifts) else np.nan,
                   pixel_shift_p95=float(np.percentile(shifts, 95)) if len(shifts) else np.nan,
                   n_eligible_objects=len(eligible), n_object_points=n_obj_points,
                   n_inside_box=n_inside, baseline_inside_box=n_baseline_inside,
                   retention_pct=100 * n_inside / n_obj_points if n_obj_points else np.nan,
                   alignment_drop_pp=100 * (n_baseline_inside - n_inside) / n_obj_points
                   if n_obj_points else np.nan)
    return metrics, rows, uv, fov
