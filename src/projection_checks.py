"""Kiểm chứng hình học độc lập. Chạy: python -m src.projection_checks."""
import numpy as np

from starter.kitti_io import KittiCalib, load_calib
from starter.projection import cam_to_image, velo_to_cam


def main():
    calibration = load_calib("data/synthetic/training/calib/000000.txt")
    point = velo_to_cam(np.array([[10., 0., 0.]]), calibration)
    uv, depth, mask = cam_to_image(point, calibration.P2, (375, 1242))
    np.testing.assert_allclose(depth, [9.73], atol=0.02)
    np.testing.assert_allclose(uv, [[614, 175]], atol=2)
    assert mask.tolist() == [True]
    print(f"[PASS] Synthetic reference: uv={uv[0]}, depth={depth[0]:.5f}")

    P = np.array([[100., 0, 50, 0], [0, 100, 40, 0], [0, 0, 1, 0]])
    points = np.array([[0, 0, 2], [1, 0, 2], [-1, 0, 2], [0, .8, 2],
                       [0, -.8, 2], [0, 0, -2], [0, 0, .1], [np.nan, 0, 2],
                       [0, np.inf, 2], [0, 0, 0]])
    actual_uv, actual_depth, mask = cam_to_image(points, P, (80, 100, 3))
    assert mask.tolist() == [True, False, True, False, True, False, False, False, False, False]
    np.testing.assert_allclose(actual_uv, [[50, 40], [0, 40], [50, 0]])
    np.testing.assert_allclose(actual_depth, [2, 2, 2])
    print("[PASS] Analytic pinhole, half-open image bounds, depth, NaN/Inf and original mask")

    P_zero = P.copy()
    P_zero[2] = 0
    assert not cam_to_image(points, P_zero, (80, 100))[2].any()
    empty = cam_to_image(np.empty((0, 3)), P, (80, 100))
    assert [v.shape for v in empty] == [(0, 2), (0,), (0,)]
    print("[PASS] Zero homogeneous denominator and empty cloud")

    # Rigid transform kiểm bằng công thức tay, không dùng phép chiếu để tạo expected.
    R = np.array([[0., -1, 0], [1, 0, 0], [0, 0, 1]])
    test_calib = KittiCalib(P, np.eye(3), np.column_stack((R, [1, 2, 3])))
    np.testing.assert_allclose(velo_to_cam(np.array([[2, 4, 6.]]), test_calib), [[-3, 4, 9]])
    print("[PASS] Rigid transform and transpose convention")

    from src.calibration_metrics import points_in_box
    from starter.kitti_io import KittiObject
    obj = KittiObject("Car", 0, 0, 0, np.array([0, 0, 99, 79]),
                      np.array([2., 2, 4]), np.array([0., 0, 10]), np.pi / 2)
    membership = points_in_box(np.array([[0., -1, 10], [0, 1, 10], [1.5, -1, 10],
                                          [0, -1, 11.5], [0, -1, 12.5]]), obj)
    assert membership.tolist() == [True, False, False, True, False]
    print("[PASS] Rotated 3D box, bottom center and h/w/l convention")

    from src.calibration_metrics import measure_frame, prepare_frame
    from src.run_calibration_qa import make_calibration
    identity = KittiCalib(P, np.eye(3), np.column_stack((np.eye(3), [0, 0, 0])))
    obj.rotation_y = 0
    obj.bbox = np.array([45., 25, 60, 40])
    frame = dict(points=np.array([[0., -1, 10, 1], [.5, -1, 10, 1], [100, 0, 10, 1]]),
                 calib=identity, labels=[obj], image=np.zeros((80, 100, 3), np.uint8))
    prepared = prepare_frame(frame)
    drift = KittiCalib(P, np.eye(3), np.column_stack((np.eye(3), [20, 0, 0])))
    _, measured, _, _ = measure_frame(frame, prepared, drift)
    assert measured[0]["n_object_points"] == 2
    assert measured[0]["baseline_retention_pct"] == 100
    assert measured[0]["retention_pct"] == 0
    assert measured[0]["retention_drop_pp"] == 100
    np.testing.assert_allclose(make_calibration(identity, "kitti", "lateral", .05).Tr_velo_to_cam[:, 3], [0, .05, 0])
    np.testing.assert_allclose(make_calibration(identity, "nuscenes", "lateral", .05).Tr_velo_to_cam[:, 3], [-.05, 0, 0])
    np.testing.assert_allclose(identity.Tr_velo_to_cam[:, 3], [0, 0, 0])
    print("[PASS] Frozen denominator includes off-image points; physical sensor axes; original calibration unchanged")


if __name__ == "__main__":
    main()
