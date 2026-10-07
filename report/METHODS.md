# Phương pháp và giới hạn — Topic A

## 1. Chuỗi hình học

`velo_to_cam` dùng `T_cam_velo = R0_rect @ Tr_velo_to_cam` dạng đồng nhất 4×4. Với mảng mỗi hàng một điểm, nhân `points_h @ T_cam_velo.T`. `cam_to_image` nhân `P2.T`, chia cho thành phần thứ ba, lọc z_cam>0.1 m, finite, mẫu chia>1e-12 và pixel thuộc `[0,W)×[0,H)`. Mask luôn cùng số hàng với input, UV/depth giữ thứ tự input hợp lệ.

Kiểm chứng `src/projection_checks.py` gồm pinhole giải tích với điểm trên biên trái/trên/phải/dưới, điểm sau camera, depth đúng ngưỡng, NaN/Inf, denominator bằng 0, cloud rỗng, rigid transform và 3D box xoay. Synthetic `(10,0,0)` cho pixel `(613.9641,175.0065)` và depth `9.72732 m`.

## 2. Membership và mẫu số

Label KITTI ở rectified camera frame, dimensions=(h,w,l), location ở **đáy** box. Tọa độ local = `(point_cam - location) @ R_y`; điểm nằm trong box nếu x∈[-l/2,l/2], y∈[-h,0], z∈[-w/2,w/2], tolerance số học 1e-6 m, không padding.

Membership được tính **một lần bằng calibration gốc**, không thay theo drift. Giữ cả điểm object ngoài ảnh trong mẫu số. Có thể một điểm thuộc nhiều box nếu box chồng nhau: đó là nhiều cặp point-object, không ép gán độc quyền. Với object i: q_i = 100 × số điểm chiếu trong ảnh và trong bbox_i / số điểm baseline trong 3D box_i. Object 0 điểm có metric NaN; 1–19 điểm vẫn xuất CSV nhưng không vào tổng hợp chính.

Class cố định: Car, Pedestrian, Cyclist, Bicycle. KITTI Cyclist và nuScenes Bicycle không hoàn toàn đồng nghĩa; chưa chuẩn hóa ontology. Depth bin dùng **z của label trong camera frame**, không phải range Euclidean LiDAR: [0,15), [15,30), [30,∞).

Retention theo điểm = 100×Σinside/Σpoints, nên vật nhiều điểm có trọng số cao. Macro retention = trung bình q_i, mỗi object một trọng số. Bài báo cáo cả hai. Truncated object không bị loại: weighted baseline KITTI chỉ 62.59% trong khi macro 91.25% vì nhiều điểm nằm ngoài ảnh ở một số xe rất gần; ví dụ frame 000011 object 4 truncated=0.98, 3251 điểm, q=6.37%.

FOV frame = 100×số điểm trong ảnh / số điểm XYZ hữu hạn, không chia cho tất cả dòng bao gồm NaN. Pixel displacement dùng điểm baseline trong FOV và còn chiếu được sau drift; **bao gồm điểm bị đẩy ra ngoài FOV**, loại điểm không còn chiếu được và xuất số mất này riêng. p50/p95 được tính **trong từng frame**; số trong summary là trung bình các phân vị frame, không phải phân vị của pool tất cả điểm.

## 3. Sweep và tính công bằng

19 cấu hình: baseline; yaw ±0.5/±1/±2/±3°; lateral ±2/±5/±10 cm; pitch ±1°; roll ±1°. Mỗi cấu hình chỉ thay một yếu tố, giữ nguyên image, label, points và calibration gốc. Dùng toàn bộ điểm, không random; seed=42 là metadata, không tác động số đo.

Drift được ghép `Tr_velo_to_cam @ D` trong raw LiDAR frame, tức mô phỏng **lỗi calibration**, không sửa vật thể hoặc label. KITTI forward=x,left=y,up=z; nuScenes forward=y,left=−x,up=z. Lateral dương=trái; roll quanh forward; pitch quanh left; yaw quanh up. Do đó nuScenes lateral dùng tx=−value, roll dùng pitch_deg=value, pitch dùng roll_deg=−value. Calibration được deep-copy, dữ liệu gốc không đổi.

## 4. Score, ngưỡng và split

Score QA offline của frame = `q_baseline_weighted − q_drift_weighted`, đơn vị điểm phần trăm. Đây là **reference-based score có label**, không phải detector drift tự vận hành trên xe.

Định nghĩa trước fit: actionable `|yaw|≥1°`; baseline và `|yaw|=0.5°` được coi là control/tolerated. Điều này không có nghĩa 0.5° an toàn cho mọi ứng dụng. Chọn threshold không âm, tối đa balanced accuracy trên train; khi hòa chọn threshold nhỏ hơn. Decision dùng score>threshold, không dùng dịch chuyển pixel làm feature.

KITTI chia frame ID theo danh sách **có score** đã sort: vị trí chẵn train, lẻ test; 9 frame mỗi tập. nuScenes fit scene-0103 ban ngày (36 frame có score), test scene-1094 ban đêm (32 frame có score). Test không tham gia chọn threshold. Mỗi frame có 6 cấu hình positive và 3 control; các cấu hình trong cùng frame tương quan, không coi chúng là các scene độc lập.

Hai KITTI frame và 12 nuScenes frame không có object thuộc class đã chọn với ≥20 điểm: vẫn giữ FOV/pixel statistics, score NaN và không tham gia threshold. Coverage là 18/20 và 68/80 frame. Không được biến NaN thành “calibration tốt”. Tập nuScenes gần nhau về thời gian trong một scene; chia ngày→đêm là kiểm tra đổi scene/điều kiện, không đủ để kết luận tổng quát hóa rộng.

## 5. Tại sao các sensor khác nhau?

Theo data/README.md: KITTI LiDAR 64 beam, nuScenes 32 beam; ảnh nuScenes lớn hơn. Số điểm không phải nguyên nhân duy nhất của pixel sensitivity: pixel shift còn phụ thuộc intrinsics/focal length và vị trí trong ảnh. FOV toàn cloud phụ thuộc camera hướng trước, LiDAR 360°, góc mở camera và scene, không phải accuracy.

nuScenes loader bù ego motion giữa timestamp LiDAR/camera, nhưng không bù chuyển động riêng từng vật thể. Box camera được xấp xỉ bằng yaw và bbox 2D được suy ra từ 8 góc box, không phải 2D annotation độc lập. Retention vì vậy có tính phụ thuộc vào cách dựng label; không thể dùng để tuyên bố sensor này tốt hơn sensor kia. Trong class/điều kiện lọc đã chọn, nuScenes không có object ≥30 m đủ 20 điểm.

## 6. Failure và hướng cải thiện

KITTI cyclist 20 điểm ở 44.76 m: −2° yaw làm q=0 dù FOV gần như giữ nguyên. Một điểm tương ứng 5 pp, nên sample ít điểm cần ghi coverage, không suy diễn thành recall 3D detector.

nuScenes scene-1094_008: pedestrian giảm 70 pp nhưng weighted score toàn frame chỉ 1.102 pp. Một xe 72 điểm cải thiện 40.278 pp, bù 29 điểm; các xe khác mất 17+5 điểm và pedestrian mất14 điểm, tổng giảm7/635=1.102 pp. Drift có thể vô tình tăng retention ở object bị truncated/misaligned baseline; score tổng bị triệt tiêu.

Hướng tiếp theo: dùng macro/per-class score, kiểm riêng object nhỏ, hạn chế truncated object cho score nhưng vẫn báo coverage, xác minh bằng label 2D độc lập; metric không cần label có thể dựa biên ảnh/depth nhưng chưa được triển khai trong bài. Không đo detector recall/AP hoặc latency, không khẳng định hiệu năng thời gian thực.
