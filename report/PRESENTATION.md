# Nội dung trình bày 3 phút — Lưu Quang Khải, H210, 2A202602599


## 0:00–0:30 — Câu hỏi và claim

“Em chọn topic A, kiểm tra calibration LiDAR–camera. Câu hỏi là: nếu giá đỡ cảm biến lệch một độ, ảnh hưởng có thể đo được không? Trên KITTI, yaw +1° làm retention trung bình theo object giảm gần 24 điểm phần trăm, nhưng tỷ lệ điểm trong toàn ảnh gần như không đổi.”

## 0:30–1:00 — Demo và cách kiểm chứng

Mở `overlay_near.png`, `overlay_mid.png`, `overlay_far.png`: object 9 m, 16.5 m, 38.26 m. Nói rõ hai hệ trục, chuỗi `P2·R0_rect·Tr_velo_to_cam`, chia tọa độ đồng nhất, bỏ NaN và điểm sau camera. Điểm chuẩn `(10,0,0)` chiếu tới `(613.96,175.01)`, depth≈9.73 m.

## 1:00–1:40 — Thí nghiệm và số liệu

Mở `yaw_vs_metrics.png`: 20 KITTI và 80 nuScenes, 19 cấu hình mỗi frame. Membership object và mẫu số đóng băng trước drift. “Em phân biệt retention theo điểm với trung bình theo object vì xe gần nhiều điểm có thể lấn át người đi bộ. Trên KITTI, p50 dịch chuyển trung bình theo frame khoảng 14.59 px khi yaw +1°; trên nuScenes khoảng 25.27 px. Khác biệt còn phụ thuộc camera intrinsics, không chỉ số beam.”

## 1:40–2:20 — Failure

Mở `fail_02_fov_metric_blind_spot.png`: cyclist 44.76 m, yaw −2°, 20/20 điểm trong box thành 0/20; FOV chỉ thay 0.00498 pp. Lỗi Geometry gây projection lệch; lỗi Metric khiến FOV không cảnh báo.
Mở `fail_03_alignment_score_miss.png`: pedestrian mất 70 pp nhưng score frame vẫn dưới ngưỡng, do xe nhiều điểm và một object cải thiện giả bù mất mát. Nêu đây là failure trên tập test, không chọn lại ngưỡng để che lỗi.

## 2:20–3:00 — Triển khai và giới hạn

“Metric hiện tại cần label và calibration baseline nên chỉ dùng offline QA. TPR/FPR còn chưa phù hợp triển khai trực tiếp. Em đề xuất log coverage, score theo class, timestamp và invalid ratio; kiểm chứng bằng label độc lập và thêm scene. Khi không đủ object, phải báo thiếu bằng chứng. Em chưa đo latency nên chưa kết luận thời gian thực.”

## Câu hỏi tự ôn

- **Vì sao không đổi membership sau perturb?** Để giữ cùng điểm object/mẫu số; đổi membership sẽ lẫn thay đổi calibration với thay đổi tập đánh giá.
- **Một độ có luôn phát hiện được?** Không; score có false negative. Cần xem class, khoảng cách, coverage và ngưỡng đúng sensor.
- **Vì sao frame score có thể cải thiện khi calibration sai?** Box rộng/truncated hoặc baseline mismatch cho phép điểm dịch vào box; tổng điểm có thể bù mất mát của object khác.
- **Vì sao nuScenes không phải so sánh accuracy công bằng với KITTI?** Nhãn 2D sinh từ 3D box, sensor/intrinsics/scene khác, chưa chuẩn hóa ontology.
- **Score test có dùng để chọn ngưỡng không?** Không. KITTI chia frame; nuScenes fit ngày và test scene đêm.
- **Tỷ lệ 0/20 có phải detector bỏ sót 100% không?** Không. Đây là projection retention, bài không chạy detector.
- **Chưa được gọi demo có mất điểm trình bày không?** Theo đề, giảng viên dùng REPORT để chấm người không được gọi.
