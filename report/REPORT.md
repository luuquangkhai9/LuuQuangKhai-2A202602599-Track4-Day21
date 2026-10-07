# Báo cáo Day 6: Độ nhạy của LiDAR–camera projection trước calibration drift

- **Họ tên:** Lưu Quang Khải
- **MSSV:** 2A202602599
- **Lớp:** Chờ xác nhận tên lớp từ học viên
- **Link repo:** https://github.com/luuquangkhai9/LuuQuangKhai-2A202602599-Track4-Day21
- **Topic:** A — LiDAR-camera projection QA
- **Dataset:** data/synthetic, data/kitti_mini, data/nuscenes_mini_subset
- **Các frame đã dùng:** Dự kiến 5 synthetic, toàn bộ 20 KITTI và 80 nuScenes; danh sách cụ thể sẽ lưu trong results/run_config.json.

## 1. Claim

Giả thuyết trước thí nghiệm: sai lệch yaw làm tăng độ dịch chuyển pixel và giảm tỷ lệ điểm object còn nằm trong 2D box; tỷ lệ điểm trong FOV có thể bỏ sót drift. Chưa xác nhận bằng số liệu.

## 2. Evidence

Chưa chạy benchmark. Tập điểm thuộc object được xác định với calibration gốc và giữ cố định giữa mọi mức perturb; điểm ra ngoài ảnh vẫn nằm trong mẫu số.

## 3. Failure case

Sẽ tìm lỗi Geometry do calibration lệch và lỗi Metric nếu FOV không phản ánh mismatch. Chỉ báo cáo trường hợp quan sát được từ code chạy thật.

## 4. Khuyến nghị nếu triển khai thật

Use-case ADAS: theo dõi calibration khi giá đỡ cảm biến bị dịch chuyển. Metric dựa vào label dùng cho QA offline; cần xác minh phương pháp không dùng label trước triển khai online.

## 5. Cách chạy lại

Môi trường Python CPU; cài requirements.txt. Lệnh tái tạo sẽ bổ sung sau khi thực nghiệm.

## 6. Khai báo sử dụng AI

Codex hỗ trợ lập kế hoạch, cài đặt code và kiểm tra thực nghiệm. Việc kiểm chứng của Codex sẽ được ghi lại; học viên cần tự chạy lại, đọc code và xác minh trước khi nộp.
