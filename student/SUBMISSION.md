# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Nguyễn Đức Triệu
- MSSV: 2A202602978
- Email: ductrieunguyen897@gmail.com
- Link repo (fork): https://github.com/ductrieunguyen897-code/K4-L2L3-DAY23-NguyenDucTrieu-2A202602978-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`):

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: `compare`, `[0, 198]`, `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, `0`
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: `0.9701 (97.01%)`, `0.7004 (70.04%)`, `519 / 16 / 222`
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: `0.1503 m`, `502`, `11.3436`, `0`, `239`, `2.5226`
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: `0.1359 m`, `502`, `9.2668`, `0`, `239`, `2.5226`
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss:
  Cả hai chế độ đều đạt số cặp confirmed track ghép với xe thật ground-truth bằng nhau (`matches = 502`) và không sinh ra bất kỳ track ảo nào (`ghost_track_frames = 0`), số ground-truth bị bỏ sót như nhau (`missed_gt_frames = 239`, chủ yếu do detector bỏ sót `fn = 222`). Điểm khác biệt quan trọng là chế độ Fused kết hợp đo đạc 2D camera giúp giảm tổng bình phương sai số từ `11.3436` xuống `9.2668`, làm giảm sai số vị trí không gian 3D RMSE từ `0.1503 m` xuống `0.1359 m` (cải thiện độ chính xác ~9.6%), chứng minh việc tích hợp thông tin góc quan sát của camera hỗ trợ EKF tinh chỉnh trạng thái tốt hơn so với chỉ dùng LiDAR đơn lẻ.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)?
   LiDAR đo trực tiếp toạ độ 3D trong không gian $z = [x, y, z]^T \in \mathbb{R}^3$ với ma trận hiệp phương sai nhiễu $R$ đơn vị mét vuông ($m^2$), quan hệ đo tuyến tính với ma trận $H$ cố định ($3 \times 6$). Ngược lại, camera đo toạ độ pixel 2D trên mặt phẳng ảnh $z = [u, v]^T \in \mathbb{R}^2$ với $R$ đơn vị pixel bình phương ($px^2$), quan hệ đo phi tuyến qua mô hình camera pinhole $h(x)$ nên ma trận $H$ ($2 \times 6$) là Jacobian cục bộ phụ thuộc vào trạng thái hiện tại $x$.

2. Vì sao cần gating Mahalanobis trước khi gán?
   Khoảng cách Mahalanobis $d^2 = \gamma^T S^{-1} \gamma$ tính khoảng cách sai lệch có trọng số theo ma trận hiệp phương sai sai số $S = H P H^T + R$, phản ánh chính xác độ tin cậy và phân bố không gian của cả cảm biến lẫn track (thay vì khoảng cách hình học thông thường). Gating với phân phối $\chi^2$ giúp loại bỏ các đo đạc ngoại lai (outliers) hoặc vật thể không liên quan trước khi giải thuật ghép cặp tham lam thực thi, giảm thiểu nguy cơ gán nhầm đối tượng khi mật độ giao thông cao.

3. Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.
   Pipeline là **track-then-fuse**: hệ thống duy trì một danh sách track duy nhất, ở mỗi frame thực hiện EKF predict trước, sau đó gán và cập nhật với đo LiDAR (AssocL), rồi tiếp tục gán và cập nhật bổ sung với đo Camera (AssocC) trên cùng các track đó. Trên log `fusion-run-lab`, trong mỗi frame luôn in ra tiến trình: predict trạng thái $\rightarrow$ trích xuất hộp 3D LiDAR và AssocL để cập nhật lifecycle $\rightarrow$ nạp đo FRONT camera và AssocC để tinh chỉnh toạ độ EKF.

4. Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?
   Khi camera lệch calibration (sai tham số nội intrinsics hoặc ngoại extrinsics), giá trị dự báo hình chiếu $h(x)$ sẽ bị lệch có hệ thống so với vị trí pixel thực tế $z$. Triệu chứng xuất hiện là vector innovation $\gamma = z - h(x)$ bị lệch hằng số (systematic bias, kỳ vọng sai số khác 0). Điều này khiến bình phương khoảng cách Mahalanobis $d^2$ tăng cao vượt ngưỡng $\chi^2$ làm mất ghép cặp (miss), hoặc nếu vẫn ghép được thì Kalman gain $K$ sẽ kéo lệch vị trí ước lượng của track khiến sai số RMSE tăng vọt.

5. Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng?
   Giải thích vì sao lidar quyết định score/init/delete còn camera chỉ EKF update.
   Hàm cần `sensor` tường minh vì ngay cả khi frame không có phép đo nào (`meas_list` rỗng), hệ thống vẫn phải gọi `manager.manage_tracks(unassigned_tracks, [], sensor)`: nếu sensor là LiDAR thì các track chưa gán nằm trong tầm nhìn (in FOV) sẽ bị phạt trừ điểm miss và xét xoá; còn nếu sensor là Camera thì không được trừ điểm hay xoá track. LiDAR quyết định vòng đời vì cung cấp đầy đủ kích thước và toạ độ 3D đáng tin cậy để tạo và duy trì track 3D; trong khi Camera 2D chỉ có góc chiếu 2D (mất thông tin độ sâu) và dễ bị nhiễu/che khuất nên chỉ được dùng để cập nhật tinh chỉnh trạng thái EKF mà không được quyền sinh hoặc huỷ track.

6. Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track.
   - Điều kiện xác nhận: Track chuyển sang trạng thái `confirmed` khi điểm tin cậy `score > confirmed_threshold` (0.8).
   - Giữ confirmed sau miss: Khi gặp frame LiDAR không có đo tương ứng (miss trong FOV), track bị trừ điểm `1/window`, nhưng nếu đã ở trạng thái `confirmed` thì vẫn giữ nguyên trạng thái `confirmed` chứ không bị hạ cấp.
   - Điều kiện xoá track: Xoá khi phương sai vị trí mặt phẳng ngang vượt giới hạn ($P_{xx} > \text{max\_P}$ hoặc $P_{yy} > \text{max\_P}$ với $\text{max\_P} = 9.0\text{ m}^2$), hoặc track đã `confirmed` có `score < delete_threshold` (0.6), hoặc track chưa confirmed (`tentative` / `initialized`) có `score <= 0.0`.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- Không

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Antigravity AI Assistant
- Dùng cho phần nào (hàm, câu hỏi, debug): Hỗ trợ phân tích mã nguồn EKF, kiểm tra tính toán ma trận Jacobian, chạy script kiểm thử và đối chiếu kết quả theo rubric.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): Chạy toàn bộ 128 kiểm thử tự động pytest (đạt 100% pass), chạy lệnh fusion-run-lab so sánh trên toàn bộ 199 frame dữ liệu Waymo, và kiểm tra hợp lệ bằng tools/check_submission.py.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [x] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
