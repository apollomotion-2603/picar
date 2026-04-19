# NMPC Control Project (ROS2 + Acados)

Dự án này sử dụng mô hình động lực học và thuật toán NMPC (cài đặt bằng Acados) kết hợp với các package ROS2 để điều khiển quỹ đạo và vận tốc cho xe tự hành ảo trên Gazebo.

Dưới đây là các hướng dẫn chi tiết để thiết lập, hiệu chỉnh và chạy project.

---

## 1. Yêu cầu môi trường

Để NMPC node chạy được, Acados library phải được liên kết đúng lúc runtime thông qua 2 biến môi trường.

**Bắt buộc** ghi vào `~/.bashrc` để không phải lặp lại mỗi khi mở terminal mới:

```bash
# Cấu hình đường dẫn đến source code Acados
export ACADOS_SOURCE_DIR=/home/nhatranxuan/test/acados
# Thêm library của Acados vào LD_LIBRARY_PATH để ROS2 node dùng được (.so files)
export LD_LIBRARY_PATH=/home/nhatranxuan/test/acados/lib:$LD_LIBRARY_PATH
```
> **Lưu ý**: Sau khi thêm vào file, nhớ chạy dòng lệnh `source ~/.bashrc`.

---

## 2. Cách Build Package

Các node Python và C++ đều được build bằng công cụ `colcon` chuẩn của ROS2. Hệ thống nằm trong workspace: `~/test/Experiment/test8`.

#### Lệnh Build tiêu chuẩn
Lệnh này sẽ compile lại các thay đổi mới trong bất kỳ file C++ hay Python nào:

```bash
cd ~/test/Experiment/test8
source /opt/ros/foxy/setup.bash
colcon build --packages-select race_cars gazebo_ackermann_steering_vehicle
```

#### Lệnh Build Symlink (Khuyên dùng khi sửa node Của Python)
Chỉ dùng khi **đã tạo file `__init__.py`** và cập nhật `setup.py` dưới dạng flat-layout. Cờ `--symlink-install` cho phép sửa file code Python mà **không cần build lại**.
```bash
colcon build --packages-select race_cars --symlink-install
```

Sau khi `build` thành công, nhớ phải source environment:
```bash
source ~/test/Experiment/test8/install/setup.bash
```

---

## 3. Cách Run ROS2 Node

Tất cả đã được gộp vào 1 file Launch tổng, ngoài ra khi cần gỡ lỗi ta chia thành 2 terminal chạy riêng.

### Option A: Chạy tự động tất cả module trong 1 file (Launch Profile)
Sử dụng nếu bạn không cần thiết thay đổi hay gỡ lỗi riêng lẻ.
*Nhớ phải đổi `keyboard_controller` thành `nmpc_node` trong `vehicle.launch.py` thì node control mới có tác dụng (line 214).*
```bash
ros2 launch gazebo_ackermann_steering_vehicle vehicle.launch.py
```

### Option B: Tách NMPC Node thành tiến trình riêng (khuyên dùng khi Debug)
File config mặc định `vehicle.launch.py` sẽ load xe Gazebo + Pipeline Camera + Node xe thủ công (`keyboard_controller`). Bạn có thể nhìn thấy output của NMPC terminal dễ dàng.

**Terminal 1**: Mở Simulation & Pipeline Camera:
```bash
cd ~/test/Experiment/test8
source /opt/ros/foxy/setup.bash && source install/setup.bash
ros2 launch gazebo_ackermann_steering_vehicle vehicle.launch.py
```

**Terminal 2**: Mở NMPC Node:
```bash
cd ~/test/Experiment/test8
source /opt/ros/foxy/setup.bash && source install/setup.bash
# Đảm bảo bạn đã source ~/.bashrc có LD_LIBRARY_PATH của Acados
ros2 run race_cars racecar_nmpc_node
```

> Node NMPC sẽ in ra quá trình tính toán solver. Xe sẽ tự lên ga với target (vd: 2.0 m/s) và bám làn dựa vào dữ liệu Camera.

---

## 4. Cách thay đổi thông số / Sửa File

Tất cả các thông số chủ chốt để điều khiển nằm trong node ROS2 Python thuộc package `race_cars`.

### Thay đổi Vận tốc / Logic điều khiển ROS2
Mở file `~/test/Experiment/test8/src/race_cars/racecar_nmpc_node.py`
+ Dòng `self.target_v = 2.0` (quyết định tốc độ tham chiếu chạy xe Gazebo).
+ `self.Tf = 1.0` và `self.N = 50` là biến setup độ lớn quá trình tính toán solver của Horizon.

**Quy trình nếu sửa file này:** 
Nếu build với `--symlink-install`, bạn CHỈ cần **tắt chạy lại `ros2 run`** ở Terminal 2 để thay đổi có hiệu lực. Nếu không bạn cần `colcon build` lại package.

### Thay đổi Mô hình Động lực học
Việc cấu hình ma trận `Q`, `R`, số chiều input, state sẽ nằm ở:
+ `~/test/Experiment/test8/src/race_cars/bicycle_model.py`: Chứa Symbolic Variable (CasCaDi) cho các biến, hệ số khí động học, phương trình chuyển động của Ackermann.
+ `~/test/Experiment/test8/src/race_cars/acados_settings.py`: Khởi động biến và tạo ràng buộc không gian.

> **Cảnh báo**: Bất kỳ khi nào sửa đổi hai file `bicycle_model.py` hoặc `acados_settings.py`, phần mềm Acados buộc phải tạo ra C Code mới nhằm ánh xạ lại phương trình. Nếu KHÔNG TẠO LẠI, NMPC node sẽ sử dụng thư viện `.so` từ thuật toán cũ, dẫn tới tính toán sai. Vui lòng xem bước (5).

---

## 5. Dùng Acados sinh ra Code C mới (Regenerate Code)

Hệ thống của bài toán được mô tả bằng C Code được dịch từ CasADi python. 

**Khi nào cần tạo lại Code C?**
Khi bạn thay đổi hệ phương trình, ma trận của State, Control hoặc parameter (VD: Sửa code trong file `bicycle_model.py` hoặc `acados_settings.py`). 

**Các bước sử dụng Code sinh từ Acados:**

**Bước 1**: Đảm bảo environment liên quan Acados đã hoạt động
```bash
export ACADOS_SOURCE_DIR=/home/nhatranxuan/test/acados
export LD_LIBRARY_PATH=/home/nhatranxuan/test/acados/lib:$LD_LIBRARY_PATH
```

**Bước 2**: Chạy lệnh Generate trong thư mục của package để sinh mã `c_generated_code/`
```bash
cd /home/nhatranxuan/test/Experiment/test8/src/race_cars
python3 -c "from acados_settings import acados_settings; c, m, s = acados_settings(1.0, 50)"
```
Terminal xuất ra `OK — solver created successfully` là đã hoàn tất. 

> *Ghi chú: Lệnh `acados_settings.py` thực tế sẽ gọi OcpSolver và xuất các `.so` code mới đè lên folder code cũ. Lưu ý số `1.0` và `50` tương ứng số Tf và Horizon của bộ config mặc định.*

**Bước 3**: Không cần build `colcon build`.
Trình `race_cars` ROS2 ở runtime sẽ nạp trực tiếp file thư viện `libacados_ocp_solver_Spatialbicycle_model.so` mới được sinh ra từ Acados thông qua cơ chế `LD_LIBRARY_PATH`. Việc `ros2 run` lại thẳng vào Node NMPC luôn sẽ ghi nhận kết quả thuật toán mới nhất mà không cần compiler của ROS can thiệp.
