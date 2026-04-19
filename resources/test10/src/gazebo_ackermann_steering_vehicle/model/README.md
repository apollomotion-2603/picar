# Car URDF Xacro - Hướng Dẫn Chi Tiết

## Giới Thiệu

Tệp `car.urdf.xacro` định nghĩa mô hình robot xe tự lái Ackermann cho mô phỏng trong Gazebo. Nó sử dụng **Xacro** (XML Macros) để tạo cấu trúc linh hoạt và tái sử dụng được.

---

## Data Structure - Cấu Trúc Dữ Liệu

### 1. **Properties (Thuộc Tính)**

```xml
<xacro:property name="property_name" value="value"/>
```

**Ý nghĩa:** Khai báo các biến toàn cục để tái sử dụng trong toàn file

| Property | Giá Trị | Đơn Vị | Ý Nghĩa |
|----------|--------|--------|---------|
| `chassis_length` | 0.5 | m | Chiều dài thân xe |
| `chassis_width` | 0.3 | m | Chiều rộng thân xe |
| `chassis_height` | 0.1 | m | Chiều cao thân xe |
| `wheel_radius` | 0.1 | m | Bán kính bánh xe |
| `wheel_width` | 0.05 | m | Độ dày/chiều rộng bánh xe |
| `wheel_x_offset` | 0.2 | m | Khoảng cách bánh xe theo trục dọc (X) |
| `wheel_y_offset` | 0.175 | m | Khoảng cách bánh xe theo trục ngang (Y) |

**Cách sử dụng:** Tham chiếu với cú pháp `${property_name}`

---

### 2. **Macros (Mẫu Tái Sử Dụng)**

#### **2.1 box_inertia - Quán Tính Hình Hộp**

```xml
<xacro:macro name="box_inertia" params="m w h d">
  <inertial>
    <mass value="${m}"/>
    <inertia ixx="${(m/12) * (h*h + d*d)}" 
             ixy="0.0" ixz="0.0" 
             iyy="${(m/12) * (w*w + d*d)}" 
             iyz="0.0" 
             izz="${(m/12) * (w*w + h*h)}"/>
  </inertial>
</xacro:macro>
```

**Tham số:**
- `m`: Khối lượng (kg)
- `w`: Chiều rộng (width) - m
- `h`: Chiều cao (height) - m
- `d`: Chiều sâu (depth) - m

**Ma Trận Quán Tính (3x3):**

$$\begin{pmatrix}
I_{xx} & 0 & 0 \\
0 & I_{yy} & 0 \\
0 & 0 & I_{zz}
\end{pmatrix}$$

Trong đó:
- $I_{xx} = \frac{m}{12}(h^2 + d^2)$ - Quán tính quay quanh **trục X**
- $I_{yy} = \frac{m}{12}(w^2 + d^2)$ - Quán tính quay quanh **trục Y**
- $I_{zz} = \frac{m}{12}(w^2 + h^2)$ - Quán tính quay quanh **trục Z**

**Giải thích:**
- Các thành phần off-diagonal (0.0) vì vật thể có tính đối xứng
- Càng lớn giá trị I, càng khó quay quanh trục đó

#### **2.2 cylinder_inertia - Quán Tính Hình Trụ**

```xml
<xacro:macro name="cylinder_inertia" params="m r h">
  <inertial>
    <mass value="${m}"/>
    <inertia ixx="${(m/12) * (3*r*r + h*h)}" 
             ixy="0" ixz="0" 
             iyy="${(m/12) * (3*r*r + h*h)}" 
             iyz="0" 
             izz="${(m/2) * (r*r)}"/>
  </inertial>
</xacro:macro>
```

**Tham số:**
- `m`: Khối lượng (kg)
- `r`: Bán kính (radius) - m
- `h`: Chiều cao (height) - m

**Công thức:** Khác với hình hộp vì hình dạng khác nhau

---

### 3. **Links (Liên Kết / Thành Phần)**

Link là các thành phần vật lý của robot

```xml
<link name="link_name">
  <visual><!-- Hình dáng để hiển thị --></visual>
  <collision><!-- Hình dáng để phát hiện va chạm --></collision>
  <inertial><!-- Tính chất vật lý --></inertial>
</link>
```

#### **3.1 base_link - Liên Kết Gốc**

```xml
<link name="base_link">
</link>
```

**Ý nghĩa:** Frame tham chiếu chính của robot (thường đặt tại tâm)

#### **3.2 chassis - Thân Xe**

```xml
<link name="chassis">
  <visual>
    <geometry>
      <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
    </geometry>
    <material name="blue"><color rgba="0 0 1 1"/></material>
  </visual>
  <collision>
    <geometry>
      <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
    </geometry>
  </collision>
  <xacro:box_inertia m="10.0" w="${chassis_width}" d="${chassis_length}" h="${chassis_height}"/>
</link>
```

**Chassis Link có 3 thành phần chính:**

##### **3.2.1 Visual (Hình Dáng Hiển Thị)**

```xml
<visual>
  <geometry>
    <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
    <!-- Hình hộp: 0.5m × 0.3m × 0.1m -->
  </geometry>
  <material name="blue"><color rgba="0 0 1 1"/></material>
</visual>
```

| Phần | Chi Tiết |
|-----|---------|
| **\<geometry>** | Định nghĩa hình dạng 3D |
| **\<box>** | Hình hộp có kích thước 0.5 × 0.3 × 0.1 m |
| **\<material>** | Màu sắc hiển thị |
| **rgba** | (Red=0, Green=0, Blue=1, Alpha=1) = **Xanh lam đầy đủ** |

**Ý nghĩa:** Định nghĩa **hình dáng và màu sắc để hiển thị** trong Gazebo. Khi chạy mô phỏng, bạn sẽ thấy một hình hộp xanh đại diện cho thân xe.

---

##### **3.2.2 Collision (Hình Dáng Va Chạm)**

```xml
<collision>
  <geometry>
    <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
  </geometry>
</collision>
```

| Khía Cạnh | Chi Tiết |
|----------|---------|
| **Tác dụng** | Định nghĩa hình dạng để **phát hiện va chạm** |
| **Hình dáng** | Thường giống với Visual |
| **Màu sắc** | Không có (vô hình) |
| **Động cơ vật lý** | Sử dụng hình dạng này để tính toán va chạm |

**Ví dụ:** Khi xe va vào vật cản, động cơ vật lý dùng collision shape để phát hiện tác động.

---

##### **3.2.3 Inertial (Tính Chất Vật Lý)**

```xml
<xacro:box_inertia m="10.0" w="${chassis_width}" d="${chassis_length}" h="${chassis_height}"/>
```

**Tham số:**
- **m="10.0":** Khối lượng = **10 kg**
- **w="${chassis_width}":** Chiều rộng = 0.3 m
- **d="${chassis_length}":** Chiều dài = 0.5 m
- **h="${chassis_height}":** Chiều cao = 0.1 m

**Macro này sinh ra:**
```xml
<inertial>
  <mass value="10.0"/>
  <inertia ixx="0.208" ixy="0.0" ixz="0.0" iyy="0.238" iyz="0.0" izz="0.142"/>
</inertial>
```

| Phần | Ý Nghĩa |
|-----|---------|
| **mass** | Khối lượng xe = 10kg |
| **inertia** | Ma trận quán tính để tính toán chuyển động quay |

**Ý nghĩa:** Định nghĩa **tính chất vật lý** để động cơ mô phỏng tính toán chuyển động, gia tốc, quay của xe một cách chính xác.

---

##### **3.2.4 So Sánh 3 Thành Phần:**

| Thành Phần | Tác Dụng | Màu/Hiển Thị | Công Thức |
|-----------|---------|-------------|----------|
| **Visual** | Để nhìn thấy | Có màu (xanh) | Hình học đơn giản |
| **Collision** | Phát hiện va chạm | Vô hình | Cùng hình dáng visual |
| **Inertial** | Tính toán vật lý | Không hiển thị | Khối lượng + quán tính |

---

##### **3.2.5 Sơ Đồ Minh Họa:**

```
┌──────────────────────────────────┐
│     Gazebo Rendering Engine      │
│   (Hiển thị Visual)              │
│     Hộp xanh 0.5×0.3×0.1 m       │
└──────────────────────────────────┘
              │
    ┌─────────┴─────────┐
    ▼                   ▼
┌────────────┐    ┌────────────────┐
│ Physics    │    │ Graphics       │
│ Engine     │    │ Engine         │
│            │    │                │
│ Collision: │    │ Visual:        │
│ Hộp        │    │ Hộp xanh       │
│            │    │                │
│ Inertial:  │    │                │
│ 10kg       │    │                │
│ Quán tính  │    │                │
└────────────┘    └────────────────┘
   (Va chạm)     (Nhìn thấy)
```

---

##### **3.2.6 Khi Mô Phỏng Xảy Ra:**

1. **Khởi động:** Gazebo đọc cấu hình Chassis Link
2. **Hiển thị:** Vẽ hình hộp xanh (từ Visual)
3. **Vật lý:** Physics Engine tải Inertial (10kg)
4. **Tương tác:** Khi có va chạm, dùng Collision shape để phát hiện và Inertial để tính lực tác dụng

#### **3.3 Bánh Xe (Wheels) - Ackermann Steering Model**

Robot sử dụng **mô hình Ackermann steering** (như xe thực tế):

**Bánh Sau (Rear Wheels) - ACTIVE (Dẫn Động):**
- `rear_left_wheel`, `rear_right_wheel`: Chỉ quay (không lái)
- **Có motor dẫn động** → Quay liên tục (type=continuous)
- **Tác dụng:** Đẩy xe tiến tới
- **Quán tính:** 2kg mỗi bánh

**Bánh Trước (Front Wheels) - PASSIVE (Bị Đẩy):**
- `front_left_wheel`, `front_right_wheel`: Quay + xoay lái
- **KHÔNG có motor dẫn động** → Bị chassis đẩy
- **Tác dụng:** Điều khiển hướng (lái) + Bị kéo theo (passive rolling)
- **Quán tính:** 2kg mỗi bánh
- `front_left/right_steering_link`: Cơ cấu để điều khiển góc lái

---

#### **3.3.1 Cơ Chế Ackermann Steering:**

```
Motor → Bánh Sau Quay → Lực Ma Sát → Chassis Chuyển Động → Kéo Bánh Trước
  ↓         ↓              ↓               ↓                      ↓
Năng      Chủ động      Tác động      Dẫn động            Bị động/Passive
lượng     (Active)      (Tác dụng)                         (Rolling)
```

**Quy trình chi tiết:**

1. **Motor dẫn động bánh sau:**
   - Motor quay bánh sau với moment $\tau$
   - Bánh sau tạo lực ma sát với mặt đất
   
2. **Lực ma sát đẩy chassis:**
   - Lực ma sát = $F = f \times m \times g$ (ma sát động học)
   - Chassis nhận lực và chuyển động tiến tới
   
3. **Chassis chuyển động kéo bánh trước:**
   - Bánh trước được gắn vào chassis qua steering joint
   - Khi chassis chuyển động → bánh trước bị kéo theo
   - **Bánh trước KHÔNG có motor riêng** → chỉ lăn passive
   
4. **Điều khiển góc lái:**
   - Steering joint xoay quanh trục Z (-0.6 đến 0.6 radian)
   - Thay đổi góc giữa bánh trước và chassis
   - Kết quả: Xe quay theo hướng mong muốn

---

#### **3.3.2 So Sánh Bánh Sau vs Bánh Trước:**

| Khía Cạnh | Bánh Sau (Rear) | Bánh Trước (Front) |
|-----------|-----------------|-------------------|
| **Motor** | ✅ Có motor | ❌ Không motor |
| **Tác dụng** | Dẫn động (Active) | Bị đẩy (Passive) |
| **Joint Type** | `continuous` | `revolute` + `continuous` |
| **Quay** | Tự do 360° | Góc lái giới hạn ±34° |
| **Lực** | Từ motor | Từ chassis (ma sát) |
| **Chuyên viên** | Tăng tốc/Tốc độ | Điều hướng/Lái |
| **Phương trình** | $\tau = I \cdot \alpha$ | $F = m \cdot a$ |

---

#### **3.3.3 Cấu Trúc Joint của Bánh Trước:**

```
Chassis
   │
   ├─ steering_joint (type=revolute)
   │  └─ steering_link
   │     │
   │     └─ wheel_joint (type=continuous)
   │        └─ wheel (bánh thực tế)
```

**Tại sao 2 joints?**

1. **steering_joint:** Điều khiển góc lái
   - Quanh trục Z
   - Giới hạn: -0.6 đến 0.6 radian (±34°)
   
2. **wheel_joint:** Bánh lăn
   - Quay liên tục
   - **PARENT = steering_link** (chứ không phải chassis)
   - Khi steering_link xoay → bánh theo
   - Không có motor → lăn passive

---

#### **3.3.4 Hình Minh Họa Cấu Trúc:**

```
          Base Link
             │
             ├─ Chassis Joint (fixed)
             │
          Chassis (thân xe, 10kg)
             │
    ┌────────┼────────┐
    │        │        │
Bánh Sau  Bánh Sau  Cơ Cấu Lái
  Trái     Phải    (Bánh Trước)
    │        │        │
    │        │     ┌──┴─────────┐
    │        │     │            │
    │        │  steering_joint steering_joint
    │        │  (revolute)     (revolute)
    │        │     │            │
    │        │  steering_link steering_link
    │        │     │            │
    │        │  wheel_joint   wheel_joint
    │        │  (continuous)  (continuous)
    │        │     │            │
    │        │   Bánh FL      Bánh FR
    
Ký hiệu:
- Bánh Sau: Có motor → Dẫn động
- Bánh Trước: Không motor → Bị đẩy + Lái
```

---

#### **3.3.5 Khi Xe Chạy - Quy Trình:**

```
1️⃣ Controller gửi lệnh:
   ├─ Tốc độ bánh sau (RPM hoặc moment)
   └─ Góc lái bánh trước (radian)

2️⃣ Motor dẫn động:
   ├─ Quay bánh sau với moment τ
   └─ Tạo lực ma sát F = τ / r

3️⃣ Chassis chuyển động:
   ├─ Nhận lực từ bánh sau
   ├─ Tính gia tốc: a = F / m_total
   └─ Chuyển động tiến tới

4️⃣ Bánh trước bị kéo theo:
   ├─ Steering link xoay (góc lái)
   ├─ Bánh trước lăn passive
   └─ Xe thay đổi hướng

5️⃣ Kết quả:
   └─ Xe tiến tới + quay hướng (như xe thực tế)
```

---

#### **3.3.6 Công Thức Tính Toán:**

**Gia tốc xe (từ lực bánh sau):**

$$a = \frac{F_{friction}}{m_{total}} = \frac{\tau / r_{wheel}}{m_{chassis} + 4 \times m_{wheel}}$$

**Với giá trị cụ thể:**
- $\tau = 1$ N⋅m (moment motor)
- $r = 0.1$ m (bán kính bánh)
- $m_{chassis} = 10$ kg
- $m_{wheel} = 2$ kg (×4 bánh)

$$a = \frac{1 / 0.1}{10 + 4 \times 2} = \frac{10}{18} = 0.556 \text{ m/s}^2$$

**Vận tốc theo thời gian:**

$$v(t) = a \times t = 0.556 \times t$$

Ví dụ: Sau 2 giây, $v = 1.112$ m/s

---

#### **3.3.7 Ý Nghĩa Tương Tác Quán Tính:**

```
Motor (Năng lượng)
     ↓
Bánh Sau ($I_{wheel} = 0.01$ kg⋅m²)
     ↓ Chống lại bởi quán tính bánh
Lực Ma Sát
     ↓
Chassis ($I_{chassis} = 0.0917$ kg⋅m²)
     ↓ Chống lại bởi quán tính chassis
Gia Tốc Tuyến Tính
     ↓
Bánh Trước (bị kéo)
     ↓
Xe Chuyển Động
```

**Kết luận:**
- **Quán tính bánh après** chống lại tăng tốc góc của bánh
- **Quán tính chassis** chống lại tăng tốc tuyến tính của xe
- **Quán tính tổng cộng** quyết định xe tăng tốc nhanh hay chậm
- Các bánh trước bị kéo theo → không tiêu tốn năng lượng riêng

---

#### **3.4 camera_link - Camera Link**

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
    <material name="red"><color rgba="1 0 0 1"/></material>
  </visual>
</link>
```

**Ý nghĩa:** Đại diện cho camera trên xe (hình hộp đỏ 5cm × 5cm × 5cm)

---

### 4. **Joints (Khớp)**

Joint kết nối các links và định nghĩa cách chúng chuyển động

```xml
<joint name="joint_name" type="joint_type">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="x y z" rpy="roll pitch yaw"/>
  <axis xyz="x y z"/>
  <limit lower="min" upper="max" effort="effort" velocity="velocity"/>
</joint>
```

#### **4.1 Kiểu Joint Chính:**

| Kiểu | Tác Dụng | Ví Dụ |
|------|---------|-------|
| `fixed` | Không chuyển động, cố định | chassis_joint |
| `continuous` | Quay liên tục (0 đến 2π) | rear_wheel_joint |
| `revolute` | Quay có giới hạn góc | front_steering_joint |

#### **4.2 Các Joint Quan Trọng:**

**chassis_joint:**
```xml
<joint name="chassis_joint" type="fixed">
  <parent link="base_link"/>
  <child link="chassis"/>
  <origin xyz="0 0 ${wheel_radius}"/>
</joint>
```

**Chi tiết:**
- **name="chassis_joint":** Tên của khớp
- **type="fixed":** Khớp cố định - không chuyển động
- **\<parent link="base_link"/>:** Link cha (frame gốc của robot)
- **\<child link="chassis"/>:** Link con (thân xe)
- **origin xyz="0 0 ${wheel_radius}":** Vị trí tương đối giữa parent và child

**Origin - Vị Trí Tương Đối:**
```
xyz="0 0 ${wheel_radius}"
    ↓ ↓ ↓
    X Y Z

- X = 0  → Không dịch chuyển theo trục dọc
- Y = 0  → Không dịch chuyển theo trục ngang
- Z = ${wheel_radius} (0.1m) → Nâng lên 0.1m
```

**Tại Sao Cần Nâng Lên Z = 0.1m?**

Đây là **khoảng cách bằng bán kính bánh xe**. Mục đích là **đặt thân xe ở độ cao chính xác sao cho bánh xe tiếp xúc với mặt đất**.

Sơ đồ minh họa:
```
             Mặt đất
        ═════════════════ (Z = 0)
             ↑ 0.1m
       ┌─────────────┐
       │   Chassis   │  ← (Z = 0.1m)
       │ (Thân xe)   │
       └─────┬───┬───┘
            ◯   ◯     ← Bánh xe (Z = 0)
```

**Nếu không nâng (Z = 0):** Thân xe sẽ chìm vào đất, không khớp với hình học thực tế.

**Nếu nâng lên đúng lượng (Z = 0.1m):** Thân xe nằm ở độ cao phù hợp, bánh xe tiếp xúc mặt đất tự nhiên.

**rear_wheel_joint (trong macro):**
```xml
<joint name="${name}_wheel_joint" type="continuous">
  <parent link="chassis"/>
  <child link="${name}_wheel"/>
  <origin xyz="-${wheel_x_offset} ${y_offset} 0" rpy="-1.5708 0 0"/>
  <axis xyz="0 0 ${flip_axis}"/>
</joint>
```
- **Loại:** Continuous (quay liên tục)
- **Origin:** X = -0.2m (sau), Y = ±0.175m (trái/phải)
- **RPY:** -1.5708 rad = -90° (quay trục Y để bánh nằm ngang)

**front_steering_joint (trong macro):**
```xml
<joint name="${name}_steering_joint" type="revolute">
  <parent link="chassis"/>
  <child link="${name}_steering_link"/>
  <origin xyz="${wheel_x_offset} ${y_offset} 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.6" upper="0.6" effort="100" velocity="1.0"/>
</joint>
```
- **Loại:** Revolute (quay có giới hạn)
- **Giới hạn góc:** -0.6 đến 0.6 radian (±34°)
- **Trục:** Z (quay theo phương đứng để điều khiển góc lái)

**camera_joint:**
```xml
<joint name="camera_joint" type="fixed">
  <parent link="chassis"/>
  <child link="camera_link"/>
  <origin xyz="0.25 0 0.125" rpy="0 0 0"/>
</joint>
```
- **Vị trí:** Phía trước (X=0.25m) giữa (Y=0), cao (Z=0.125m)

---

### 5. **Macros cho Bánh Xe**

#### **5.1 rear_wheel - Bánh Sau**

```xml
<xacro:macro name="rear_wheel" params="name y_offset flip_axis">
  <link name="${name}_wheel">
    <!-- Visual + Collision + Inertia -->
  </link>
  <joint name="${name}_wheel_joint" type="continuous">
    <!-- Kết nối bánh với thân xe -->
  </joint>
</xacro:macro>
```

**Tham số:**
- `name`: Tên bánh (ví dụ: `rear_left`)
- `y_offset`: Vị trí ngang (±0.175m cho trái/phải)
- `flip_axis`: Hướng quay (1 hoặc -1)

#### **5.2 front_wheel - Bánh Trước**

```xml
<xacro:macro name="front_wheel" params="name y_offset flip_axis">
  <!-- Steering link (để điều khiển góc lái) -->
  <link name="${name}_steering_link">...</link>
  <joint name="${name}_steering_joint" type="revolute">...</joint>
  
  <!-- Wheel link -->
  <link name="${name}_wheel">...</link>
  <joint name="${name}_wheel_joint" type="continuous">...</joint>
</xacro:macro>
```

**Điểm khác biệt:** Có thêm steering_link để kiểm soát góc lái

---

### 6. **Gazebo Plugin - Điều Khiển Xe (Ackermann Drive Controller)**

#### **6.1 ROS2 Controller Architecture**

```
┌─────────────────────────────────────────────────────────────┐
│              ROS2 Controller Node                           │
│                                                             │
│  - Tiếp nhận lệnh từ người dùng                            │
│  - Tính toán linear/angular velocity                       │
│  - Gửi lệnh tới /my_bot/cmd_vel                            │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼ ROS2 Topic
         ┌─────────────────────────┐
         │  /my_bot/cmd_vel        │
         │  geometry_msgs/Twist    │
         │  linear.x = v (m/s)     │
         │  angular.z = ω (rad/s)  │
         └────────────┬────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────┐
│     Gazebo Ackermann Drive Plugin                          │
│  (gazebo_ros_ackermann_drive)                              │
│                                                             │
│  ┌────────────────────────────────────────┐               │
│  │ Step 1: Parse ROS Command              │               │
│  │ ├─ linear.x = tốc độ tiến tới (m/s)   │               │
│  │ └─ angular.z = tốc độ quay (rad/s)    │               │
│  └────────┬───────────────────────────────┘               │
│           │                                                 │
│  ┌────────▼────────────────────────────────┐              │
│  │ Step 2: Ackermann Kinematics Solver    │              │
│  │ ├─ Tính toán bánh sau cần quay bao     │              │
│  │ │  nhanh để đạt v_linear                │              │
│  │ └─ Tính toán góc lái bánh trước để     │              │
│  │    đạt v_angular                       │              │
│  └────────┬───────────────────────────────┘              │
│           │                                                 │
│  ┌────────▼─────────────────────────────────┐             │
│  │ Step 3: Motor Control (Torque)          │             │
│  │ ├─ Rear Left: τ = f(v_desired - v_act) │             │
│  │ └─ Rear Right: τ = f(v_desired - v_act)│             │
│  └────────┬──────────────────────────────────┘             │
│           │                                                 │
│  ┌────────▼────────────────────────────────┐              │
│  │ Step 4: Steering PID Control            │              │
│  │ ├─ Left: θ = Kp*e + Ki*∫e + Kd*de/dt  │              │
│  │ └─ Right: θ = Kp*e + Ki*∫e + Kd*de/dt │              │
│  │   Với Kp=1000, Ki=0, Kd=1              │              │
│  └────────┬────────────────────────────────┘              │
└───────────┼──────────────────────────────────────────────────┘
            │
    ┌───────┴────────┬──────────────┬──────────────┐
    │                │              │              │
    ▼                ▼              ▼              ▼
┌─────────┐    ┌─────────┐    ┌──────────┐    ┌──────────┐
│ τ_rear_ │    │ τ_rear_ │    │ θ_left_  │    │ θ_right_ │
│ left    │    │ right   │    │ steering │    │ steering │
└────┬────┘    └────┬────┘    └────┬─────┘    └────┬─────┘
     │              │              │              │
     ▼              ▼              ▼              ▼
┌──────────────────────────────────────────────────────────┐
│         Physics Engine (ODE/Bullet)                      │
│                                                          │
│  ┌─ Receive Commands from Plugin                        │
│  │                                                       │
│  ├─ Rear Wheels:                                        │
│  │  τ = I*α → Angular Acceleration                      │
│  │  → Friction Force on Ground                          │
│  │  → Linear Force to Chassis                           │
│  │                                                       │
│  ├─ Front Wheels:                                       │
│  │  θ = Steering Angle                                  │
│  │  → Passive Rolling (driven by chassis)               │
│  │                                                       │
│  ├─ Chassis:                                            │
│  │  F = m*a → Linear Acceleration                       │
│  │  τ = I*α → Angular Acceleration                      │
│  │                                                       │
│  └─ Calculate:                                          │
│     Position (x, y), Heading (θ)                        │
└───────────┬──────────────────────────────────────────────┘
            │
            ▼
   ┌─────────────────────┐
   │ Odometry Calculation│
   │                     │
   │ From Wheel Speeds → │
   │ Robot Pose & Twist  │
   └────────┬────────────┘
            │
            ▼
   ┌──────────────────────────┐
   │ Publish Results          │
   │ ├─ /my_bot/odom         │
   │ │  (position + velocity) │
   │ ├─ /tf (transforms)      │
   │ └─ /wheel_states         │
   └──────────────────────────┘
```

---

#### **6.2 Control Loop Chi Tiết**

**Quy trình khi xe nhận lệnh:**

```
1️⃣ Lệnh Từ Controller:
   ┌──────────────────┐
   │ cmd_vel:         │
   │ linear.x = 2 m/s │
   │ angular.z = 0 rad/s
   └────────┬─────────┘
            │
            ▼
2️⃣ Ackermann Kinematics:
   ├─ Wheel Speed: ω = v/r = 2/0.1 = 20 rad/s
   ├─ Steering Angle: θ = 0° (không lái)
   └─ Tính toán error: e = ω_desired - ω_current
            │
            ▼
3️⃣ Motor Control (Rear Wheels):
   ├─ Motor model: τ = K*e = 100*(20 - ω_current)
   ├─ Gửi τ → rear_left_wheel_joint
   └─ Gửi τ → rear_right_wheel_joint
            │
            ▼
4️⃣ Physics Simulation (100 Hz):
   ├─ Bánh sau quay: α = τ/I
   │  └─ ω_rear = ω_current + α*dt
   │
   ├─ Friction: F = μ*m*g
   │  └─ Chassis nhận lực F
   │
   ├─ Chassis gia tốc: a = F/m_total
   │  └─ v_chassis = v_current + a*dt
   │
   └─ Bánh trước bị kéo theo:
      └─ v_front = v_chassis (passive)
            │
            ▼
5️⃣ Odometry & Feedback:
   ├─ Tính vị trí mới: x(t+dt) = x(t) + v*dt
   ├─ Tính hướng mới: θ(t+dt) = θ(t) + ω*dt
   └─ Xuất /my_bot/odom để controller biết
            │
            ▼
6️⃣ Lặp lại (100 lần/giây)
```

---

#### **6.3 Cấu Hình Plugin Chi Tiết**

```xml
<plugin name="gazebo_ros_ackermann_drive" filename="libgazebo_ros_ackermann_drive.so">
  <!-- ROS Namespace & Topics -->
  <ros>
    <namespace>my_bot</namespace>                    <!-- Namespace: /my_bot -->
    <remapping>cmd_vel:=cmd_vel</remapping>          <!-- Input topic -->
    <remapping>odom:=odom</remapping>                <!-- Output topic -->
  </ros>

  <!-- Update Frequency -->
  <update_rate>100.0</update_rate>                   <!-- 100 Hz simulation -->

  <!-- Rear Wheels (ACTIVE - Driven) -->
  <rear_left_joint>rear_left_wheel_joint</rear_left_joint>
  <rear_right_joint>rear_right_wheel_joint</rear_right_joint>
  
  <!-- Front Wheels (PASSIVE - Steered) -->
  <front_left_joint>front_left_wheel_joint</front_left_joint>
  <front_right_joint>front_right_wheel_joint</front_right_joint>
  
  <!-- Steering Control Joints -->
  <left_steering_joint>front_left_steering_joint</left_steering_joint>
  <right_steering_joint>front_right_steering_joint</right_steering_joint>

  <!-- Kinematic Limits -->
  <max_steer>0.6</max_steer>        <!-- ±0.6 rad (±34°) -->
  <max_speed>20</max_speed>          <!-- 20 m/s max -->

  <!-- PID Gains for Steering Control -->
  <!-- Format: Kp Ki Kd -->
  <left_steering_pid_gain>1000 0 1</left_steering_pid_gain>
  <right_steering_pid_gain>1000 0 1</right_steering_pid_gain>

  <!-- Output Configuration -->
  <publish_odom>true</publish_odom>                  <!-- Publish odometry -->
  <publish_odom_tf>true</publish_odom_tf>            <!-- Publish TF -->
  <publish_wheel_tf>true</publish_wheel_tf>          <!-- Publish wheel TF -->
  
  <!-- Reference Frames -->
  <odometry_frame>odom</odometry_frame>              <!-- World frame -->
  <robot_base_frame>base_link</robot_base_frame>    <!-- Robot frame -->
</plugin>
```

---

#### **6.4 Torque vs Steering Angle**

| Komponen | Type | Tác Dụng | Input | Công Thức |
|----------|------|---------|-------|-----------|
| **Bánh Sau** | continuous | Dẫn động | Torque (τ) | $\tau = I \times \alpha$ |
| **Bánh Trước** | revolute | Lái | Góc (θ) | $\theta = K_p \times e$ |

**Ví dụ Torque Calculation:**

```
Desired speed: v = 5 m/s
Wheel radius: r = 0.1 m
Desired ω = v/r = 50 rad/s

Current ω = 30 rad/s
Error e = 50 - 30 = 20 rad/s

Torque (simplified motor model):
τ = 100 * e = 100 * 20 = 2000 N⋅m

τ → rear_left_wheel_joint
τ → rear_right_wheel_joint
```

---

#### **6.5 Control Loop Feedback**

```
┌────────────────────────────────────────────────┐
│  Desired Command (from user)                   │
│  /my_bot/cmd_vel: v_lin=2 m/s, v_ang=0 rad/s  │
└─────────────┬────────────────────────────────────┘
              │
              ▼
    ┌─────────────────────────┐
    │  Plugin Calculation     │
    │  ├─ ω_des = 20 rad/s   │
    │  └─ θ_des = 0°         │
    └────────┬────────────────┘
             │
             ▼
    ┌──────────────────────────┐
    │  Feedback from Physics   │
    │  ├─ ω_actual = 15 rad/s │
    │  └─ θ_actual = 0.05°    │
    └────────┬─────────────────┘
             │
             ▼
    ┌──────────────────────────┐
    │  Error Calculation       │
    │  ├─ e_ω = 20-15 = 5     │
    │  └─ e_θ = 0-0.05 = -0.05│
    └────────┬─────────────────┘
             │
             ▼
    ┌──────────────────────────────┐
    │  Adjust Output              │
    │  ├─ τ ↑ (tăng torque)       │
    │  └─ θ ↑ (tăng steering)     │
    └────────┬─────────────────────┘
             │
             ▼
    ┌──────────────────────────┐
    │  Next Iteration (100 Hz) │
    └──────────────────────────┘
```

---

#### **6.6 Các Joint Được Kết Nối**

**Plugin kết nối 6 joints:**

```
gazebo_ros_ackermann_drive
│
├─ DRIVE JOINTS (Quay bánh)
│  ├─ rear_left_wheel_joint   ← Nhận τ_rear_left
│  ├─ rear_right_wheel_joint  ← Nhận τ_rear_right
│  ├─ front_left_wheel_joint  ← Giám sát (feedback)
│  └─ front_right_wheel_joint ← Giám sát (feedback)
│
└─ STEERING JOINTS (Góc lái)
   ├─ front_left_steering_joint  ← Nhận θ_left
   └─ front_right_steering_joint ← Nhận θ_right
```

**Lưu ý:**
- Bánh **sau** nhận torque (dẫn động)
- Bánh **trước** nhận góc lái (steering)
- Bánh **trước** bị kéo theo do chassis chuyển động (passive)

---

### 7. **Camera Sensor**

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="my_camera">
    <camera name="head">
      <horizontal_fov>1.39</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
  </sensor>
</gazebo>
```

**Cài đặt camera:**
- **Độ phân giải:** 800×600 pixels
- **Góc nhìn ngang:** 1.39 rad (79.6°)
- **Khoảng cách:** 0.02m đến 300m

---

## Sơ Đồ Cây Liên Kết (Link Tree)

```
base_link (Frame gốc)
  └── chassis (thân xe, type: fixed)
      ├── rear_left_wheel (bánh sau trái, type: continuous)
      ├── rear_right_wheel (bánh sau phải, type: continuous)
      ├── front_left_steering_link (điều khiển lái trái, type: revolute)
      │   └── front_left_wheel (bánh trước trái, type: continuous)
      ├── front_right_steering_link (điều khiển lái phải, type: revolute)
      │   └── front_right_wheel (bánh trước phải, type: continuous)
      └── camera_link (camera, type: fixed)
```

---

## Hệ Tọa Độ (Coordinate System)

```
      Z (lên)
      |
      |_____ X (về phía trước)
     /
    Y (sang phải)
```

**Ví dụ Origin:**
- `xyz="0 0 0"`: Tại vị trí ban đầu
- `xyz="0.25 0 0.125"`: 0.25m phía trước, 0m ngang, 0.125m cao

**RPY (Roll, Pitch, Yaw):**
- Roll: Quay quanh trục X (lăn)
- Pitch: Quay quanh trục Y (cúi)
- Yaw: Quay quanh trục Z (xoay)

---

## Ví Dụ Thực Tế

### Khối lượng và Quán Tính:

**Thân xe (Chassis):**
- Khối lượng: 10 kg
- Quán tính: $I_{xx} = \frac{10}{12}(0.1^2 + 0.5^2) = 0.208$ kg⋅m²

**Bánh xe (Wheel):**
- Khối lượng: 2 kg mỗi bánh
- Quán tính: $I_{zz} = \frac{2}{2}(0.1^2) = 0.01$ kg⋅m²

---

## Cách Sử Dụng File

1. **Sinh URDF từ Xacro:**
   ```bash
   xacro car.urdf.xacro > car.urdf
   ```

2. **Kiểm tra mô hình:**
   ```bash
   urdf_to_graphiz car.urdf
   ```

3. **Tải vào Gazebo:**
   ```bash
   ros2 launch my_bot gazebo.launch.py
   ```

---

## Tài Liệu Tham Khảo

- [ROS URDF Documentation](http://wiki.ros.org/urdf/)
- [Xacro Documentation](http://wiki.ros.org/xacro/)
- [Gazebo Ackermann Drive Plugin](http://gazebosim.org/)

