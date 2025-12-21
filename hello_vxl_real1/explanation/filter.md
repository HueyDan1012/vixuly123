# Tài liệu Kỹ thuật: MPU6050 Driver & Digital Filters

## 1. Tổng quan
Dự án sử dụng vi điều khiển ATmega328P (8-bit) để điều khiển xe tự cân bằng.
Để đảm bảo hiệu năng thời gian thực (Real-time), toàn bộ code tính toán sử dụng **Số học điểm tĩnh (Fixed-Point Arithmetic)**, loại bỏ hoàn toàn số thực (`float`).

Hệ thống cung cấp 2 lựa chọn bộ lọc:
1.  **Complementary Filter:** Đơn giản, tính toán cực nhanh.
2.  **Kalman Filter:** Phức tạp hơn, khử nhiễu tốt hơn và tự động bù trôi (bias).

---

## 2. Driver MPU6050 & Thanh ghi quan trọng
Driver giao tiếp qua I2C (400kHz). Dưới đây là các thanh ghi được cấu hình trong `mpu6050.c`:

| Thanh ghi | Địa chỉ | Giá trị | Giải thích |
| :--- | :--- | :--- | :--- |
| `PWR_MGMT_1` | 0x6B | `0x00` | Đánh thức cảm biến (Mặc định là Sleep). |
| `SMPLRT_DIV` | 0x19 | `0x07` | Sample Rate Divider. Tạo mẫu dữ liệu ở tần số 1kHz. |
| `CONFIG` | 0x1A | `0x00` | Tắt bộ lọc thông thấp phần cứng (DLPF) để lấy dữ liệu thô nhanh nhất. |
| `GYRO_CONFIG` | 0x1B | `0x00` | Full Scale Range: **±250°/s**. Độ nhạy: 131 LSB/°/s. |
| `ACCEL_CONFIG` | 0x1C | `0x00` | Full Scale Range: **±2g**. Độ nhạy: 16384 LSB/g. |

---

## 3. Lý thuyết Bộ lọc (Filter Theory)

### Bài toán
* **Accelerometer (Gia tốc kế):** Đo góc nghiêng bằng trọng lực.
    * *Ưu điểm:* Không bị trôi theo thời gian.
    * *Nhược điểm:* Rất nhiễu khi xe rung lắc hoặc di chuyển (do lực quán tính).
* **Gyroscope (Con quay hồi chuyển):** Đo vận tốc góc.
    * *Ưu điểm:* Rất mượt, phản hồi nhanh tức thời.
    * *Nhược điểm:* Bị trôi (drift) tích tụ theo thời gian (tích phân sai số).

-> **Mục tiêu:** Kết hợp sự ổn định lâu dài của Accel và độ mượt ngắn hạn của Gyro.

### 3.1. Complementary Filter (Bộ lọc bù)
Đây là phương pháp "trọng số".
$$Angle = 0.98 \times (Gyro) + 0.02 \times (Accel)$$

* **Logic Code:** `(98 * GyroTerm + 2 * AccelAngle) / 100`
* **Đặc điểm:** Hoạt động như một High-pass filter cho Gyro và Low-pass filter cho Accel.
* **Ưu điểm:** Tốn cực ít CPU, chỉ vài phép nhân cộng.
* **Nhược điểm:** Cần chỉnh tay hệ số (0.98) cho phù hợp với độ rung của xe.

### 3.2. Kalman Filter (Bộ lọc Kalman)
Đây là phương pháp "dự đoán và sửa lỗi" dựa trên thống kê.



* **Cơ chế:**
    1.  **Predict:** Dùng Gyro để đoán vị trí tiếp theo.
    2.  **Update:** Dùng Accel để so sánh với vị trí vừa đoán.
    3.  **Kalman Gain (K):** Tính toán xem nên tin Gyro hay Accel hơn tại thời điểm đó (dựa trên ma trận hiệp phương sai P).
    4.  **Bias Correction:** Tự động ước lượng sai số tĩnh của Gyro và trừ dần đi.

* **Fixed-Point Implementation:**
    * Sử dụng hệ số Q16.12 (1.0 = 4096).
    * Thay thế phép chia bằng phép dịch bit (`>> 12`) để tăng tốc độ gấp 10-20 lần so với dùng `float`.

---

## 4. Hướng dẫn tích hợp vào Main

Trong `main.c`, bạn nên sử dụng cấu trúc như sau:

```c
#include "mpu6050.h"
#include "filter.h"

// Chọn bộ lọc muốn dùng
Kalman_t robotFilter; 
// Complementary_t robotFilter; // Hoặc dùng cái này

int main(void) {
    MPU6050_Init();
    
    // Khởi tạo bộ lọc
    Kalman_Init(&robotFilter);
    // Compl_Init(&robotFilter);

    // Setup Timer Interrupt 5ms (200Hz)
    // ...

    while(1) {
        // Main loop...
    }
}

// Trong ngắt Timer (Mỗi 5ms)
ISR(TIMER1_COMPA_vect) {
    // 1. Đọc Sensor
    MPU6050_ReadData((MPU6050_Data*)&mpu_data);

    // 2. Tính góc (trả về milli-degrees, vd: 4500)
    int32_t angle = Kalman_Update(&robotFilter, (MPU6050_Data*)&mpu_data, 5);
    
    // 3. Gọi hàm PID với giá trị angle này
    PID_Compute(angle);
}