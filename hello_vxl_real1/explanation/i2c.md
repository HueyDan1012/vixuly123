TWI  Module

Cần nắm vững 4 thanh ghi cốt lõi :

Trước hết ta thấy có 4 khu vực : Bus interface unit , bit rate generator ,address match unit, Control Unit .





**1. Thanh ghi TWBR ( TWI Bit rat register)**

Chứa con số từ 0 -255 dùng đ chia tần số từ CPU ra tần số SCL





**2. Thanh ghi TWCR (TWI Control Register )**

Đóng vai trò như bộ chỉ huy - bật tắt các bit để điều khiển I2C

Bit 7 - **TWINT** (TWI Interrupt Flag): "Cờ báo hiệu"



Khi Hardware làm xong việc: Nó tự động bật bit này lên 1 (Set).



Khi bạn muốn Hardware làm việc tiếp: Bạn phải ghi số 1 vào bit này để xóa nó (Clear). Đây là cơ chế hơi ngược đời của AVR: Ghi 1 để xóa.



Trong code: TWCR = (1 << TWINT) | ... nghĩa là "Xóa cờ đi và bắt đầu làm việc mới ngay".



Bit 6 - TWEA (Enable Acknowledge): "Bật trả lời"



Nếu bật bit này: Khi nhận được 1 byte, chip sẽ gửi lại tín hiệu ACK (tôi đã nhận, gửi tiếp đi).



Nếu tắt bit này: Chip sẽ gửi NACK (tôi đã nhận, đừng gửi nữa/đây là byte cuối).



Bit 5 - TWSTA (Start Condition): "Lệnh Start"



Khi bật bit này, chip sẽ chiếm đường truyền và gửi tín hiệu START.



Bit 4 - TWSTO (Stop Condition): "Lệnh Stop"



Khi bật bit này, chip sẽ gửi tín hiệu STOP và giải phóng đường truyền.



Bit 2 - TWEN (TWI Enable): "Cầu dao tổng"



Phải luôn bật bit này (bằng 1) thì module I2C mới hoạt động.



**3. TWDR ( TWI Data Register )**

Chứa dữ lieu

Khi gửi , Data sẽ nhảy vào TWDR rồi kích TWCR  Phần cứng s dịch từng bit trong TWDR đẩy sang TWCR

Khi nhận, Sau khi nhận xong ( Cờ TWINT bật ) , dữ liệu sẽ nằm gọn trong TWDR



**4. TWSR ( TWI Status Register )**

Thanh ghi cho bt trạng thái cuả đường truyền

5 bit đầu ***(TWS3...TWS7)***: Chứa mã trạng thái (Status Code).



Ví dụ: Sau khi gửi START, nếu thành công, TWSR sẽ chứa giá trị 0x08. Nếu mất kết nối, nó sẽ ra mã khác.



Trong code driver đơn giản: Chúng ta thường lờ đi việc check lỗi chi tiết để code gọn, nhưng trong driver chuyên nghiệp, người ta sẽ switch(TWSR \& 0xF8) để xem lỗi gì.



2 bit cuối ***(TWPS0, TWPS1)***: Prescaler (Bộ chia tần số).



Dùng kết hợp với TWBR để chỉnh tốc độ.



Trong code: TWSR = 0x00; nghĩa là chúng ta chọn bộ chia là 1 (không chia nhỏ xung nhịp).

