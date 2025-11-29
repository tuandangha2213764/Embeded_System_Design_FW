import serial
import time
import math
import random

# --- CẤU HÌNH ---
# Lưu ý: Nếu C# bạn chọn COM2, thì ở đây phải là COM1 (cổng phía bên kia của cầu nối)
PORT = 'COM1'  
BAUDRATE = 115200

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print(f"Da ket noi vao {PORT}. Dang gui du lieu gia lap...")
except Exception as e:
    print(f"Loi mo cong {PORT}: {e}")
    print("Hay dam bao ban da tao cap COM ao (VD: COM1 <-> COM2)")
    exit()

# Biến mô phỏng
t = 0
actual_speed = 0
setpoint = 0

try:
    while True:
        # 1. Tạo Setpoint giả (Hình sin để test đồ thị lượn sóng)
        # Thay đổi giá trị từ -200 đến 200
        setpoint = 200 * math.sin(t * 0.1)

        # 2. Mô phỏng đáp ứng của Motor (Actual Speed)
        # Motor thật sẽ có độ trễ (quán tính), không bám ngay lập tức
        # Công thức lọc thông thấp đơn giản: Speed mới = 90% Speed cũ + 10% Setpoint
        # Cộng thêm chút nhiễu (random) cho giống thật
        err = setpoint - actual_speed
        actual_speed = actual_speed + (err * 0.1) + random.uniform(-2, 2)

        # 3. Giả lập tín hiệu điều khiển (Control Signal)
        # Tỉ lệ thuận với sai số, giới hạn 0-100
        control_signal = abs(err) * 0.5
        if control_signal > 100: control_signal = 100

        # 4. Đóng gói dữ liệu đúng format C# yêu cầu
        # Format: "Setpoint, ActualSpeed (Absolute), Control"
        # Lưu ý: Code C# của bạn đang dùng Math.Abs cho Speed nhận về, 
        # nên ở đây ta gửi trị tuyệt đối, C# sẽ tự xử lý dấu dựa theo Setpoint.
        data_to_send = f"{setpoint:.2f},{abs(actual_speed):.2f},{control_signal:.2f}\r\n"

        # 5. Gửi xuống Serial
        ser.write(data_to_send.encode('utf-8'))
        
        # In ra màn hình console để kiểm tra
        print(f"Sent: {data_to_send.strip()}")

        # 6. Tăng biến thời gian và delay
        t += 1
        time.sleep(0.1) # Gửi 100ms/lần (tương đương 10Hz)

except KeyboardInterrupt:
    print("\nDa dung chuong trinh.")
    ser.close()