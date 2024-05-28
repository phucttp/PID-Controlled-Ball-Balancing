#include <Servo.h>

//Khai báo biến tọa độ banh
int x_ball, y_ball;

void setup() {
  // Khởi tạo cổng Serial với tốc độ baudrate 115200
  Serial.begin(115200);
}

void loop() {
  if (Serial.available() > 0) { // Kiểm tra xem có dữ liệu nào được gửi từ máy tính không
    String input = Serial.readStringUntil('\n'); // Đọc chuỗi từ Serial cho đến khi gặp ký tự newline (\n)

    // Gọi hàm xử lý dữ liệu đầu vào
    processData(input);
  }
}

void processData(String input) {
  // Kiểm tra sự hiện diện của banh
  if (input == "no") {
    Serial.println("no balls");
    x_ball = -1;
    y_ball = -1;
  }
  else {
    // Tách chuỗi thành phần x và y
    int separatorIndex = input.indexOf('x');
    if (separatorIndex != -1) { // Kiểm tra xem có ký tự phân tách 'x' trong chuỗi không
      String x_str = input.substring(0, separatorIndex); // Lấy phần từ đầu đến 'x'
      String y_str = input.substring(separatorIndex + 1); // Lấy phần sau 'x'

      // Chuyển đổi phần x và y từ String sang int và gọi hàm in ra tọa độ
      x_ball = x_str.toInt();
      y_ball = y_str.toInt();
      
      // Gọi hàm in tọa độ
//        printCoordinates(x_ball, y_ball);
    }
  }
}

void printCoordinates(int x, int y) {
  // In ra tọa độ x và y
  Serial.println("Toa do: ");
  Serial.print(x);
  Serial.print(" - ");
  Serial.print(y);
  Serial.println("");
  delay(500);
}
