# Webots-tehehe


## Moving to multiple point
<details>
  <summary>Code</summary>

```C
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <math.h>
#include <stdio.h>

#define MAX_SPEED 6.67
#define WHEEL_RADIUS 0.033
#define AXLE_LENGTH 0.160

// 1. Định nghĩa cấu trúc điểm lưu (Waypoint)
typedef struct {
  double x;
  double y;
} Waypoint;

int main(int argc, char **argv) {
  wb_robot_init();
  int time_step = (int)wb_robot_get_basic_time_step();

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, time_step);

  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, time_step);

  // 2. Khởi tạo lộ trình (Danh sách các điểm)
  Waypoint path[] = {
    {1.0, 0.0},   // Điểm 1: Đi thẳng lên 1m
    {1.0, 1.0},   // Điểm 2: Rẽ trái tạo thành góc vuông
    {0.0, 1.0},   // Điểm 3: Đi ngang về bên trái
    {0.0, 0.0}    // Điểm 4: Quay về vị trí xuất phát (0,0)
  };
  
  // Tính tổng số điểm trong lộ trình
  int total_waypoints = sizeof(path) / sizeof(path[0]);
  int current_wp = 0; // Bắt đầu từ điểm đầu tiên (index 0)

  double Kp_linear = 1.5;
  double Kp_angular = 4.0;

  while (wb_robot_step(time_step) != -1) {
    const double *pos = wb_gps_get_values(gps);
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);

    if (isnan(pos[0])) continue;

    double current_x = pos[0];
    double current_y = pos[1];
    double current_yaw = rpy[2]; 
    
    // 3. Lấy tọa độ mục tiêu HIỆN TẠI từ mảng lộ trình
    double target_x = path[current_wp].x;
    double target_y = path[current_wp].y;

    double dx = target_x - current_x;
    double dy = target_y - current_y;
    double distance = sqrt(dx * dx + dy * dy);
    
    // 4. Kiểm tra xem đã đến điểm hiện tại chưa
    double tolerance = 0.05; // Bán kính chấp nhận là 5cm
    if (distance < tolerance) {
      current_wp++; // Tăng chỉ số để chuyển sang điểm tiếp theo
      
      // Nếu đã đi hết danh sách điểm
      if (current_wp >= total_waypoints) {
        wb_motor_set_velocity(left_motor, 0.0);
        wb_motor_set_velocity(right_motor, 0.0);
        printf("Đã hoàn thành toàn bộ hành trình!\n");
        break; // Thoát vòng lặp, kết thúc chương trình
      } else {
        printf("Đã đến điểm %d, đang hướng tới điểm %d...\n", current_wp, current_wp + 1);
        continue; // Bỏ qua phần tính toán bên dưới, bắt đầu chu kỳ mới với mục tiêu mới
      }
    }
        
    // --- Phần tính toán P-Controller giữ nguyên ---
    double target_angle = atan2(dy, dx);
    double angle_error = target_angle - current_yaw;
    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
        
    double linear_velocity = Kp_linear * distance;
    if (linear_velocity > 0.22) linear_velocity = 0.22; 
        
    double angular_velocity = Kp_angular * angle_error;
    
    double v_left = (linear_velocity - angular_velocity * AXLE_LENGTH / 2.0) / WHEEL_RADIUS;
    double v_right = (linear_velocity + angular_velocity * AXLE_LENGTH / 2.0) / WHEEL_RADIUS;
    
    if (v_left > MAX_SPEED) v_left = MAX_SPEED;
    if (v_left < -MAX_SPEED) v_left = -MAX_SPEED;
    if (v_right > MAX_SPEED) v_right = MAX_SPEED;
    if (v_right < -MAX_SPEED) v_right = -MAX_SPEED;
    
    wb_motor_set_velocity(left_motor, v_left);
    wb_motor_set_velocity(right_motor, v_right);
  }

  wb_robot_cleanup();
  return 0;
}

```
</details>


## Moving to 1 point
<details>
  <summary>Code</summary>
  
```C
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <math.h>
#include <stdio.h>

// Thông số vật lý của TurtleBot3 Burger
#define MAX_SPEED 6.67
#define WHEEL_RADIUS 0.033
#define AXLE_LENGTH 0.160

int main(int argc, char **argv) {
  // Khởi tạo môi trường Webots
  wb_robot_init();
  int time_step = (int)wb_robot_get_basic_time_step();

  // 1. Cấu hình hệ thống động cơ
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  
  // Chuyển sang chế độ điều khiển vận tốc (Velocity control)
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // 2. Cấu hình cảm biến GPS và IMU
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, time_step);

  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, time_step);

  // 3. Tọa độ mục tiêu mong muốn trên mặt phẳng sàn (X_target, Y_target)
  double target_x = 1.5;
  double target_y = -1.0;

  // 4. Các hệ số tỷ lệ của bộ điều khiển (Gains)
  double Kp_linear = 1.5;
  double Kp_angular = 4.0;

  // Vòng lặp điều khiển chính
  while (wb_robot_step(time_step) != -1) {
    // Đọc giá trị từ cảm biến
    const double *pos = wb_gps_get_values(gps);
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);

    // Bỏ qua khung hình đầu tiên nếu GPS chưa sẵn sàng dữ liệu
    if (isnan(pos[0])) continue;

    double current_x = pos[0];
    double current_y = pos[1];   // pos[1] là trục Y (mặt phẳng sàn)
    double current_yaw = rpy[2]; // Góc quay quanh trục Z (Yaw)
    
    // Tính toán khoảng cách tới mục tiêu
    double dx = target_x - current_x;
    double dy = target_y - current_y;
    double distance = sqrt(dx * dx + dy * dy);
    
    // Kiểm tra nếu đã đến rất gần mục tiêu (sai số < 2cm)
    if (distance < 0.02) {
      wb_motor_set_velocity(left_motor, 0.0);
      wb_motor_set_velocity(right_motor, 0.0);
      printf("Đã đến vị trí mục tiêu thành công!\n");
      break;
    }
        
    // Tính toán góc hướng cần đi (sử dụng hàm atan2 của thư viện math.h)
    double target_angle = atan2(dy, dx);
    
    // Tính sai lệch góc và chuẩn hóa trong khoảng [-PI, PI]
    double angle_error = target_angle - current_yaw;
    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
        
    // Bộ điều khiển tỷ lệ P
    double linear_velocity = Kp_linear * distance;
    if (linear_velocity > 0.22) {
      linear_velocity = 0.22; // Giới hạn tốc độ tịnh tiến
    }
        
    double angular_velocity = Kp_angular * angle_error;
    
    // Nghịch đảo động học: Tính toán vận tốc góc cho từng bánh xe (rad/s)
    double v_left = (linear_velocity - angular_velocity * AXLE_LENGTH / 2.0) / WHEEL_RADIUS;
    double v_right = (linear_velocity + angular_velocity * AXLE_LENGTH / 2.0) / WHEEL_RADIUS;
    
    // Giới hạn vận tốc động cơ không vượt quá giới hạn vật lý
    if (v_left > MAX_SPEED) v_left = MAX_SPEED;
    if (v_left < -MAX_SPEED) v_left = -MAX_SPEED;
    if (v_right > MAX_SPEED) v_right = MAX_SPEED;
    if (v_right < -MAX_SPEED) v_right = -MAX_SPEED;
    
    // Cập nhật tốc độ xuống động cơ
    wb_motor_set_velocity(left_motor, v_left);
    wb_motor_set_velocity(right_motor, v_right);
  }

  // Dọn dẹp bộ nhớ trước khi thoát
  wb_robot_cleanup();
  return 0;
}

```
</details>



