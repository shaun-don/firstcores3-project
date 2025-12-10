#include <M5Unified.h>

// Complementary filter constants
const float ALPHA = 0.98;  // Weight for gyroscope (0.98 = 98% gyro, 2% accel)
const float DT = 0.01;     // Time step in seconds (10ms)

// Filtered orientation values
float pitch = 0.0, roll = 0.0, yaw = 0.0;

// Magnetometer calibration values
float mag_offset_x = 0.0, mag_offset_y = 0.0, mag_offset_z = 0.0;
float mag_scale_x = 1.0, mag_scale_y = 1.0, mag_scale_z = 1.0;
bool calibrated = false;

// Calibration min/max values
float mag_min_x = 10000, mag_min_y = 10000, mag_min_z = 10000;
float mag_max_x = -10000, mag_max_y = -10000, mag_max_z = -10000;

void calibrateMagnetometer() {
  M5.Display.clear();
  M5.Display.setCursor(10, 60);
  M5.Display.setTextColor(YELLOW);
  M5.Display.println("Magnetometer Calibration");
  M5.Display.setCursor(10, 100);
  M5.Display.println("Rotate device in");
  M5.Display.setCursor(10, 130);
  M5.Display.println("figure-8 pattern");
  M5.Display.setCursor(10, 160);
  M5.Display.println("Press button when done");
  
  unsigned long startTime = millis();
  int samples = 0;
  
  while (millis() - startTime < 20000) {  // 20 second calibration window
    M5.update();
    
    if (M5.BtnA.wasPressed()) {
      break;  // Exit calibration early if button pressed
    }
    
    auto imu_update = M5.Imu.update();
    if (imu_update) {
      auto data = M5.Imu.getImuData();
      
      // Track min/max values
      if (data.mag.x < mag_min_x) mag_min_x = data.mag.x;
      if (data.mag.x > mag_max_x) mag_max_x = data.mag.x;
      if (data.mag.y < mag_min_y) mag_min_y = data.mag.y;
      if (data.mag.y > mag_max_y) mag_max_y = data.mag.y;
      if (data.mag.z < mag_min_z) mag_min_z = data.mag.z;
      if (data.mag.z > mag_max_z) mag_max_z = data.mag.z;
      
      samples++;
      
      // Show progress
      M5.Display.fillRect(10, 200, 300, 30, BLACK);
      M5.Display.setCursor(10, 200);
      M5.Display.printf("Samples: %d", samples);
    }
    
    delay(10);
  }
  
  // Calculate offsets and scale factors
  mag_offset_x = (mag_max_x + mag_min_x) / 2.0;
  mag_offset_y = (mag_max_y + mag_min_y) / 2.0;
  mag_offset_z = (mag_max_z + mag_min_z) / 2.0;
  
  float mag_range_x = mag_max_x - mag_min_x;
  float mag_range_y = mag_max_y - mag_min_y;
  float mag_range_z = mag_max_z - mag_min_z;
  
  float avg_range = (mag_range_x + mag_range_y + mag_range_z) / 3.0;
  
  mag_scale_x = avg_range / mag_range_x;
  mag_scale_y = avg_range / mag_range_y;
  mag_scale_z = avg_range / mag_range_z;
  
  calibrated = true;
  
  M5.Display.clear();
  M5.Display.setTextColor(GREEN);
  M5.Display.setCursor(10, 100);
  M5.Display.println("Calibration Complete!");
  delay(2000);
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(WHITE);
  
  // Initialize IMU
  M5.Imu.begin();
  
  M5.Display.println("IMU Initialized");
  M5.Display.println("");
  M5.Display.println("Press Btn A to");
  M5.Display.println("calibrate compass");
  M5.Display.println("");
  M5.Display.println("Or wait 3 sec to skip");
  
  unsigned long startWait = millis();
  while (millis() - startWait < 3000) {
    M5.update();
    if (M5.BtnA.wasPressed()) {
      calibrateMagnetometer();
      break;
    }
    delay(10);
  }
  
  M5.Display.clear();
}

void loop() {
  M5.update();
  
  // Check if button pressed to recalibrate
  if (M5.BtnA.wasPressed()) {
    calibrateMagnetometer();
  }
  
  static unsigned long lastTime = millis();
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  auto imu_update = M5.Imu.update();
  
  if (imu_update) {
    auto data = M5.Imu.getImuData();
    
    // Calculate pitch and roll from accelerometer (in degrees)
    float accel_pitch = atan2(data.accel.y, sqrt(data.accel.x * data.accel.x + data.accel.z * data.accel.z)) * 180.0 / PI;
    float accel_roll = atan2(-data.accel.x, data.accel.z) * 180.0 / PI;
    
    // Integrate gyroscope data (convert from deg/s to degrees)
    pitch = ALPHA * (pitch + data.gyro.x * dt) + (1.0 - ALPHA) * accel_pitch;
    roll = ALPHA * (roll + data.gyro.y * dt) + (1.0 - ALPHA) * accel_roll;
    
    // Apply magnetometer calibration
    float mag_x = (data.mag.x - mag_offset_x) * mag_scale_x;
    float mag_y = (data.mag.y - mag_offset_y) * mag_scale_y;
    float mag_z = (data.mag.z - mag_offset_z) * mag_scale_z;
    
    // Tilt compensation for heading
    float pitch_rad = pitch * PI / 180.0;
    float roll_rad = roll * PI / 180.0;
    
    float mag_x_comp = mag_x * cos(pitch_rad) + mag_z * sin(pitch_rad);
    float mag_y_comp = mag_x * sin(roll_rad) * sin(pitch_rad) + 
                       mag_y * cos(roll_rad) - 
                       mag_z * sin(roll_rad) * cos(pitch_rad);
    
    // Calculate heading (yaw) with tilt compensation
    yaw = atan2(mag_y_comp, mag_x_comp) * 180.0 / PI;
    
    // Normalize heading to 0-360 degrees
    if (yaw < 0) {
      yaw += 360.0;
    }
    
    // Display on screen
    M5.Display.clear();
    
    // Show calibration status
    M5.Display.setCursor(10, 10);
    if (calibrated) {
      M5.Display.setTextColor(GREEN);
      M5.Display.print("CAL: OK");
    } else {
      M5.Display.setTextColor(YELLOW);
      M5.Display.print("CAL: NONE");
    }
    
    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(10, 40);
    M5.Display.printf("Heading: %.1f deg", yaw);
    
    M5.Display.setCursor(10, 80);
    M5.Display.printf("Pitch:   %.1f deg", pitch);
    
    M5.Display.setCursor(10, 120);
    M5.Display.printf("Roll:    %.1f deg", roll);
    
    // Visual compass indicator
    int centerX = 270;
    int centerY = 80;
    int radius = 40;
    
    // Draw compass circle
    M5.Display.drawCircle(centerX, centerY, radius, WHITE);
    M5.Display.drawCircle(centerX, centerY, radius-1, WHITE);
    
    // Draw N marker
    M5.Display.setTextSize(1);
    M5.Display.setCursor(centerX - 4, centerY - radius - 15);
    M5.Display.print("N");
    
    // Draw heading line
    float heading_rad = yaw * PI / 180.0;
    int lineX = centerX + radius * 0.8 * sin(heading_rad);
    int lineY = centerY - radius * 0.8 * cos(heading_rad);
    M5.Display.drawLine(centerX, centerY, lineX, lineY, RED);
    M5.Display.fillCircle(lineX, lineY, 3, RED);
    
    // Instructions
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 210);
    M5.Display.print("Press Btn A to (re)calibrate");
  }
  
  delay(10);
}