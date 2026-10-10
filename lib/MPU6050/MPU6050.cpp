#include "MPU6050.h"
#include "BLEManager.h"
#include <math.h>

// Các địa chỉ thanh ghi phần cứng MPU6050
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

MPU6050::MPU6050(uint8_t addr) {
  _addr = addr;
  _gyroZOffset = 0.0;
  _roll = 0.0;
  _pitch = 0.0;
  _yaw = 0.0;
  _yawSetpoint = 0.0;
  _lastTime = 0;
  _stillTime = 0.0;
  _smoothZRate = 0.0;
  _lastZRate = 0.0;
}

bool MPU6050::begin() {
  // 1. Kiểm tra thiết bị phản hồi
  Wire.beginTransmission(_addr);
  if (Wire.endTransmission() != 0) {
    return false; // Không tìm thấy MPU6050 ở địa chỉ _addr
  }

  // 2. Đánh thức MPU6050 (Power Management 1 = 0, sử dụng xung Clock dao động
  // con quay trục X để chống trôi nhiệt)
  Wire.beginTransmission(_addr);
  Wire.write(MPU6050_REG_PWR_MGMT_1);
  Wire.write(0x01); // 0x01: PLL with X Gyro reference (ổn định nhiệt hơn nhiều
                    // so với internal 8MHz oscillator 0x00)
  if (Wire.endTransmission() != 0) {
    return false;
  }
  delay(15); // Đợi PLL khoá dao động ổn định

  // 3. Cấu hình DLPF (Digital Low Pass Filter) thanh ghi 0x1A:
  // Cấu hình mức 3: Băng thông Gyro 42Hz, trễ 4.9ms.
  // Lọc sạch toàn bộ rung động cơ học tần số cao từ động cơ và mặt sàn, giảm
  // 90% nhiễu trôi!
  Wire.beginTransmission(_addr);
  Wire.write(0x1A);
  Wire.write(0x03);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  // 4. Cấu hình tần số lấy mẫu (Sample Rate Divider thanh ghi 0x19):
  // Sample Rate = 1kHz / (1 + 4) = 200Hz (chu kỳ 5ms một mẫu)
  Wire.beginTransmission(_addr);
  Wire.write(0x19);
  Wire.write(0x04);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  // 5. Cấu hình dải đo Gyro: +/- 2000 deg/s (Hệ số nhạy: 16.4 LSB/(deg/s))
  Wire.beginTransmission(_addr);
  Wire.write(MPU6050_REG_GYRO_CONFIG);
  Wire.write(0x18);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  // 6. Cấu hình dải đo Accel: +/- 8g
  Wire.beginTransmission(_addr);
  Wire.write(MPU6050_REG_ACCEL_CONFIG);
  Wire.write(0x10);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  _lastTime = micros();
  return true;
}

bool MPU6050::readRawData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx,
                          int16_t *gy, int16_t *gz) {
  Wire.beginTransmission(_addr);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) {
    return false; // I2C NACK hoặc bus bận, tránh gọi requestFrom gây Error 259
  }

  uint8_t bytesReceived = Wire.requestFrom(_addr, (uint8_t)14);
  if (bytesReceived != 14) {
    return false; // Không nhận đủ byte dữ liệu
  }

  *ax = (int16_t)((Wire.read() << 8) | Wire.read());
  *ay = (int16_t)((Wire.read() << 8) | Wire.read());
  *az = (int16_t)((Wire.read() << 8) | Wire.read());
  Wire.read();
  Wire.read(); // Bỏ qua giá trị nhiệt độ (Temperature)
  *gx = (int16_t)((Wire.read() << 8) | Wire.read());
  *gy = (int16_t)((Wire.read() << 8) | Wire.read());
  *gz = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

void MPU6050::calibrate(int samples) {
  delay(100); // Chờ ổn định điện áp và phần cứng
  float sumZ = 0;
  int validSamples = 0;
  int16_t ax, ay, az, gx, gy, gz;

  // Bỏ 50 mẫu đầu cho bộ lọc phần cứng DLPF ổn định hoàn toàn
  for (int i = 0; i < 50; i++) {
    readRawData(&ax, &ay, &az, &gx, &gy, &gz);
    delay(3);
  }

  // Lấy mẫu trung bình tĩnh với tối thiểu 400 mẫu để tính offset trục Z cực
  // chuẩn
  int targetSamples = samples < 400 ? 400 : samples;
  for (int i = 0; i < targetSamples; i++) {
    if (readRawData(&ax, &ay, &az, &gx, &gy, &gz)) {
      sumZ += gz;
      validSamples++;
    }
    delay(3);
  }

  // Tính toán độ lệch tĩnh (offset) cho trục Z với dải 2000 deg/s (16.4
  // LSB/(deg/s))
  if (validSamples > 0) {
    _gyroZOffset = (sumZ / (float)validSamples) / 16.4f;
  }
  _stillTime = 0.0;
  _smoothZRate = 0.0f;
  _lastZRate = 0.0f;

  String calMsg = ">> GYRO CALIB: " + String(validSamples) + "/" +
                  String(targetSamples) +
                  " mau hop le | offset Z = " + String(_gyroZOffset, 3) +
                  " do/s (DLPF 42Hz)";
  Serial.println(calMsg);
  if (bleManager.isConnected()) {
    bleManager.println(calMsg);
  }
  _roll = 0.0;
  _pitch = 0.0;
  _yaw = 0.0;
  _yawSetpoint = 0.0;
  _lastTime = micros();
}

void MPU6050::update() {
  int16_t ax, ay, az, gx, gy, gz;
  if (!readRawData(&ax, &ay, &az, &gx, &gy, &gz)) {
    return; // Đọc I2C thất bại -> bỏ qua vòng này, không tính toán sai góc
  }

  unsigned long now = micros();
  float dt = (now - _lastTime) / 1000000.0f; // Đổi từ micro giây sang giây
  if (_lastTime == 0 || dt > 0.1f || dt <= 0.0f) {
    dt = 0.01f;
  }
  _lastTime = now;

  // Góc nghiêng lấy từ gia tốc kế, tính theo độ.
  _roll = atan2((float)ay, (float)az) * 180.0f / PI;
  _pitch =
      atan2(-(float)ax, sqrt((float)ay * ay + (float)az * az)) * 180.0f / PI;

  // Đổi giá trị thô sang đơn vị độ/giây (deg/s) với hệ số 16.4 (cho dải 2000
  // deg/s)
  float rawRate =
      (gz / 16.4f) - _gyroZOffset; // độ/giây sau khi trừ offset lúc calib

  // Bộ lọc thông thấp EMA làm mịn vận tốc góc (khử nhiễu giật spike)
  _smoothZRate = (0.75f * _smoothZRate) + (0.25f * rawRate);

  // Nhận diện trạng thái đứng yên thực sự:
  // Khi xe đứng yên, cả vận tốc góc sau lọc và độ biến thiên đều nhỏ (< 0.75
  // deg/s)
  float deltaRate = fabsf(rawRate - _lastZRate);
  _lastZRate = rawRate;

  // Still detection: dùng rawRate (KHÔNG dùng smoothZRate vì EMA có thể
  // trung bình hóa dao động PID về 0 và kích hoạt nhầm)
  // Chỉ khoá khi raw gyro THỰC SỰ yên tĩnh (< 0.3 deg/s liên tục > 1.5s)
  if (fabsf(rawRate) < 0.3f && deltaRate < 0.20f) {
    _stillTime += dt;
    if (_stillTime > 1.5f) {
      // Bù drift nhiệt khi đứng yên lâu
      _gyroZOffset += _smoothZRate * 0.002f;
      // Khoá hoàn toàn chỉ khi chắc chắn đứng yên
      _smoothZRate = 0.0f;
    }
  } else {
    _stillTime = 0.0f;
  }

  // Deadband rất nhỏ: chỉ loại nhiễu điện tử nền (~0.05 deg/s)
  if (fabsf(_smoothZRate) < 0.05f) {
    _smoothZRate = 0.0f;
  }

  // Chiều quay chuẩn quanh trục Z (quay trái -> Yaw dương, quay phải -> Yaw âm)
  float gyroZRate = _smoothZRate;

  // Tích phân vận tốc góc theo thời gian để ra góc Yaw
  _yaw += gyroZRate * dt;
}

bool MPU6050::rotateToAngle(float targetAngle, uint8_t m1In1, uint8_t m1In2,
                            uint8_t m2In1, uint8_t m2In2, float compensation,
                            uint8_t turnSpeed, unsigned long timeoutMs) {
  if (targetAngle == 0.0f) {
    return true;
  }

  update();
  float startAngle = _yaw;
  float requestedAngle = fabs(targetAngle);
  float effectiveAngle =
      requestedAngle > compensation ? requestedAngle - compensation : 0.0f;

  String startMsg = ">> BAT DAU QUAY: Goc hien tai = " + String(startAngle, 2) +
                    " | Can quay = " + String(effectiveAngle, 2) +
                    " do | Toc do PWM = " + String(turnSpeed);
  Serial.println(startMsg);
  if (bleManager.isConnected()) {
    bleManager.println(startMsg);
    String jsonStart =
        "{\"type\":\"turn_start\",\"start\":" + String(startAngle, 2) +
        ",\"effective\":" + String(effectiveAngle, 2) +
        ",\"spd\":" + String(turnSpeed) + "}";
    bleManager.println(jsonStart);
  }

  // Điều khiển động cơ quay theo tốc độ PWM:
  // targetAngle > 0: Quay trái (Motor trái lùi, Motor phải tiến)
  // targetAngle < 0: Quay phải (Motor trái tiến, Motor phải lùi)
  unsigned long turnStart = millis();
  bool reachedTarget = false;

  // Giảm tốc tiếp cận (Proportional Deceleration)
  // Trong 35 độ cuối cùng, tốc độ PWM giảm dần từ turnSpeed về MIN_TURN_SPEED
  constexpr uint8_t MIN_TURN_SPEED = 42;
  constexpr float DECEL_ZONE = 35.0f;

  while (millis() - turnStart <= timeoutMs) {
    update();
    float turnedAngle = fabs(_yaw - startAngle);
    float remainingAngle = effectiveAngle - turnedAngle;

    if (turnedAngle >= effectiveAngle || remainingAngle <= 0.0f) {
      reachedTarget = true;
      break;
    }

    // Tính tốc độ giảm dần theo góc còn lại
    uint8_t currentSpeed = turnSpeed;
    if (remainingAngle < DECEL_ZONE && turnSpeed > MIN_TURN_SPEED) {
      float ratio = remainingAngle / DECEL_ZONE; // 1.0 -> 0.0
      currentSpeed =
          MIN_TURN_SPEED + (uint8_t)((turnSpeed - MIN_TURN_SPEED) * ratio);
    }

    if (targetAngle > 0.0f) {
      // Quay trái: Motor trái lùi, Motor phải tiến
      analogWrite(m1In1, 0);
      analogWrite(m1In2, currentSpeed);
      analogWrite(m2In1, currentSpeed);
      analogWrite(m2In2, 0);
    } else {
      // Quay phải: Motor trái tiến, Motor phải lùi
      analogWrite(m1In1, currentSpeed);
      analogWrite(m1In2, 0);
      analogWrite(m2In1, 0);
      analogWrite(m2In2, currentSpeed);
    }

    delay(2);
  }

  // 1. Phanh ngắn mạch (Active Braking 100% PWM) trong 70ms để triệt tiêu hoàn
  // toàn quán tính
  analogWrite(m1In1, 255);
  analogWrite(m1In2, 255);
  analogWrite(m2In1, 255);
  analogWrite(m2In2, 255);
  delay(70);

  // 2. Nhả motor về trạng thái thả tự do (Tắt dứt điểm)
  analogWrite(m1In1, 0);
  analogWrite(m1In2, 0);
  analogWrite(m2In1, 0);
  analogWrite(m2In2, 0);
  digitalWrite(m1In1, LOW);
  digitalWrite(m1In2, LOW);
  digitalWrite(m2In1, LOW);
  digitalWrite(m2In2, LOW);

  update();
  float turnedTotal = fabs(_yaw - startAngle);

  // Reset góc yaw về 0 sau khi quay xong
  resetYaw();

  String resMsg = ">> KET QUA: Goc da quay duoc = " + String(turnedTotal, 2) +
                  " do | Goc hien tai (da reset) = " + String(_yaw, 2) +
                  (reachedTarget ? " (THANH CONG)"
                                 : " (TIMEOUT - KET BANH HOAC LOI DOC GYRO)");
  Serial.println(resMsg);
  if (bleManager.isConnected()) {
    bleManager.println(resMsg);
    String jsonRes =
        "{\"type\":\"turn_res\",\"target\":" + String(fabs(targetAngle), 1) +
        ",\"turned\":" + String(turnedTotal, 2) +
        ",\"yaw\":" + String(_yaw, 2) +
        ",\"success\":" + String(reachedTarget ? "true" : "false") + "}";
    bleManager.println(jsonRes);
  }

  return reachedTarget;
}

float MPU6050::getRoll() const { return _roll; }

float MPU6050::getPitch() const { return _pitch; }

float MPU6050::getYaw() const { return _yaw; }

float MPU6050::getRelativeYaw() const { return _yaw - _yawSetpoint; }

void MPU6050::setYawSetpoint() { _yawSetpoint = _yaw; }

void MPU6050::resetYaw() {
  _yaw = 0.0f;
  _yawSetpoint = 0.0f;
  _smoothZRate = 0.0f;
  _lastZRate = 0.0f;
  _stillTime = 0.0f;
}