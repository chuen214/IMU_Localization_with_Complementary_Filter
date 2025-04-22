#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <QMC5883LCompass.h>

LiquidCrystal_PCF8574 lcd(0x27);
Adafruit_MPU6050 mpu;
QMC5883LCompass compass;

float yaw = 0.0f;  // Yaw 角度 (累積旋轉角度)
float yaw_mag = 0.0f; // 磁力計 Yaw 角度
float yaw_fused = 0.0f; // 互補濾波後的 Yaw 角度
const float alpha = 0.98f; // 互補濾波權重

unsigned long prevTime = 0;

float vY = 0, vZ = 0;
float sY = 0, sZ = 0;

// **誤差修正變數**
float biasAccY = 0.0f, biasAccZ = 0.0f;
float biasGyroZ = 0.0f;
float yaw_offset = 0.0f; // 磁力計初始角度偏移

void calibrateSensors() {
  Serial.println("Calibrating sensors... Keep MPU6050 still for 10 seconds.");
  lcd.setCursor(0, 0);
  lcd.print("Calibrating");
  lcd.setCursor(0, 1);
  lcd.print("sensors...");

  float sumAccY = 0, sumAccZ = 0;
  float sumGyroZ = 0;
  int count = 0;

  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sumAccY += a.acceleration.y;
    sumAccZ += a.acceleration.z;
    sumGyroZ += g.gyro.x;

    count++;
    delay(10);
  }

  biasAccY = sumAccY / count;
  biasAccZ = sumAccZ / count;  // **去除重力**
  biasGyroZ = sumGyroZ / count;
  Serial.println("Calibration complete.");
}

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.setBacklight(255);
  lcd.clear();

  if (!mpu.begin()) {
    Serial.println("無法找到 MPU6050 傳感器！");
    while (1) { delay(10); }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  
  delay(100);
  calibrateSensors();

  // 讀取磁力計初始角度作為偏移
  compass.read();
  yaw_offset = atan2(compass.getY(), compass.getX()) * 57.2958;
  if (yaw_offset < 0) yaw_offset += 360.0;

  prevTime = micros();
}

void loop() {
  unsigned long currTime = micros();
  float dt = (currTime - prevTime) / 1000000.0;
  prevTime = currTime;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  compass.read();

  // **消除誤差**
  float accY = a.acceleration.y - biasAccY;
  float accZ = a.acceleration.z - biasAccZ;
  float gyroZ = g.gyro.x - biasGyroZ;
  float gyroZ_deg = gyroZ * 180.0 / PI;

  // **積分計算 Yaw 旋轉角度**
  yaw += gyroZ_deg * dt;
  if (yaw > 180) yaw -= 360;
  else if (yaw < -180) yaw += 360;

  // **磁力計計算 Yaw 角度**
  yaw_mag = atan2(compass.getY(), compass.getX()) * 57.2958 - yaw_offset;
  if (yaw_mag < 0) yaw_mag += 360.0;

  // **互補濾波融合兩種測量方式**
  yaw_fused = alpha * (yaw_fused + gyroZ_deg * dt) + (1 - alpha) * yaw_mag;
  if (yaw_fused > 180) yaw_fused -= 360;
  else if (yaw_fused < -180) yaw_fused += 360;

  // **計算速度和位移**
  if (fabs(accY) >= 0.05) {
    vY += accY * dt;
    if (fabs(accY) < 0.1) vY *= 0.99;  // 減小漂移
    sY += vY * dt;
  }
  
  if (fabs(accZ) >= 0.05) {
    vZ += accZ * dt;
    if (fabs(accZ) < 0.1) vZ *= 0.99;
    sZ += vZ * dt;
  }

  Serial.print("Yaw_fused=");
  Serial.print(yaw_fused, 2);
  Serial.print(" deg, AccY=");
  Serial.print(accY, 3);
  Serial.print(" m/s², AccZ=");
  Serial.print(accZ, 3);
  Serial.print(" m/s², vY=");
  Serial.print(vY, 3);
  Serial.print(" m/s, vZ=");
  Serial.print(vZ, 3);
  Serial.print(" m/s, sY=");
  Serial.print(sY, 3);
  Serial.print(" m, sZ=");
  Serial.print(sZ, 3);
  Serial.println(" m");

  // **LCD 顯示**
  lcd.setCursor(0, 0);
  lcd.print("Yaw: ");
  lcd.print(yaw_fused, 2);
  lcd.print(" deg  ");

  lcd.setCursor(0, 1);
  lcd.print("Y:");
  lcd.print(sY, 2);
  lcd.print("m ");
  lcd.print("Z:");
  lcd.print(sZ, 2);
  lcd.print("m ");

  delay(50);
}
