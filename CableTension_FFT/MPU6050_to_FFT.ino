#include<Wire.h>

const uint16_t MPU_ADDR = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int16_t AcX_Out, AcY_Out, AcZ_Out;
uint32_t freq = 100000;
bool requestFrom_flag = true;
uint8_t requestFrom_size = 14;
String str;

void setup() {
  Serial.begin(250000);
  //Serial.begin(115200);
  Wire.begin(21, 22, freq); // sda, scl
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  //Serial.println("Setup complete");
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  //AcX_Out = 0;
  AcX_Out = map(AcX, -16384, 16384, 0, 4095);
  AcY_Out = map(AcY, -16384, 16384, 0, 4095);
  AcZ_Out = map(AcZ, -16384, 16384, 0, 4095);
  
  str = String(AcX_Out) + "*" + String(AcY_Out) + "*" + String(AcZ_Out) + "*";
  Serial.println(str);
}
