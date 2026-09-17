#include <Wire.h>

#define MPU_ADDR 0x68
#define SDA_PIN 8
#define SCL_PIN 9

// Raw 16-bit register values
int16_t rawAccelX, rawAccelY, rawAccelZ;
int16_t rawTemp;
int16_t rawGyroX, rawGyroY, rawGyroZ;

void wakeMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // Set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Native USB CDC wait

  Wire.begin(SDA_PIN, SCL_PIN);
  wakeMPU6050();

  Serial.println("MPU6050 direct I2C communication initialized.");
}

void loop() {
  // 1. Point I2C register pointer to 0x3B (ACCEL_XOUT_H)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false); // Repeated start to keep connection open

  // 2. Request 14 consecutive bytes (6 Accel, 2 Temp, 6 Gyro)
  Wire.requestFrom(MPU_ADDR, 14, true);

  if (Wire.available() == 14) {
    // Read High and Low bytes and combine into 16-bit signed integers
    rawAccelX = (Wire.read() << 8) | Wire.read();
    rawAccelY = (Wire.read() << 8) | Wire.read();
    rawAccelZ = (Wire.read() << 8) | Wire.read();
    
    rawTemp   = (Wire.read() << 8) | Wire.read();
    
    rawGyroX  = (Wire.read() << 8) | Wire.read();
    rawGyroY  = (Wire.read() << 8) | Wire.read();
    rawGyroZ  = (Wire.read() << 8) | Wire.read();

    // 3. Convert raw values to physical units
    float ax = rawAccelX / 16384.0; // Acceleration in g
    float ay = rawAccelY / 16384.0;
    float az = rawAccelZ / 16384.0;


    float gx = rawGyroX / 131.0; // Angular velocity in deg/s
    float gy = rawGyroY / 131.0;
    float gz = rawGyroZ / 131.0;

    // Output formatted readings
    Serial.print(ax); Serial.print(","); Serial.print(ay); Serial.print(","); Serial.print(az);Serial.print(",");
    Serial.print(gx); Serial.print(","); Serial.print(gy); Serial.print(","); Serial.println(gz);
  }

  delay(10);
}