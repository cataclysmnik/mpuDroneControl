/*
 * MPU6050 Sensor Reader - For use with Transmitter ESP
 *
 * This Arduino reads MPU6050 sensor data and sends it via Serial
 * to the PC running the Python dashboard in Transmitter mode.
 *
 * This can run on a separate Arduino connected to MPU6050, or you can
 * connect MPU6050 directly to the transmitter ESP32 and combine this
 * code with esp_transmitter.ino
 *
 * Hardware:
 * - Arduino/ESP32
 * - MPU6050 IMU (I2C: SDA, SCL)
 * - Connected to PC running Python dashboard
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup()
{
    Serial.begin(115200);

    delay(1500); // Allow MPU to boot
    Wire.begin(21, 22);
    Wire.setClock(100000);

    // Initialize MPU6050
    while (!mpu.begin())
    {
        delay(200); // Retry until MPU is ready
    }

    // Configure MPU6050
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("MPU6050 Ready");
}

void loop()
{
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Send as CSV (format expected by Python app in transmitter mode)
    Serial.print(accel.acceleration.x);
    Serial.print(",");
    Serial.print(accel.acceleration.y);
    Serial.print(",");
    Serial.print(accel.acceleration.z);
    Serial.print(",");
    Serial.print(gyro.gyro.x);
    Serial.print(",");
    Serial.print(gyro.gyro.y);
    Serial.print(",");
    Serial.println(gyro.gyro.z);

    delay(10); // ~100Hz update rate
}
