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
#include <MPU6050.h>

MPU6050 mpu;

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    // Initialize MPU6050
    mpu.initialize();

    if (!mpu.testConnection())
    {
        Serial.println("MPU6050 connection failed");
        while (1)
            ;
    }

    // Configure MPU6050
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // ±2g
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // ±250°/s

    Serial.println("MPU6050 Ready");
}

void loop()
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    // Read raw sensor data
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);

    // Convert to physical units
    // Accelerometer: ±2g -> divide by 16384 to get g
    // Gyroscope: ±250°/s -> divide by 131 to get °/s
    float accelX = ax / 16384.0;
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;
    float gyroX = gx / 131.0;
    float gyroY = gy / 131.0;
    float gyroZ = gz / 131.0;

    // Send as CSV (format expected by Python app in transmitter mode)
    Serial.print(accelX, 2);
    Serial.print(",");
    Serial.print(accelY, 2);
    Serial.print(",");
    Serial.print(accelZ, 2);
    Serial.print(",");
    Serial.print(gyroX, 2);
    Serial.print(",");
    Serial.print(gyroY, 2);
    Serial.print(",");
    Serial.println(gyroZ, 2);

    delay(20); // 50Hz update rate
}
