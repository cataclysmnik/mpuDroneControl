/*
 * Combined MPU6050 + ESP-NOW Transmitter
 *
 * This sketch combines MPU6050 reading and ESP-NOW transmission in one ESP32.
 * Useful for a standalone transmitter that doesn't need a separate Arduino.
 *
 * Hardware:
 * - ESP32
 * - MPU6050 (I2C: SDA=GPIO21, SCL=GPIO22 on most ESP32)
 *
 * This can work in two modes:
 * 1. Standalone: Reads MPU6050, processes data, and sends via ESP-NOW
 * 2. With PC: Also sends data to PC for visualization
 */

#include <Wire.h>
#include <MPU6050.h>
#include <esp_now.h>
#include <WiFi.h>

// MPU6050 object
MPU6050 mpu;

// Receiver ESP MAC Address - CHANGE THIS
uint8_t receiverMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Data structure for MAVLink control data
typedef struct
{
    float roll_stick;
    float pitch_stick;
    float throttle;
    float yaw_stick;
    float ax, ay, az;
    float gx, gy, gz;
} MavlinkData;

MavlinkData txData;
esp_now_peer_info_t peerInfo;

// Configuration
#define SEND_TO_PC true     // Set false if no PC connection needed
#define SEND_TO_ESPNOW true // Set false to disable ESP-NOW
#define UPDATE_RATE_MS 20   // 50Hz update rate

unsigned long lastUpdate = 0;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (SEND_TO_PC)
    {
        if (status == ESP_NOW_SEND_SUCCESS)
        {
            Serial.println("TX:OK");
        }
        else
        {
            Serial.println("TX:FAIL");
        }
    }
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    // Initialize MPU6050
    Serial.println("Initializing MPU6050...");
    mpu.initialize();

    if (!mpu.testConnection())
    {
        Serial.println("MPU6050 connection failed!");
        while (1)
            delay(100);
    }

    // Configure MPU6050
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // ±2g
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // ±250°/s

    Serial.println("MPU6050 OK");

    if (SEND_TO_ESPNOW)
    {
        // Set device as a Wi-Fi Station
        WiFi.mode(WIFI_STA);

        Serial.print("Transmitter MAC: ");
        Serial.println(WiFi.macAddress());

        // Init ESP-NOW
        if (esp_now_init() != ESP_OK)
        {
            Serial.println("ESP-NOW init failed!");
            return;
        }

        esp_now_register_send_cb(OnDataSent);

        // Register peer
        memcpy(peerInfo.peer_addr, receiverMAC, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;

        if (esp_now_add_peer(&peerInfo) != ESP_OK)
        {
            Serial.println("Failed to add peer");
            return;
        }

        Serial.println("ESP-NOW Ready");
    }

    Serial.println("System Ready");
}

void loop()
{
    unsigned long currentTime = millis();

    if (currentTime - lastUpdate >= UPDATE_RATE_MS)
    {
        lastUpdate = currentTime;

        // Read MPU6050
        int16_t ax, ay, az;
        int16_t gx, gy, gz;

        mpu.getAcceleration(&ax, &ay, &az);
        mpu.getRotation(&gx, &gy, &gz);

        // Convert to physical units
        float accelX = ax / 16384.0;
        float accelY = ay / 16384.0;
        float accelZ = az / 16384.0;
        float gyroX = gx / 131.0;
        float gyroY = gy / 131.0;
        float gyroZ = gz / 131.0;

        if (SEND_TO_PC)
        {
            // Send CSV format to PC for Python app (transmitter mode)
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
        }

        // Check for control data from PC (if connected)
        if (Serial.available())
        {
            String line = Serial.readStringUntil('\n');
            line.trim();

            if (line.startsWith("SEND:"))
            {
                // Received MAVLink data from PC to transmit via ESP-NOW
                String hexData = line.substring(5);

                if (hexData.length() == 80 && SEND_TO_ESPNOW)
                {
                    uint8_t buffer[40];
                    for (int i = 0; i < 40; i++)
                    {
                        String byteStr = hexData.substring(i * 2, i * 2 + 2);
                        buffer[i] = (uint8_t)strtol(byteStr.c_str(), NULL, 16);
                    }

                    memcpy(&txData, buffer, sizeof(MavlinkData));

                    // Send via ESP-NOW
                    esp_now_send(receiverMAC, (uint8_t *)&txData, sizeof(MavlinkData));
                }
            }
        }
    }
}
