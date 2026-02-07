/*
 * Combined MPU6050 + ESP-NOW Transmitter
 *
 * This sketch combines MPU6050 reading and ESP-NOW transmission in one ESP32.
 * Useful for a standalone transmitter that doesn't need a separate Arduino.
 *
 * Hardware:
 * - ESP32
 * - MPU6050 (I2C: SDA=GPIO21, SCL=GPIO22)
 *
 * This can work in two modes:
 * 1. Standalone: Reads MPU6050, processes data, and sends via ESP-NOW
 * 2. With PC: Also sends data to PC for visualization
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_now.h>
#include <WiFi.h>

// MPU6050 object
Adafruit_MPU6050 mpu;

// Receiver ESP MAC Address - CHANGE THIS
uint8_t receiverMAC[] = {0xC0, 0xCD, 0xD6, 0x8D, 0xAB, 0x1C};

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
#define UPDATE_RATE_MS 10   // ~100Hz update rate

unsigned long lastUpdate = 0;

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status)
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

    delay(1500); // Allow MPU to boot
    Wire.begin(21, 22);
    Wire.setClock(100000);

    // Initialize MPU6050
    Serial.println("Initializing MPU6050...");

    while (!mpu.begin())
    {
        Serial.println("MPU6050 connection failed! Retrying...");
        delay(200);
    }

    // Configure MPU6050
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

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
        sensors_event_t accel, gyro, temp;
        mpu.getEvent(&accel, &gyro, &temp);

        // Extract values (Adafruit library returns in m/s² and rad/s)
        float accelX = accel.acceleration.x;
        float accelY = accel.acceleration.y;
        float accelZ = accel.acceleration.z;
        float gyroX = gyro.gyro.x;
        float gyroY = gyro.gyro.y;
        float gyroZ = gyro.gyro.z;

        if (SEND_TO_PC)
        {
            // Send CSV format to PC for Python app (transmitter mode)
            Serial.print(accelX);
            Serial.print(",");
            Serial.print(accelY);
            Serial.print(",");
            Serial.print(accelZ);
            Serial.print(",");
            Serial.print(gyroX);
            Serial.print(",");
            Serial.print(gyroY);
            Serial.print(",");
            Serial.println(gyroZ);
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
