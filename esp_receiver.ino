/*
 * ESP-NOW Receiver - Receives dual MPU6050 control data via ESP-NOW
 *
 * This ESP receives data from the transmitter ESP (with dual MPU6050s)
 * via ESP-NOW and forwards it to the PC over Serial for display.
 *
 * Hardware:
 * - ESP32 or ESP8266
 * - Connected to PC running the Python dashboard in Receiver mode
 *
 * Data Flow:
 * Transmitter ESP32 (Dual MPU) -> ESP-NOW -> This ESP32 -> Serial -> PC
 */

#include <esp_now.h>
#include <WiFi.h>

// Data structure for dual MPU control data (must match transmitter)
typedef struct
{
    float roll_stick;                // From MPU1 X-axis
    float pitch_stick;               // From MPU1 Y-axis
    float throttle;                  // From MPU2 X-axis
    float yaw_stick;                 // From MPU2 Y-axis
    float mpu1_ax, mpu1_ay, mpu1_az; // MPU1 accelerometer
    float mpu1_gx, mpu1_gy, mpu1_gz; // MPU1 gyroscope
    float mpu2_ax, mpu2_ay, mpu2_az; // MPU2 accelerometer
    float mpu2_gx, mpu2_gy, mpu2_gz; // MPU2 gyroscope
} DualControlData;

DualControlData rxData;

// Callback when data is received
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len)
{
    if (len == sizeof(DualControlData))
    {
        memcpy(&rxData, incomingData, sizeof(DualControlData));

        // Convert to hex string for Python app
        uint8_t *dataBytes = (uint8_t *)&rxData;
        String hexString = "ESPNOW:";

        for (int i = 0; i < sizeof(DualControlData); i++)
        {
            if (dataBytes[i] < 16)
                hexString += "0";
            hexString += String(dataBytes[i], HEX);
        }
        hexString.toUpperCase();

        // Send to PC via Serial
        Serial.println(hexString);

        // Also send CSV format for easier debugging
        Serial.print("DUAL_CSV: ");
        Serial.print(rxData.roll_stick);
        Serial.print(",");
        Serial.print(rxData.pitch_stick);
        Serial.print(",");
        Serial.print(rxData.throttle);
        Serial.print(",");
        Serial.print(rxData.yaw_stick);
        Serial.print(",");
        // MPU1 data
        Serial.print(rxData.mpu1_ax);
        Serial.print(",");
        Serial.print(rxData.mpu1_ay);
        Serial.print(",");
        Serial.print(rxData.mpu1_az);
        Serial.print(",");
        Serial.print(rxData.mpu1_gx);
        Serial.print(",");
        Serial.print(rxData.mpu1_gy);
        Serial.print(",");
        Serial.print(rxData.mpu1_gz);
        Serial.print(",");
        // MPU2 data
        Serial.print(rxData.mpu2_ax);
        Serial.print(",");
        Serial.print(rxData.mpu2_ay);
        Serial.print(",");
        Serial.print(rxData.mpu2_az);
        Serial.print(",");
        Serial.print(rxData.mpu2_gx);
        Serial.print(",");
        Serial.print(rxData.mpu2_gy);
        Serial.print(",");
        Serial.println(rxData.mpu2_gz);
    }
    else
    {
        Serial.print("Received wrong data size: ");
        Serial.print(len);
        Serial.print(" expected: ");
        Serial.println(sizeof(DualControlData));
    }
}

void setup()
{
    Serial.begin(115200);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Print MAC address for configuration
    Serial.print("Receiver MAC Address: ");
    Serial.println(WiFi.macAddress());
    Serial.println("Copy this MAC to transmitter code!");

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register receive callback
    esp_now_register_recv_cb(OnDataRecv);

    Serial.println("ESP-NOW Receiver Ready");
    Serial.println("Waiting for ESP-NOW data...");
}

void loop()
{
    // Nothing to do here - all handled in callback
    delay(10);
}
