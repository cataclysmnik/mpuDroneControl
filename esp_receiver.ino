/*
 * ESP-NOW Receiver - Receives MAVLink data via ESP-NOW and sends to PC
 *
 * This ESP receives data from the transmitter ESP via ESP-NOW and forwards
 * it to the PC over Serial for display in the Python dashboard.
 *
 * Hardware:
 * - ESP32 or ESP8266
 * - Connected to PC running the Python dashboard in Receiver mode
 *
 * Data Flow:
 * Transmitter ESP32 -> ESP-NOW -> This ESP32 -> Serial -> PC (Python)
 */

#include <esp_now.h>
#include <WiFi.h>

// Data structure for MAVLink control data (must match transmitter)
typedef struct
{
    float roll_stick;
    float pitch_stick;
    float throttle;
    float yaw_stick;
    float ax, ay, az; // Accelerometer
    float gx, gy, gz; // Gyroscope
} MavlinkData;

MavlinkData rxData;

// Callback when data is received
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len)
{
    if (len == sizeof(MavlinkData))
    {
        memcpy(&rxData, incomingData, sizeof(MavlinkData));

        // Convert to hex string for Python app
        uint8_t *dataBytes = (uint8_t *)&rxData;
        String hexString = "ESPNOW:";

        for (int i = 0; i < sizeof(MavlinkData); i++)
        {
            if (dataBytes[i] < 16)
                hexString += "0";
            hexString += String(dataBytes[i], HEX);
        }
        hexString.toUpperCase();

        // Send to PC via Serial
        Serial.println(hexString);

        // Also send CSV format for easier debugging
        Serial.print("CSV: ");
        Serial.print(rxData.roll_stick);
        Serial.print(",");
        Serial.print(rxData.pitch_stick);
        Serial.print(",");
        Serial.print(rxData.throttle);
        Serial.print(",");
        Serial.print(rxData.yaw_stick);
        Serial.print(",");
        Serial.print(rxData.ax);
        Serial.print(",");
        Serial.print(rxData.ay);
        Serial.print(",");
        Serial.print(rxData.az);
        Serial.print(",");
        Serial.print(rxData.gx);
        Serial.print(",");
        Serial.print(rxData.gy);
        Serial.print(",");
        Serial.println(rxData.gz);
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
