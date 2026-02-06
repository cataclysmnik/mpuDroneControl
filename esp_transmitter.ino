/*
 * ESP-NOW Transmitter - Reads MPU6050 and sends MAVLink data via ESP-NOW
 *
 * This ESP receives data from the PC (Python app) over Serial and transmits
 * it to another ESP using ESP-NOW protocol.
 *
 * Hardware:
 * - ESP32 or ESP8266
 * - Connected to PC running the Python dashboard in Transmitter mode
 *
 * Data Flow:
 * PC (Python) -> Serial -> ESP32 -> ESP-NOW -> Receiver ESP32
 */

#include <esp_now.h>
#include <WiFi.h>

// Receiver ESP MAC Address - CHANGE THIS to your receiver's MAC address
// To find MAC: Upload a simple sketch that prints WiFi.macAddress()
uint8_t receiverMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Data structure for MAVLink control data (must match on both sides)
typedef struct
{
    float roll_stick;
    float pitch_stick;
    float throttle;
    float yaw_stick;
    float ax, ay, az; // Accelerometer
    float gx, gy, gz; // Gyroscope
} MavlinkData;

MavlinkData txData;
esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        Serial.println("ESPNOW_TX:OK");
    }
    else
    {
        Serial.println("ESPNOW_TX:FAIL");
    }
}

void setup()
{
    Serial.begin(115200);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Print MAC address for reference
    Serial.print("Transmitter MAC: ");
    Serial.println(WiFi.macAddress());

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register send callback
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

    Serial.println("ESP-NOW Transmitter Ready");
    Serial.println("Waiting for data from PC...");
}

void loop()
{
    // Check for data from Serial (PC Python app)
    if (Serial.available())
    {
        String line = Serial.readStringUntil('\n');
        line.trim();

        // Expected format: "SEND:<40 bytes of hex data>"
        if (line.startsWith("SEND:"))
        {
            String hexData = line.substring(5);

            // Convert hex string to bytes
            if (hexData.length() == 80)
            { // 40 bytes = 80 hex chars
                uint8_t buffer[40];
                for (int i = 0; i < 40; i++)
                {
                    String byteStr = hexData.substring(i * 2, i * 2 + 2);
                    buffer[i] = (uint8_t)strtol(byteStr.c_str(), NULL, 16);
                }

                // Copy to struct
                memcpy(&txData, buffer, sizeof(MavlinkData));

                // Send via ESP-NOW
                esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&txData, sizeof(MavlinkData));

                if (result == ESP_OK)
                {
                    Serial.print("Data sent: Roll=");
                    Serial.print(txData.roll_stick);
                    Serial.print(" Pitch=");
                    Serial.print(txData.pitch_stick);
                    Serial.print(" Thr=");
                    Serial.println(txData.throttle);
                }
                else
                {
                    Serial.println("Error sending data");
                }
            }
        }
    }

    delay(10);
}
