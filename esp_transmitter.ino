/*
 * ESP-NOW Transmitter - Sends dual MPU6050 control data via ESP-NOW
 *
 * This ESP receives data from the PC (Python app) over Serial and transmits
 * it to another ESP using ESP-NOW protocol.
 *
 * Updated to support dual MPU6050 controller setup.
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
uint8_t receiverMAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Data structure for dual MPU control data (must match on both sides)
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

DualControlData txData;
esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status)
{
    // No status messages - keep serial output clean
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

        // Expected format: "SEND:<hex data>"
        // For dual MPU: 52 bytes = 104 hex chars
        if (line.startsWith("SEND:"))
        {
            String hexData = line.substring(5);

            // Convert hex string to bytes (DualControlData = 52 bytes)
            if (hexData.length() == 104)
            { // 52 bytes = 104 hex chars
                uint8_t buffer[52];
                for (int i = 0; i < 52; i++)
                {
                    String byteStr = hexData.substring(i * 2, i * 2 + 2);
                    buffer[i] = (uint8_t)strtol(byteStr.c_str(), NULL, 16);
                }

                // Copy to struct
                memcpy(&txData, buffer, sizeof(DualControlData));

                // Send via ESP-NOW (no status messages)
                esp_now_send(receiverMAC, (uint8_t *)&txData, sizeof(DualControlData));
            }
            // Silently ignore invalid data length
        }
    }

    delay(10);
}
