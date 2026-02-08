/*
 * Dual MPU6050 + ESP-NOW Transmitter
 *
 * This sketch reads TWO MPU6050 sensors and sends dual-control data via ESP-NOW
 *
 * MPU #1 (I2C Address 0x68) - Roll & Pitch Control
 *   - Tilt around X-axis → Roll (orientation angle)
 *   - Tilt around Y-axis → Pitch (orientation angle)
 *
 * MPU #2 (I2C Address 0x69) - Throttle & Yaw Control
 *   - Tilt around Y-axis → Throttle (0% at level, increases with forward tilt)
 *   - Tilt around X-axis → Yaw (orientation angle)
 *
 * Hardware:
 * - ESP32
 * - MPU6050 #1 (AD0 LOW → 0x68) for Roll/Pitch
 * - MPU6050 #2 (AD0 HIGH → 0x69) for Throttle/Yaw
 * - Both share I2C: SDA=GPIO21, SCL=GPIO22
 *
 * See connection diagram at end of file
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_now.h>
#include <WiFi.h>

// Two MPU6050 objects with different I2C addresses
Adafruit_MPU6050 mpu1; // 0x68 - Roll & Pitch
Adafruit_MPU6050 mpu2; // 0x69 - Throttle & Yaw

// Receiver ESP MAC Address - CHANGE THIS
uint8_t receiverMAC[] = {0xC0, 0xCD, 0xD6, 0x8D, 0xAB, 0x1C};

// Data structure for dual control data
typedef struct
{
    float roll_stick;                // From MPU1 X-axis (roll angle)
    float pitch_stick;               // From MPU1 Y-axis (pitch angle)
    float throttle;                  // From MPU2 Y-axis (pitch angle)
    float yaw_stick;                 // From MPU2 X-axis (roll angle)
    float mpu1_ax, mpu1_ay, mpu1_az; // MPU1 accelerometer
    float mpu1_gx, mpu1_gy, mpu1_gz; // MPU1 gyroscope
    float mpu2_ax, mpu2_ay, mpu2_az; // MPU2 accelerometer
    float mpu2_gx, mpu2_gy, mpu2_gz; // MPU2 gyroscope
} DualControlData;

DualControlData txData;
esp_now_peer_info_t peerInfo;

// Configuration
#define SEND_TO_PC true     // Set false if no PC connection needed
#define SEND_TO_ESPNOW true // Set false to disable ESP-NOW
#define UPDATE_RATE_MS 20   // ~50Hz update rate (reduced for dual sensor)

// Calibration offsets (not needed for orientation-based control)
// These could be used for accelerometer calibration if needed

// Control mapping ranges (adjust these to your preference)
// These scales map tilt angles (in degrees) to control values
#define ROLL_SCALE 2.0      // degrees of tilt to control units
#define PITCH_SCALE 2.0     // degrees of tilt to control units
#define THROTTLE_SCALE 3.33 // degrees of tilt to control units (30° = 100%)
#define YAW_SCALE 2.0       // degrees of tilt to control units

#define MAX_TILT_ANGLE 50.0 // Maximum tilt angle in degrees for full control

// Smoothing configuration
#define SMOOTHING_FACTOR 0.15 // Lower = more smoothing (0.0-1.0), 0.15 provides good balance

// Smoothed values for control outputs
float smoothed_roll = 0.0;
float smoothed_pitch = 0.0;
float smoothed_throttle = 0.0; // Start at zero throttle (level condition)
float smoothed_yaw = 0.0;

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

// Helper function to calculate roll angle from accelerometer
float calculateRoll(float ay, float az)
{
    return atan2(ay, az) * 180.0 / PI;
}

// Helper function to calculate pitch angle from accelerometer
float calculatePitch(float ax, float ay, float az)
{
    return atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
}

// Exponential Moving Average smoothing function
float smoothValue(float current_value, float previous_smoothed, float alpha)
{
    return (alpha * current_value) + ((1.0 - alpha) * previous_smoothed);
}

void setup()
{
    Serial.begin(115200);
    delay(1500); // Allow MPUs to boot

    // Initialize I2C
    Wire.begin(21, 22);    // SDA=21, SCL=22
    Wire.setClock(400000); // 400kHz I2C speed

    // Initialize MPU6050 #1 (0x68) - Roll & Pitch
    Serial.println("Initializing MPU6050 #1 (0x68 - Roll/Pitch)...");
    if (!mpu1.begin(0x68, &Wire))
    {
        Serial.println("Failed to find MPU #1 at 0x68!");
        Serial.println("Check wiring and AD0 pin (must be LOW/GND)");
        while (1)
        {
            delay(1000);
        }
    }

    mpu1.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu1.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu1.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 #1 OK");

    // Initialize MPU6050 #2 (0x69) - Throttle & Yaw
    Serial.println("Initializing MPU6050 #2 (0x69 - Throttle/Yaw)...");
    if (!mpu2.begin(0x69, &Wire))
    {
        Serial.println("Failed to find MPU #2 at 0x69!");
        Serial.println("Check wiring and AD0 pin (must be HIGH/3.3V)");
        while (1)
        {
            delay(1000);
        }
    }

    mpu2.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu2.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu2.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 #2 OK");

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

    Serial.println("\n=== DUAL MPU6050 SYSTEM READY ===");
    Serial.println("MPU1 (0x68): Roll & Pitch");
    Serial.println("MPU2 (0x69): Throttle & Yaw");
    Serial.println("================================\n");
}

void loop()
{
    unsigned long currentTime = millis();

    if (currentTime - lastUpdate >= UPDATE_RATE_MS)
    {
        lastUpdate = currentTime;

        // Read both MPU6050 sensors
        sensors_event_t accel1, gyro1, temp1;
        sensors_event_t accel2, gyro2, temp2;

        mpu1.getEvent(&accel1, &gyro1, &temp1);
        mpu2.getEvent(&accel2, &gyro2, &temp2);

        // Store raw accelerometer and gyro data for telemetry
        txData.mpu1_ax = accel1.acceleration.x;
        txData.mpu1_ay = accel1.acceleration.y;
        txData.mpu1_az = accel1.acceleration.z;
        txData.mpu1_gx = gyro1.gyro.x;
        txData.mpu1_gy = gyro1.gyro.y;
        txData.mpu1_gz = gyro1.gyro.z;

        txData.mpu2_ax = accel2.acceleration.x;
        txData.mpu2_ay = accel2.acceleration.y;
        txData.mpu2_az = accel2.acceleration.z;
        txData.mpu2_gx = gyro2.gyro.x;
        txData.mpu2_gy = gyro2.gyro.y;
        txData.mpu2_gz = gyro2.gyro.z;

        // MPU1 Control Mapping (Roll & Pitch) - Based on Orientation
        // Calculate roll from accelerometer (tilt around X-axis)
        float roll_angle = calculateRoll(accel1.acceleration.y, accel1.acceleration.z);
        float roll_raw = constrain(roll_angle * ROLL_SCALE, -100.0, 100.0);
        smoothed_roll = smoothValue(roll_raw, smoothed_roll, SMOOTHING_FACTOR);
        txData.roll_stick = smoothed_roll;

        // Calculate pitch from accelerometer (tilt around Y-axis)
        float pitch_angle = calculatePitch(accel1.acceleration.x, accel1.acceleration.y, accel1.acceleration.z);
        float pitch_raw = constrain(pitch_angle * PITCH_SCALE, -100.0, 100.0);
        smoothed_pitch = smoothValue(pitch_raw, smoothed_pitch, SMOOTHING_FACTOR);
        txData.pitch_stick = smoothed_pitch;

        // MPU2 Control Mapping (Throttle & Yaw) - Based on Orientation
        // Calculate throttle from tilt angle (0-100 range) - using pitch axis
        // Level = 0%, tilted forward = higher throttle (0-100%)
        float throttle_angle = calculatePitch(accel2.acceleration.x, accel2.acceleration.y, accel2.acceleration.z);
        float throttle_raw = constrain(throttle_angle * THROTTLE_SCALE, 0.0, 100.0);
        smoothed_throttle = smoothValue(throttle_raw, smoothed_throttle, SMOOTHING_FACTOR);
        txData.throttle = smoothed_throttle;

        // Calculate yaw from tilt angle - using roll axis
        float yaw_angle = calculateRoll(accel2.acceleration.y, accel2.acceleration.z);
        float yaw_raw = constrain(yaw_angle * YAW_SCALE, -100.0, 100.0);
        smoothed_yaw = smoothValue(yaw_raw, smoothed_yaw, SMOOTHING_FACTOR);
        txData.yaw_stick = smoothed_yaw;

        // Send data to PC for monitoring (CSV format)
        if (SEND_TO_PC)
        {
            Serial.print("DUAL:");
            Serial.print(txData.roll_stick);
            Serial.print(",");
            Serial.print(txData.pitch_stick);
            Serial.print(",");
            Serial.print(txData.throttle);
            Serial.print(",");
            Serial.print(txData.yaw_stick);
            Serial.print(",");
            // MPU1 raw data
            Serial.print(txData.mpu1_gx);
            Serial.print(",");
            Serial.print(txData.mpu1_gy);
            Serial.print(",");
            Serial.print(txData.mpu1_gz);
            Serial.print(",");
            // MPU2 raw data
            Serial.print(txData.mpu2_gx);
            Serial.print(",");
            Serial.print(txData.mpu2_gy);
            Serial.print(",");
            Serial.println(txData.mpu2_gz);
        }

        // Send via ESP-NOW
        if (SEND_TO_ESPNOW)
        {
            esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&txData, sizeof(DualControlData));

            if (result != ESP_OK && SEND_TO_PC)
            {
                Serial.println("ESP-NOW send error");
            }
        }
    }
}

/*
 * ============================================
 * WIRING DIAGRAM FOR DUAL MPU6050 SETUP
 * ============================================
 *
 * ESP32 Pinout:
 * GPIO21 → SDA (shared by both MPUs)
 * GPIO22 → SCL (shared by both MPUs)
 * 3.3V   → VCC (both MPUs)
 * GND    → GND (both MPUs)
 *
 * MPU6050 #1 (Roll & Pitch) - Address 0x68:
 * ┌─────────────────┐
 * │  MPU6050 #1     │
 * ├─────────────────┤
 * │ VCC  → 3.3V     │
 * │ GND  → GND      │
 * │ SCL  → GPIO22   │
 * │ SDA  → GPIO21   │
 * │ AD0  → GND      │ ← CRITICAL: AD0 to GND for 0x68
 * │ INT  → (unused) │
 * └─────────────────┘
 *
 * MPU6050 #2 (Throttle & Yaw) - Address 0x69:
 * ┌─────────────────┐
 * │  MPU6050 #2     │
 * ├─────────────────┤
 * │ VCC  → 3.3V     │
 * │ GND  → GND      │
 * │ SCL  → GPIO22   │
 * │ SDA  → GPIO21   │
 * │ AD0  → 3.3V     │ ← CRITICAL: AD0 to 3.3V for 0x69
 * │ INT  → (unused) │
 * └─────────────────┘
 *
 * Connection Summary:
 * - Both MPUs share the same I2C bus (SDA & SCL)
 * - MPU1: AD0 pin connected to GND → I2C address 0x68
 * - MPU2: AD0 pin connected to 3.3V → I2C address 0x69
 * - This allows both sensors to work on the same bus
 *
 * Physical Layout Recommendation:
 * - Mount MPU1 on the right grip (for roll/pitch control)
 * - Mount MPU2 on the left grip (for throttle/yaw control)
 * - Keep wires short to minimize noise
 * - Use 4.7kΩ pull-up resistors on SDA/SCL if having connection issues
 *
 * Control Mapping:
 * MPU1 (Right Grip):
 *   - Tilt left/right → Roll (orientation angle)
 *   - Tilt forward/back → Pitch (orientation angle)
 *
 * MPU2 (Left Grip):
 *   - Tilt forward/back → Throttle (0% at level, 100% at ~30° forward)
 *   - Tilt left/right → Yaw (orientation angle)
 *
 * ============================================
 */
