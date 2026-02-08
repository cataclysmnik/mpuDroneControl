/*
 * I2C Scanner for ESP32
 *
 * This utility scans the I2C bus and reports all detected devices.
 * Use this to verify both MPU6050 sensors are properly connected.
 *
 * Expected Output:
 * - MPU #1 at address 0x68 (AD0 = GND)
 * - MPU #2 at address 0x69 (AD0 = 3.3V)
 *
 * Upload this sketch first to verify your wiring before using the main code.
 */

#include <Wire.h>

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for Serial Monitor

    delay(1000);

    Serial.println("\n\n========================================");
    Serial.println("   ESP32 I2C Scanner for Dual MPU6050");
    Serial.println("========================================");

    // Initialize I2C on ESP32 default pins
    Wire.begin(21, 22);    // SDA=GPIO21, SCL=GPIO22
    Wire.setClock(400000); // 400kHz

    Serial.println("I2C Bus Configuration:");
    Serial.println("  SDA: GPIO21");
    Serial.println("  SCL: GPIO22");
    Serial.println("  Speed: 400kHz");
    Serial.println("========================================\n");
}

void loop()
{
    byte error, address;
    int deviceCount = 0;

    Serial.println("Scanning I2C bus...");
    Serial.println("-------------------");

    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("✓ Device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);

            // Identify common devices
            if (address == 0x68)
            {
                Serial.println(" → MPU6050 #1 (AD0=GND)");
            }
            else if (address == 0x69)
            {
                Serial.println(" → MPU6050 #2 (AD0=3.3V)");
            }
            else
            {
                Serial.println(" → Unknown device");
            }

            deviceCount++;
        }
        else if (error == 4)
        {
            Serial.print("✗ Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }

    Serial.println("-------------------");
    Serial.print("Scan complete. Found ");
    Serial.print(deviceCount);
    Serial.println(" device(s).\n");

    // Diagnosis
    if (deviceCount == 0)
    {
        Serial.println("⚠ WARNING: No I2C devices found!");
        Serial.println("\nTroubleshooting:");
        Serial.println("1. Check SDA wire (GPIO21)");
        Serial.println("2. Check SCL wire (GPIO22)");
        Serial.println("3. Verify 3.3V power connection");
        Serial.println("4. Verify GND connection");
        Serial.println("5. Check for loose connections");
    }
    else if (deviceCount == 1)
    {
        Serial.println("⚠ WARNING: Only 1 device found!");
        Serial.println("\nExpected Configuration:");
        Serial.println("- MPU #1 at 0x68 (AD0 pin → GND)");
        Serial.println("- MPU #2 at 0x69 (AD0 pin → 3.3V)");
        Serial.println("\nCheck the second MPU's AD0 pin wiring.");
    }
    else if (deviceCount == 2)
    {
        Serial.println("✓ SUCCESS: Both MPU6050 sensors detected!");
        Serial.println("\nYou can now upload the main dual MPU code.");
    }
    else
    {
        Serial.println("⚠ Unexpected number of devices detected.");
        Serial.println("  Expected: 2 (both MPU6050 sensors)");
    }

    Serial.println("\n========================================");
    Serial.println("Next scan in 5 seconds...\n");
    delay(5000);
}
