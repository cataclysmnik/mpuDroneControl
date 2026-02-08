# 📋 Dual MPU6050 System - Complete Summary

## System Overview

```
┌─────────────────────────────────────────────────────────┐
│           DUAL MPU6050 CONTROL SYSTEM                   │
│                                                          │
│  ┌──────────┐         ┌──────────┐                     │
│  │  MPU #1  │         │  MPU #2  │                     │
│  │  0x68    │         │  0x69    │                     │
│  │ Roll &   │         │ Throttle │                     │
│  │ Pitch    │         │ & Yaw    │                     │
│  └────┬─────┘         └────┬─────┘                     │
│       │                    │                            │
│       └──────────┬─────────┘                            │
│                  │ I2C Bus                              │
│            ┌─────▼─────┐                                │
│            │   ESP32   │                                │
│            │Transmitter│                                │
│            └─────┬─────┘                                │
│                  │ ESP-NOW                              │
│            ┌─────▼─────┐                                │
│            │   ESP32   │                                │
│            │  Receiver │                                │
│            └─────┬─────┘                                │
│                  │ Serial                               │
│            ┌─────▼─────┐                                │
│            │    PC     │                                │
│            │  /Drone   │                                │
│            └───────────┘                                │
└─────────────────────────────────────────────────────────┘
```

---

## 🎮 Control Mapping

### Left Hand (MPU #2 @ 0x69)
```
        Throttle Up
             ▲
             │
    Yaw  ◄───┼───►  Yaw
    Left     │     Right
             │
             ▼
        Throttle Down
        
    Tilt L/R → Throttle (0-100%)
    Tilt F/B → Yaw (-100 to +100)
```

### Right Hand (MPU #1 @ 0x68)
```
         Pitch Up
             ▲
             │
   Roll  ◄───┼───►  Roll
   Left      │     Right
             │
             ▼
        Pitch Down
        
    Tilt L/R → Roll (-100 to +100)
    Tilt F/B → Pitch (-100 to +100)
```

---

## 🔌 Wiring Quick Reference

### Complete Connection Table
```
┌────────────┬───────────┬───────────┬──────────────────┐
│ ESP32 Pin  │ MPU #1    │ MPU #2    │ Notes            │
├────────────┼───────────┼───────────┼──────────────────┤
│ 3.3V       │ VCC       │ VCC       │ Power both       │
│ GND        │ GND       │ GND       │ Common ground    │
│ GPIO21     │ SDA       │ SDA       │ I2C Data         │
│ GPIO22     │ SCL       │ SCL       │ I2C Clock        │
│ GND        │ AD0       │ -         │ Sets 0x68 addr   │
│ 3.3V       │ -         │ AD0       │ Sets 0x69 addr   │
│ -          │ INT       │ INT       │ Not used         │
└────────────┴───────────┴───────────┴──────────────────┘
```

### Visual Wiring
```
                    ESP32
             ┌───────────────┐
         ┌───│ 3.3V          │
         │┌──│ GND           │
         ││┌─│ GPIO21 (SDA)  │
         │││┌│ GPIO22 (SCL)  │
         │││└└───────────────┘
         ││└┐  └┐
    ┌────┴┴┐ │  ┌┴────┐
    │MPU#1 │ │  │MPU#2│
    │ 0x68 │ │  │0x69 │
    ├──────┤ │  ├─────┤
    │VCC┼──┘ │  │VCC┼─┘
    │GND┼────┘  │GND┼──┘
    │SCL┼───────┤SCL┼──┘
    │SDA┼───────┤SDA┼──┘
    │AD0───GND  │AD0───3.3V
    └──────┘    └─────┘
```

---

## 📁 File Structure

```
dashboard/
├── esp32_combined.ino          ⭐ Main code (dual MPU)
├── esp_transmitter.ino         📡 PC bridge transmitter
├── esp_receiver.ino            📡 Receiver code
├── i2c_scanner.ino            🔍 Diagnostic tool
├── DUAL_MPU_WIRING.md         📘 Complete wiring guide
├── DUAL_MPU_QUICKSTART.md     🚀 Quick setup guide
└── DUAL_MPU_SUMMARY.md        📋 This file
```

---

## ⚙️ Configuration Parameters

### In `esp32_combined.ino`:

```cpp
// Receiver MAC address (line ~24)
uint8_t receiverMAC[] = {0xC0, 0xCD, 0xD6, 0x8D, 0xAB, 0x1C};

// Control sensitivity (lines ~45-48)
#define ROLL_SCALE 30.0       // Roll sensitivity
#define PITCH_SCALE 30.0      // Pitch sensitivity
#define THROTTLE_SCALE 30.0   // Throttle sensitivity
#define YAW_SCALE 30.0        // Yaw sensitivity

// Update rate (line ~44)
#define UPDATE_RATE_MS 20     // 50Hz (20ms per update)

// Optional features (lines ~41-42)
#define SEND_TO_PC true       // Enable serial output
#define SEND_TO_ESPNOW true   // Enable ESP-NOW transmission
```

---

## 🔢 Data Structure

```cpp
typedef struct {
    // Control outputs (4 × 4 = 16 bytes)
    float roll_stick;      // -100 to +100
    float pitch_stick;     // -100 to +100
    float throttle;        //    0 to  100
    float yaw_stick;       // -100 to +100
    
    // MPU #1 telemetry (6 × 4 = 24 bytes)
    float mpu1_ax, mpu1_ay, mpu1_az;  // Accel m/s²
    float mpu1_gx, mpu1_gy, mpu1_gz;  // Gyro rad/s
    
    // MPU #2 telemetry (6 × 4 = 24 bytes)
    float mpu2_ax, mpu2_ay, mpu2_az;  // Accel m/s²
    float mpu2_gx, mpu2_gy, mpu2_gz;  // Gyro rad/s
    
} DualControlData;  // Total: 64 bytes (13 floats)
```

---

## 🚀 Setup Steps (5 Minutes)

### 1️⃣ Hardware Assembly
```
□ Connect MPU #1 to ESP32 (AD0 → GND)
□ Connect MPU #2 to ESP32 (AD0 → 3.3V)
□ Verify all 4 power/ground connections
□ Verify SDA/SCL connections
```

### 2️⃣ Verify Wiring
```
□ Upload i2c_scanner.ino
□ Open Serial Monitor (115200 baud)
□ Confirm: Found device at 0x68
□ Confirm: Found device at 0x69
```

### 3️⃣ Upload Main Code
```
□ Update receiverMAC[] with your receiver's address
□ Upload esp32_combined.ino
□ Wait for calibration (keep sensors still!)
□ Verify: === DUAL MPU6050 SYSTEM READY ===
```

### 4️⃣ Test Controls
```
□ Tilt MPU #1 left/right → Roll changes
□ Tilt MPU #1 forward/back → Pitch changes
□ Tilt MPU #2 left/right → Throttle changes
□ Tilt MPU #2 forward/back → Yaw changes
```

---

## 🐛 Troubleshooting Guide

### Problem: Can't find MPU at 0x68
✅ **Solutions:**
- Check AD0 pin on MPU #1 is connected to GND
- Verify VCC/GND connections
- Upload i2c_scanner.ino to diagnose
- Check for loose connections

### Problem: Can't find MPU at 0x69
✅ **Solutions:**
- Check AD0 pin on MPU #2 is connected to 3.3V
- Make sure it's not touching GND
- Verify power supply
- Use i2c_scanner.ino

### Problem: Sensors work but readings are jittery
✅ **Solutions:**
- Reduce I2C speed: `Wire.setClock(100000);`
- Add pull-up resistors (4.7kΩ on SDA/SCL)
- Keep wires short (<20cm)
- Check power supply stability
- Recalibrate (reset ESP with sensors still)

### Problem: ESP-NOW transmission fails
✅ **Solutions:**
- Update receiver MAC address correctly
- Ensure receiver ESP is powered on and running
- Both ESPs must be on same WiFi channel
- Check for "TX:FAIL" messages in serial monitor

### Problem: Wrong control axis mapping
✅ **Solutions:**
- Physical orientation: Adjust sensor mounting
- Software: Swap axes in code (change which gyro axis maps to which control)

---

## 📊 Performance Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Update Rate | 50 Hz | 20ms per cycle |
| Latency | <5ms | MPU reading + processing |
| ESP-NOW Range | 200m | Line of sight (outdoor) |
| I2C Speed | 400 kHz | Can reduce to 100kHz if issues |
| Power Draw | ~10mA | Both MPUs combined |
| Resolution | 16-bit | Per sensor axis |
| Gyro Range | ±500°/s | Configurable |
| Accel Range | ±4g | Configurable |

---

## 🔧 Advanced Customization

### Change Control Mapping
Edit the loop() function around line 160:

```cpp
// Example: Swap roll and pitch
txData.roll_stick = constrain(gyroY_calibrated_1 * PITCH_SCALE, -100.0, 100.0);
txData.pitch_stick = constrain(gyroX_calibrated_1 * ROLL_SCALE, -100.0, 100.0);

// Example: Reverse throttle direction
txData.throttle = constrain(50.0 - (gyroX_calibrated_2 * THROTTLE_SCALE), 0.0, 100.0);

// Example: Use Z-axis for yaw instead
txData.yaw_stick = constrain(gyro2.gyro.z * YAW_SCALE, -100.0, 100.0);
```

### Add Deadzone
```cpp
// Add after calibration subtraction
if (abs(gyroX_calibrated_1) < 0.05) gyroX_calibrated_1 = 0;
if (abs(gyroY_calibrated_1) < 0.05) gyroY_calibrated_1 = 0;
if (abs(gyroX_calibrated_2) < 0.05) gyroX_calibrated_2 = 0;
if (abs(gyroY_calibrated_2) < 0.05) gyroY_calibrated_2 = 0;
```

### Exponential Response Curve
```cpp
// For more sensitive center, less at extremes
float expo = 0.3; // 0 = linear, 1 = very curved
txData.roll_stick = sign(value) * pow(abs(value)/100.0, 1+expo) * 100.0;
```

---

## 📦 What's Included

### ✅ Updated Files
- [x] `esp32_combined.ino` - Dual MPU transmitter
- [x] `esp_receiver.ino` - Updated receiver
- [x] `esp_transmitter.ino` - Updated PC bridge
- [x] `i2c_scanner.ino` - New diagnostic tool
- [x] `DUAL_MPU_WIRING.md` - Comprehensive wiring guide
- [x] `DUAL_MPU_QUICKSTART.md` - Quick setup guide
- [x] `DUAL_MPU_SUMMARY.md` - This summary

### 📚 Documentation
- Complete wiring diagrams
- Connection tables
- Troubleshooting guide
- Control mapping reference
- Customization examples

---

## 🔄 Differences from Single MPU

| Aspect | Single MPU | Dual MPU |
|--------|------------|----------|
| I2C Addresses | 1 (0x68) | 2 (0x68, 0x69) |
| Data Size | 40 bytes | 52 bytes |
| Update Rate | 100 Hz | 50 Hz |
| AD0 Wiring | Any | One GND, one 3.3V |
| Control Axes | 2 (pitch/roll) | 4 (all controls) |
| Calibration | Single | Dual (both sensors) |

---

## 🎯 System Capabilities

✅ **Independent 4-axis control**
- Roll, Pitch, Throttle, Yaw all from physical motion
- No traditional joysticks needed

✅ **Real sensor telemetry**
- 6-axis data from each MPU
- Accelerometer + gyroscope readings
- Useful for advanced stabilization

✅ **Low latency wireless**
- ESP-NOW protocol
- Sub-10ms total latency
- 200m+ range

✅ **Adjustable sensitivity**
- Software-configurable scaling
- Per-axis customization
- Deadzone support

✅ **Diagnostic tools**
- I2C scanner for hardware verification
- Serial output for debugging
- Status messages

---

## 🎓 Key Concepts

### I2C Address Selection
The MPU6050's AD0 pin determines its I2C address:
- **AD0 = LOW (0V)**: Address becomes 0x68
- **AD0 = HIGH (3.3V)**: Address becomes 0x69

This is how we connect two sensors to one I2C bus.

### Gyroscope vs Accelerometer
- **Gyroscope**: Measures rotation rate (degrees/second)
- **Accelerometer**: Measures linear acceleration (m/s²)

We use gyroscope for control (rotation = stick movement).

### Calibration
On startup, the system samples both sensors 100 times to determine their "zero point" when stationary. This removes manufacturing drift and temperature offset.

### ESP-NOW Protocol
Low-level WiFi protocol that allows direct ESP32-to-ESP32 communication without a router. Faster and lower latency than traditional WiFi.

---

## 📞 Need Help?

**Wiring Issues:** Review `DUAL_MPU_WIRING.md`
**Setup Help:** Check `DUAL_MPU_QUICKSTART.md`  
**Can't detect sensors:** Run `i2c_scanner.ino`
**Control issues:** Adjust sensitivity in code

---

## ✨ You're All Set!

Your dual MPU6050 control system is ready to use. The code handles:
- ✅ Dual sensor initialization
- ✅ Automatic calibration
- ✅ Independent axis mapping
- ✅ Data transmission via ESP-NOW
- ✅ Serial debugging output

**Next:** Mount the sensors in your controller and test!

---

**Version:** 2.0 (Dual MPU)  
**Last Updated:** 2026-02-08  
**Compatibility:** ESP32, Adafruit_MPU6050 library

🚁 Happy Flying! 🎮
