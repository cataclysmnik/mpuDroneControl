# Dual MPU6050 Quick Setup Guide

## What Changed?

Your system now supports **TWO MPU6050 sensors** for independent control:

### MPU #1 (Address 0x68) - Roll & Pitch
- **X-axis rotation** → Roll control
- **Y-axis rotation** → Pitch control

### MPU #2 (Address 0x69) - Throttle & Yaw
- **X-axis rotation** → Throttle control (0-100%)
- **Y-axis rotation** → Yaw control

---

## Hardware Setup (5 Minutes)

### Step 1: Wire MPU #1 (Roll/Pitch)
```
MPU #1          ESP32
------          -----
VCC      →      3.3V
GND      →      GND
SCL      →      GPIO22
SDA      →      GPIO21
AD0      →      GND  ⚠️ IMPORTANT
```

### Step 2: Wire MPU #2 (Throttle/Yaw)
```
MPU #2          ESP32
------          -----
VCC      →      3.3V
GND      →      GND
SCL      →      GPIO22
SDA      →      GPIO21
AD0      →      3.3V  ⚠️ IMPORTANT (Different!)
```

**Critical:** The AD0 pin determines the I2C address:
- MPU #1: AD0 → GND = Address 0x68
- MPU #2: AD0 → 3.3V = Address 0x69

---

## Software Setup

### Option 1: Verify Wiring (Recommended First)
1. Upload `i2c_scanner.ino` to ESP32
2. Open Serial Monitor (115200 baud)
3. Verify both sensors detected:
   ```
   ✓ Device found at address 0x68 → MPU6050 #1
   ✓ Device found at address 0x69 → MPU6050 #2
   Found 2 device(s).
   ```

### Option 2: Upload Main Code
1. Upload `esp32_combined.ino` to ESP32
2. Open Serial Monitor (115200 baud)
3. You should see:
   ```
   Initializing MPU6050 #1 (0x68)...
   MPU6050 #1 OK
   Initializing MPU6050 #2 (0x69)...
   MPU6050 #2 OK
   Calibrating sensors...
   === DUAL MPU6050 SYSTEM READY ===
   ```

---

## Configuration

### Adjust Receiver MAC Address
Edit line in `esp32_combined.ino`:
```cpp
uint8_t receiverMAC[] = {0xC0, 0xCD, 0xD6, 0x8D, 0xAB, 0x1C};
```
Change to your receiver ESP32's MAC address.

### Adjust Control Sensitivity
Edit these values in `esp32_combined.ino`:
```cpp
#define ROLL_SCALE 30.0       // Increase = more sensitive
#define PITCH_SCALE 30.0      // Increase = more sensitive
#define THROTTLE_SCALE 30.0   // Increase = more sensitive
#define YAW_SCALE 30.0        // Increase = more sensitive
```

---

## Testing

### 1. Basic Sensor Test
Keep both sensors still during startup for calibration (2 seconds).

### 2. Test MPU #1 (Roll/Pitch)
Tilt MPU #1:
- Left/Right → Should see "Roll" value change
- Forward/Back → Should see "Pitch" value change

Serial output:
```
DUAL:-15.2,8.4,50.0,0.0,...
      ^^^^  ^^^
      Roll  Pitch
```

### 3. Test MPU #2 (Throttle/Yaw)
Tilt MPU #2:
- Left/Right → Should see "Throttle" change (0-100)
- Forward/Back → Should see "Yaw" value change

Serial output:
```
DUAL:0.0,0.0,65.3,-12.1,...
             ^^^^  ^^^^^
             Thr   Yaw
```

---

## Physical Mounting Ideas

### Two-Handed Controller
```
    LEFT HAND               RIGHT HAND
┌─────────────────┐   ┌─────────────────┐
│    MPU #2       │   │     MPU #1      │
│  (Throttle/Yaw) │   │  (Roll/Pitch)   │
├─────────────────┤   ├─────────────────┤
│ Tilt L/R: Throt │   │ Tilt L/R: Roll  │
│ Tilt F/B: Yaw   │   │ Tilt F/B: Pitch │
└─────────────────┘   └─────────────────┘
         ▲                     ▲
         └──────┬──────────────┘
                │
           ESP32 Unit
```

---

## Troubleshooting

### ❌ "Failed to find MPU #1 at 0x68"
- Check MPU #1's AD0 pin is connected to **GND**
- Verify VCC/GND/SDA/SCL connections
- Upload `i2c_scanner.ino` to diagnose

### ❌ "Failed to find MPU #2 at 0x69"
- Check MPU #2's AD0 pin is connected to **3.3V**
- Make sure it's not shorted to GND
- Upload `i2c_scanner.ino` to diagnose

### ❌ Sensors detected but erratic readings
- Keep sensors still during calibration
- Reduce I2C speed: Change `Wire.setClock(400000)` to `Wire.setClock(100000)`
- Add 4.7kΩ pull-up resistors on SDA/SCL if wires are long

### ❌ ESP-NOW not working
- Update receiver MAC address in code
- Make sure receiver ESP is running
- Check both ESPs are on same WiFi channel

---

## Data Structure

New struct size: **52 bytes** (increased from 40 bytes)

```cpp
typedef struct {
    float roll_stick;      // 4 bytes - MPU1 X-axis
    float pitch_stick;     // 4 bytes - MPU1 Y-axis
    float throttle;        // 4 bytes - MPU2 X-axis
    float yaw_stick;       // 4 bytes - MPU2 Y-axis
    float mpu1_ax, ay, az; // 12 bytes
    float mpu1_gx, gy, gz; // 12 bytes
    float mpu2_ax, ay, az; // 12 bytes
    float mpu2_gx, gy, gz; // 12 bytes
} DualControlData;         // Total: 52 bytes
```

---

## Files Overview

| File | Purpose |
|------|---------|
| `esp32_combined.ino` | Main transmitter code with dual MPU support |
| `esp_receiver.ino` | Receiver code (updated for dual MPU data) |
| `esp_transmitter.ino` | PC bridge transmitter (updated for dual MPU) |
| `i2c_scanner.ino` | Diagnostic tool to verify sensor wiring |
| `DUAL_MPU_WIRING.md` | Complete wiring guide with diagrams |

---

## Control Value Ranges

| Control | Range | Center | Units |
|---------|-------|--------|-------|
| Roll | -100 to +100 | 0 | Arbitrary |
| Pitch | -100 to +100 | 0 | Arbitrary |
| Throttle | 0 to 100 | 50 | Percentage |
| Yaw | -100 to +100 | 0 | Arbitrary |

Values are automatically constrained to these ranges.

---

## Next Steps

1. ✅ Wire both MPU6050 sensors
2. ✅ Upload `i2c_scanner.ino` to verify
3. ✅ Upload `esp32_combined.ino`
4. ✅ Calibrate (keep still during startup)
5. ✅ Test each control axis
6. ✅ Adjust sensitivity if needed
7. ✅ Mount in your controller
8. ✅ Test with receiver/drone

---

## Quick Reference

**I2C Addresses:**
- 0x68 = MPU #1 (AD0 LOW)
- 0x69 = MPU #2 (AD0 HIGH)

**ESP32 Pins:**
- GPIO21 = SDA
- GPIO22 = SCL

**Calibration:**
- Automatic on startup
- Keep both sensors still for 2 seconds

**Data Rate:**
- 50Hz (20ms update interval)
- Reduced from 100Hz to handle dual sensors

---

Ready to test! 🎮
