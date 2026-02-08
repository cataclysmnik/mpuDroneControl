# Dual MPU6050 Wiring Guide for ESP32 Transmitter

## Overview
This guide shows how to connect TWO MPU6050 sensors to a single ESP32 for dual-axis control (Roll/Pitch on one, Throttle/Yaw on the other).

## Hardware Requirements
- 1x ESP32 Development Board
- 2x MPU6050 Sensor Modules (GY-521 or similar)
- Jumper wires
- Optional: 2x 4.7kΩ resistors (for I2C pull-ups if needed)

---

## I2C Address Configuration

The MPU6050 has a selectable I2C address controlled by the AD0 pin:
- **AD0 = LOW (GND)**: Address is **0x68** ← MPU #1
- **AD0 = HIGH (3.3V)**: Address is **0x69** ← MPU #2

This allows both sensors to work on the same I2C bus!

---

## Pin Connections

### ESP32 I2C Pins
```
GPIO21 → SDA (Data line - shared by both MPUs)
GPIO22 → SCL (Clock line - shared by both MPUs)
3.3V   → Power supply for both MPUs
GND    → Ground for both MPUs
```

### MPU6050 #1 (Roll & Pitch Control) - Address 0x68

```
┌─────────────────────────┐
│   MPU6050 Module #1     │
│  (Roll & Pitch Stick)   │
├─────────────────────────┤
│  VCC  →  ESP32 3.3V     │
│  GND  →  ESP32 GND      │
│  SCL  →  ESP32 GPIO22   │
│  SDA  →  ESP32 GPIO21   │
│  AD0  →  GND            │  ⚠️ CRITICAL: Connect to GND for 0x68
│  INT  →  (not connected)│
└─────────────────────────┘
```

### MPU6050 #2 (Throttle & Yaw Control) - Address 0x69

```
┌─────────────────────────┐
│   MPU6050 Module #2     │
│ (Throttle & Yaw Stick)  │
├─────────────────────────┤
│  VCC  →  ESP32 3.3V     │
│  GND  →  ESP32 GND      │
│  SCL  →  ESP32 GPIO22   │
│  SDA  →  ESP32 GPIO21   │
│  AD0  →  3.3V           │  ⚠️ CRITICAL: Connect to 3.3V for 0x69
│  INT  →  (not connected)│
└─────────────────────────┘
```

---

## Complete Wiring Diagram

```
                         ESP32
                    ┌─────────────┐
                    │             │
         ┌──────────│ 3.3V        │
         │      ┌───│ GND         │
         │      │┌──│ GPIO21(SDA) │
         │      ││┌─│ GPIO22(SCL) │
         │      │││ │             │
         │      │││ └─────────────┘
         │      │││
         │      │││
    ┌────┴──┐   │││   ┌────┴──┐
    │ MPU#1 │   │││   │ MPU#2 │
    │ 0x68  │   │││   │ 0x69  │
    ├───────┤   │││   ├───────┤
    │ VCC ←─┘   │││   │ VCC ←─┘
    │ GND ←─────┘││   │ GND ←─┘
    │ SCL ←──────┘│   │ SCL ←─┘
    │ SDA ←───────┘   │ SDA ←─┘
    │ AD0 ← GND       │ AD0 ← 3.3V
    │ INT (NC)        │ INT (NC)
    └───────┘         └───────┘
```

---

## Connection Table

| ESP32 Pin | MPU #1 Pin | MPU #2 Pin | Notes |
|-----------|------------|------------|-------|
| 3.3V      | VCC        | VCC        | Power both sensors |
| GND       | GND        | GND        | Common ground |
| GPIO21    | SDA        | SDA        | I2C Data (shared) |
| GPIO22    | SCL        | SCL        | I2C Clock (shared) |
| GND       | AD0        | -          | Sets MPU #1 to 0x68 |
| 3.3V      | -          | AD0        | Sets MPU #2 to 0x69 |

---

## Physical Mounting Recommendations

### Option 1: Dual-Grip Controller
```
Left Grip (MPU #2)          Right Grip (MPU #1)
┌─────────────┐             ┌─────────────┐
│   MPU #2    │             │   MPU #1    │
│  Throttle   │             │    Roll     │
│    Yaw      │             │   Pitch     │
└─────────────┘             └─────────────┘
     ▲                           ▲
     └───────────┬───────────────┘
                 │
            ESP32 Unit
```

### Option 2: Single Unit (Prototype)
Mount both MPUs on a single board for testing:
- MPU #1 at top (tilt for roll/pitch)
- MPU #2 at bottom (tilt for throttle/yaw)

---

## Control Mapping

### MPU #1 (0x68) - Roll & Pitch
- **Tilt Left/Right** (X-axis rotation) → **Roll Control**
- **Tilt Forward/Back** (Y-axis rotation) → **Pitch Control**

### MPU #2 (0x69) - Throttle & Yaw
- **Tilt Left/Right** (X-axis rotation) → **Throttle Control** (0-100%)
- **Tilt Forward/Back** (Y-axis rotation) → **Yaw Control**

---

## Troubleshooting

### Problem: "Failed to find MPU #1 at 0x68"
**Solutions:**
1. Check MPU #1's AD0 pin is connected to GND
2. Verify VCC and GND connections
3. Check SDA/SCL wiring
4. Run I2C scanner to detect addresses

### Problem: "Failed to find MPU #2 at 0x69"
**Solutions:**
1. Check MPU #2's AD0 pin is connected to 3.3V
2. Ensure not shorted to GND
3. Verify power supply can handle both sensors

### Problem: Both sensors detected but erratic readings
**Solutions:**
1. Add 4.7kΩ pull-up resistors on SDA and SCL lines
2. Keep wire lengths short (<15cm recommended)
3. Use shielded cable if interference persists
4. Check power supply stability (both sensors draw ~3-5mA each)

### Problem: I2C communication errors
**Solutions:**
1. Reduce I2C speed in code (400kHz → 100kHz)
2. Check for loose connections
3. Ensure common ground between all devices

---

## I2C Scanner Test Code

Use this to verify both sensors are detected:

```cpp
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  
  Serial.println("\nI2C Scanner");
  Serial.println("Scanning...");
  
  byte count = 0;
  for (byte i = 0; i < 128; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      if (i < 16) Serial.print("0");
      Serial.println(i, HEX);
      count++;
    }
  }
  
  Serial.print("Found ");
  Serial.print(count);
  Serial.println(" device(s)");
}

void loop() {}
```

Expected output:
```
Found device at 0x68  ← MPU #1
Found device at 0x69  ← MPU #2
Found 2 device(s)
```

---

## Optional: Pull-up Resistors

If you experience communication issues, add pull-up resistors:

```
        4.7kΩ           4.7kΩ
3.3V ────/\/\/\──┬──────/\/\/\──┬─── 3.3V
                 │               │
             GPIO21(SDA)    GPIO22(SCL)
                 │               │
              (To MPUs)      (To MPUs)
```

Most MPU6050 modules have built-in pull-ups, so external ones are usually not needed.

---

## Power Considerations

- Each MPU6050 draws approximately 3-5mA when active
- Total current: ~10mA (well within ESP32's 3.3V regulator capacity)
- For battery operation, consider the ESP32's power consumption (~80-240mA) as the primary concern

---

## Next Steps

1. Wire both MPUs according to this guide
2. Upload `esp32_combined.ino` to your ESP32
3. Open Serial Monitor (115200 baud)
4. Verify both sensors initialize correctly
5. Test by tilting each sensor and observing control output

---

## Summary Checklist

- [ ] MPU #1 AD0 connected to GND (address 0x68)
- [ ] MPU #2 AD0 connected to 3.3V (address 0x69)
- [ ] Both VCC pins connected to ESP32 3.3V
- [ ] Both GND pins connected to ESP32 GND
- [ ] Both SDA pins connected to GPIO21
- [ ] Both SCL pins connected to GPIO22
- [ ] Code uploaded and sensors detected
- [ ] Calibration completed with sensors stationary
- [ ] Control outputs respond to sensor movements

---

**Ready to fly! 🚁**
