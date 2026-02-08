# Python Viewer - Dual MPU6050 Update

## What's New

The Python viewer (`mpu6050_viewer.py`) has been updated to support the dual MPU6050 system with **automatic detection** and **backward compatibility**!

---

## ✨ Key Features

### 🔄 Automatic Mode Detection
The application automatically detects whether you're using:
- **Single MPU6050** (legacy mode)
- **Dual MPU6050** (new mode)

No configuration needed - it just works!

### 📊 Dual MPU Data Display

When dual MPU mode is detected, the UI updates to show:

1. **Control Values** (new row in sensor data):
   - Roll: -100 to +100
   - Pitch: -100 to +100
   - Throttle: 0 to 100
   - Yaw: -100 to +100

2. **MPU Sensor Data**:
   - MPU1 gyro data (Roll & Pitch control)
   - MPU2 gyro data (Throttle & Yaw control)

3. **Mode Indicator**:
   - Window title shows "- Dual MPU" when detected
   - MAVLink display shows "🎮 DUAL MPU MODE"
   - Status bar indicates dual MPU active

---

## 📡 Supported Data Formats

### Transmitter Mode

#### Single MPU (Legacy):
```
ax,ay,az,gx,gy,gz
```

#### Dual MPU (New):
```
DUAL:roll,pitch,throttle,yaw,mpu1_gx,mpu1_gy,mpu1_gz,mpu2_gx,mpu2_gy,mpu2_gz
```

### Receiver Mode

#### Single MPU CSV:
```
CSV: roll,pitch,throttle,yaw,ax,ay,az,gx,gy,gz
```

#### Dual MPU CSV:
```
DUAL_CSV: roll,pitch,throttle,yaw,mpu1_ax,mpu1_ay,mpu1_az,mpu1_gx,mpu1_gy,mpu1_gz,mpu2_ax,mpu2_ay,mpu2_az,mpu2_gx,mpu2_gy,mpu2_gz
```

#### ESP-NOW Packets:
- **Single MPU**: 40 bytes (10 floats)
- **Dual MPU**: 52 bytes (13 floats)

---

## 🎮 Control Display

### Single MPU Mode
Shows traditional IMU orientation:
- Roll angle (degrees)
- Pitch angle (degrees)
- Yaw angle (degrees)
- Stabilization feedback

### Dual MPU Mode
Shows hardware control values:
- Roll stick position (-100 to +100)
- Pitch stick position (-100 to +100)
- Throttle position (0 to 100)
- Yaw stick position (-100 to +100)
- MPU assignment info

---

## 📋 Usage

### With Dual MPU Hardware

1. **Wire dual MPU6050s** to ESP32 (see [DUAL_MPU_WIRING.md](DUAL_MPU_WIRING.md))
2. **Upload** `esp32_combined.ino` to transmitter
3. **Run** Python viewer:
   ```bash
   python mpu6050_viewer.py
   ```
4. **Select** Transmitter mode
5. **Connect** to COM port
6. **Verify** dual MPU mode activates automatically

Expected output:
```
Status: Connected - Dual MPU6050 Mode Active
Window Title: MPU6050 Orientation Viewer with ESP-NOW - Dual MPU
```

### Backward Compatibility

The viewer still works with single MPU setups:
- Old `mpu6050_reader.ino` sketches
- Legacy data formats
- Single sensor mode

No changes needed to existing projects!

---

## 🔍 Visual Indicators

### Window Title
- **Single MPU**: `MPU6050 Orientation Viewer with ESP-NOW`
- **Dual MPU**: `MPU6050 Orientation Viewer with ESP-NOW - Dual MPU`

### Status Bar
- **Single MPU**: `Status: Connected`
- **Dual MPU**: `Status: Connected - Dual MPU6050 Mode Active`

### MAVLink Display Header
- **Single MPU**: `=== MAVLink RC_CHANNELS_OVERRIDE (Single MPU Mode) ===`
- **Dual MPU**: `=== MAVLink RC_CHANNELS_OVERRIDE (🎮 DUAL MPU MODE) ===`

### Sensor Data Panel
**Single MPU**:
```
Accelerometer: AX: 0.00  AY: 0.00  AZ: 9.81
Gyroscope:     GX: 0.00  GY: 0.00  GZ: 0.00
Orientation:   Roll: 0.00°  Pitch: 0.00°  Yaw: 0.00°
```

**Dual MPU**:
```
Accelerometer: AX: 0.00  AY: 0.00  AZ: 9.81
Gyroscope:     MPU1-GX: 0.00  MPU1-GY: 0.00  MPU2-GX: 0.00 (T) / GY: 0.00 (Y)
Orientation:   Roll: 0.00°  Pitch: 0.00°  Yaw: 0.00°
Controls:      Roll: 0.0  Pitch: 0.0  Throttle: 50.0  Yaw: 0.0
```

---

## 🔧 Technical Details

### Data Parsing Logic

```python
# Automatic format detection
if line.startswith('DUAL:'):
    # Parse dual MPU format
    dual_mpu_mode = True
    # Process 10+ values
    
elif len(parts) == 6:
    # Parse single MPU format
    dual_mpu_mode = False
    # Process 6 values (ax,ay,az,gx,gy,gz)
```

### ESP-NOW Packet Size Detection

```python
if len(data_bytes) >= 52:
    # Dual MPU: 52 bytes (13 floats)
    dual_mpu_mode = True
    # Parse dual structure
    
elif len(data_bytes) >= 40:
    # Single MPU: 40 bytes (10 floats)
    dual_mpu_mode = False
    # Parse single structure
```

---

## 📦 New Signals

Added to `SerialReader` class:

```python
dual_data_received = Signal(dict)  # For dual MPU mode
```

Dictionary structure:
```python
{
    'roll_stick': float,      # -100 to +100
    'pitch_stick': float,     # -100 to +100
    'throttle': float,        # 0 to 100
    'yaw_stick': float,       # -100 to +100
    'mpu1_gx': float,         # MPU1 gyro X
    'mpu1_gy': float,         # MPU1 gyro Y
    'mpu1_gz': float,         # MPU1 gyro Z
    'mpu2_gx': float,         # MPU2 gyro X
    'mpu2_gy': float,         # MPU2 gyro Y
    'mpu2_gz': float          # MPU2 gyro Z
}
```

---

## 🎯 Control Value Scaling

### From Hardware to UI

Dual MPU hardware sends values in ranges:
- Roll/Pitch/Yaw: -100 to +100
- Throttle: 0 to 100

Python viewer normalizes to:
- Roll/Pitch/Yaw: -1.0 to +1.0
- Throttle: 0.0 to 1.0

### To MAVLink

Normalized values converted to:
- **PWM**: 1000-2000 µs
- **MANUAL_CONTROL**: -1000 to +1000 (roll/pitch/yaw), 0 to +1000 (throttle)

---

## 🐛 Troubleshooting

### Dual MPU Not Detected

**Symptoms**: Single MPU mode despite dual hardware

**Solutions**:
1. Verify ESP32 is sending "DUAL:" prefix
2. Check Serial Monitor for correct data format
3. Ensure both MPU6050s are initialized (check ESP serial output)
4. Upload latest `esp32_combined.ino`

### Control Values Not Updating

**Symptoms**: Controls stuck at zero

**Solutions**:
1. Verify calibration completed on ESP32
2. Check both MPUs are responding (tilt sensors)
3. Verify ESP-NOW transmission (check TX:OK messages)
4. Reconnect Python viewer

### Wrong Control Assignment

**Symptoms**: Roll affects throttle, etc.

**Solutions**:
1. Check MPU I2C addresses (0x68 and 0x69)
2. Verify AD0 pin connections
3. Use `i2c_scanner.ino` to confirm addresses
4. Check control mapping in ESP code

---

## ⚡ Performance

### Update Rates
- **Dual MPU**: 50 Hz (20ms per update)
- **Single MPU**: 100 Hz (10ms per update)

### Data Sizes
- **Dual MPU ESP-NOW**: 52 bytes per packet
- **Single MPU ESP-NOW**: 40 bytes per packet
- **Dual MPU Serial**: ~60 characters per line
- **Single MPU Serial**: ~40 characters per line

---

## 🔄 Migration Guide

### From Single to Dual MPU

**No Python code changes needed!** Just:

1. Update hardware (add second MPU)
2. Upload new ESP32 code
3. Run same Python viewer
4. Dual mode activates automatically

### Keeping Single MPU

**No changes needed!** The viewer continues to support:
- Legacy sketches
- Single sensor setups
- Existing projects

---

## 📝 Testing Checklist

### Single MPU Mode
- [ ] Connects to COM port
- [ ] Displays accelerometer data
- [ ] Displays gyroscope data
- [ ] Shows orientation angles
- [ ] 3D visualization works
- [ ] Virtual sticks respond
- [ ] MAVLink output correct

### Dual MPU Mode
- [ ] Detects dual MPU automatically
- [ ] Window title shows "- Dual MPU"
- [ ] Status shows dual mode active
- [ ] Control values display (roll/pitch/throttle/yaw)
- [ ] MPU1 and MPU2 data shown
- [ ] Virtual sticks match controls
- [ ] MAVLink shows dual MPU indicator
- [ ] ESP-NOW transmission works

---

## 📚 Related Documentation

- [DUAL_MPU_WIRING.md](DUAL_MPU_WIRING.md) - Hardware wiring guide
- [DUAL_MPU_QUICKSTART.md](DUAL_MPU_QUICKSTART.md) - Quick setup instructions
- [DUAL_MPU_SUMMARY.md](DUAL_MPU_SUMMARY.md) - Complete system reference
- [README.md](README.md) - Main project documentation

---

## 🎉 Benefits

### For Users
✅ Automatic detection - no configuration
✅ Backward compatible - old projects still work
✅ Clear visual feedback - know which mode is active
✅ Better control display - see actual control values
✅ Full 4-axis control from hardware

### For Developers
✅ Clean code structure
✅ Extensible design
✅ Robust parsing
✅ Error handling
✅ Future-proof architecture

---

**Version**: 2.0 - Dual MPU Support
**Date**: February 8, 2026
**Compatibility**: Python 3.7+, PySide6

Enjoy your dual MPU6050 control system! 🎮🚁
