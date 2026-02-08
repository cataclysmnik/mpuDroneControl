# MPU6050 Orientation Viewer with ESP-NOW Support

A real-time 3D visualization application for MPU6050 IMU sensor data with MAVLink command generation and ESP-NOW wireless transmission support using PySide6 and OpenGL.

## 🆕 Dual MPU6050 Support (Updated)

**NEW:** The system now supports **TWO MPU6050 sensors** for independent 4-axis control!

- **MPU #1 (0x68)**: Roll & Pitch control (X and Y axis rotation)
- **MPU #2 (0x69)**: Throttle & Yaw control (X and Y axis rotation)

This allows for intuitive two-handed controller operation where each hand controls different axes. See [DUAL_MPU_QUICKSTART.md](DUAL_MPU_QUICKSTART.md) for setup instructions.

### ✨ Python Viewer Updates
The Python application now **automatically detects** dual vs single MPU mode:
- ✅ **Auto-detection** - No configuration needed
- ✅ **Backward compatible** - Works with single MPU setups
- ✅ **Enhanced display** - Shows control values from both MPUs
- ✅ **Mode indicators** - Clear visual feedback for dual MPU mode

See [PYTHON_DUAL_MPU_UPDATE.md](PYTHON_DUAL_MPU_UPDATE.md) for details.

### Quick Start with Dual MPU
1. Wire both MPU6050s to ESP32 (different AD0 pins - see [DUAL_MPU_WIRING.md](DUAL_MPU_WIRING.md))
2. Upload `esp32_combined.ino` to transmitter ESP32
3. Upload `esp_receiver.ino` to receiver ESP32  
4. Run Python app - dual mode activates automatically!

---

## Features

- **Dual Mode Operation**: 
  - **Transmitter Mode**: Reads MPU6050 sensor, generates MAVLink commands, and sends via ESP-NOW
  - **Receiver Mode**: Receives ESP-NOW data from another ESP and displays exactly as on transmitter
- **COM Port Selection**: Choose from available serial ports
- **Real-time Data Display**: Shows accelerometer and gyroscope values
- **Sensor Fusion**: Uses complementary filter to calculate orientation (roll, pitch, yaw)
- **3D Visualization**: Interactive OpenGL rendering of sensor orientation
- **Virtual Flight Sticks**: Visual representation of MAVLink control inputs
- **IMU-Controlled Flight**: Control virtual sticks by tilting the MPU6050
- **MAVLink Command Generation**: Real-time RC_CHANNELS_OVERRIDE and MANUAL_CONTROL message display
- **ESP-NOW Wireless**: Transmit control data wirelessly between two laptops via ESP32

## Requirements

- Python 3.7+
- PySide6
- pyserial
- numpy
- PyOpenGL
- PyOpenGL-accelerate
- pymavlink

## Installation

```bash
pip install PySide6 pyserial numpy PyOpenGL PyOpenGL-accelerate pymavlink
```

## Hardware Setup

### Option 1: Single MPU6050 (Legacy)
- One Arduino/ESP32 with MPU6050
- Serial connection to PC

### Option 2: Dual MPU6050 (Recommended for Full Control)
- **ESP32** with **TWO MPU6050 sensors**
- MPU #1 (AD0=GND, address 0x68) for Roll & Pitch
- MPU #2 (AD0=3.3V, address 0x69) for Throttle & Yaw
- Upload `esp32_combined.ino` to ESP32
- See [DUAL_MPU_WIRING.md](DUAL_MPU_WIRING.md) for complete wiring guide

### Option 3: Wireless Two-Laptop Setup (ESP-NOW)

#### Transmitter Side:
1. **Laptop 1** running Python app in **Transmitter Mode**
2. **Arduino/ESP32 #1** with MPU6050 sensor
3. **ESP32 Transmitter** for ESP-NOW (can be same as #2)
   - Upload [esp_transmitter.ino](esp_transmitter.ino)

#### Receiver Side:
1. **Laptop 2** running Python app in **Receiver Mode**
2. **ESP32 Receiver** for ESP-NOW
   - Upload [esp_receiver.ino](esp_receiver.ino)

## Usage

### Transmitter Mode (Sending)

1. **Hardware Setup**:
   - Connect MPU6050 to Arduino/ESP32
   - Upload `mpu6050_reader.ino` if using separate Arduino
   - Upload `esp_transmitter.ino` to ESP32 transmitter
   - Update receiver MAC address in transmitter code

2. **Software**:
   - Run: `python mpu6050_viewer.py`
   - Select **Transmitter** mode
   - Select COM port for MPU6050 data
   - Click **Connect**
   - Tilt the sensor to control virtual sticks
   - Data is automatically sent to ESP for ESP-NOW transmission

### Receiver Mode (Receiving)

1. **Hardware Setup**:
   - Upload `esp_receiver.ino` to ESP32 receiver
   - Connect receiver ESP32 to laptop via USB
   - Run receiver ESP first to see its MAC address

2. **Software**:
   - Run: `python mpu6050_viewer.py`
   - Select **Receiver** mode
   - Select COM port for receiver ESP32
   - Click **Connect**
   - Display will mirror exactly what's shown on transmitter side

## Data Formats

### Single MPU Mode Input (from MPU6050):
```
ax,ay,az,gx,gy,gz
```

### Dual MPU Mode Output (from ESP32):
```
DUAL:roll,pitch,throttle,yaw,mpu1_gx,mpu1_gy,mpu1_gz,mpu2_gx,mpu2_gy,mpu2_gz
```

### ESP-NOW Packet Structure (Single MPU - 40 bytes):
```c
struct {
  float roll_stick;    // -1.0 to 1.0
  float pitch_stick;   // -1.0 to 1.0
  float throttle;      // 0.0 to 1.0
  float yaw_stick;     // -1.0 to 1.0
  float ax, ay, az;    // Accelerometer (g)
  float gx, gy, gz;    // Gyroscope (deg/s)
} // Total: 10 floats × 4 bytes = 40 bytes
```

### ESP-NOW Packet Structure (Dual MPU - 52 bytes):
```c
struct {
  float roll_stick;      // From MPU1 X-axis
  float pitch_stick;     // From MPU1 Y-axis
  float throttle;        // From MPU2 X-axis
  float yaw_stick;       // From MPU2 Y-axis
  float mpu1_ax, mpu1_ay, mpu1_az;  // MPU1 accelerometer
  float mpu1_gx, mpu1_gy, mpu1_gz;  // MPU1 gyroscope
  float mpu2_ax, mpu2_ay, mpu2_az;  // MPU2 accelerometer
  float mpu2_gx, mpu2_gy, mpu2_gz;  // MPU2 gyroscope
} // Total: 13 floats × 4 bytes = 52 bytes
```
  float ax, ay, az;    // Accelerometer (g)
  float gx, gy, gz;    // Gyroscope (deg/s)
} // Total: 10 floats × 4 bytes = 40 bytes
```

### Receiver Mode Output (from ESP):
```
ESPNOW:<80 hex characters representing 40 bytes>
```

## ESP-NOW Configuration

1. **Get Receiver MAC Address**:
   - Upload `esp_receiver.ino`
   - Open Serial Monitor
   - Copy the MAC address shown

2. **Configure Transmitter**:
   - Open `esp_transmitter.ino`
   - Update line: `uint8_t receiverMAC[] = {0xXX, 0xXX, ...};`
   - Upload to transmitter ESP

3. **Test Connection**:
   - Power both ESPs
   - Receiver should show "Waiting for ESP-NOW data..."
   - Transmitter should show "ESP-NOW Transmitter Ready"
   - Connect Python app on both sides

## Control Features

### Manual Control Mode
- Drag virtual joysticks with mouse
- Left stick: Throttle (Y) and Yaw (X)
- Right stick: Pitch (Y) and Roll (X)

### IMU Control Mode (Default)
- **Roll**: Tilt sensor left/right
- **Pitch**: Tilt sensor forward/backward
- **Throttle**: Rotate sensor around Z-axis (yaw rate gesture)
- **Yaw**: Currently fixed at center (can add manual control)

### Adjustable Parameters
- **Max Angle**: 15°, 30°, 45°, or 60° for roll/pitch scaling
- **Roll/Pitch Deadzone**: Ignore small angles (0-30°)
- **Throttle Sensitivity**: How quickly throttle responds (1x-20x)
- **Yaw Deadzone**: Ignore small rotation rates for throttle (0-50°/s)
- **Invert Roll/Pitch**: Flip control direction
- **Gesture Throttle**: Enable/disable rotation-based throttle control

### Zero/Level Function
- Click "Zero/Level" to calibrate current position as reference
- Resets throttle to zero
- Useful for compensating sensor mounting angle

## MAVLink Output

The application generates two MAVLink message formats:

### RC_CHANNELS_OVERRIDE
```
Channel 1 (Roll):     1000-2000 µs
Channel 2 (Pitch):    1000-2000 µs  
Channel 3 (Throttle): 1000-2000 µs
Channel 4 (Yaw):      1000-2000 µs
```

### MANUAL_CONTROL
```
X (Pitch):   -1000 to +1000
Y (Roll):    -1000 to +1000
Z (Throttle): 0 to +1000
R (Yaw):     -1000 to +1000
```

## Troubleshooting

### Connection Issues
- Close Arduino IDE Serial Monitor before connecting
- Ensure correct baud rate (default: 115200)
- Check COM port selection
- Verify sensor wiring (MPU6050: VCC, GND, SDA, SCL)

### ESP-NOW Issues
- Verify both ESPs are powered
- Check MAC address configuration in transmitter
- Ensure both ESPs use same data structure
- Both ESPs must be in WiFi Station mode
- Range: typically 100-200m line of sight

### No Data Display
- Check MPU6050 connections
- Verify data format matches expected CSV
- Try different baud rate
- Check Serial Monitor output from Arduino

### Receiver Not Updating
- Verify transmitter is connected and sending
- Check receiver ESP serial output
- Ensure Python app is in Receiver mode
- Check ESP-NOW pairing status

## Files Included

### Python Application
- `mpu6050_viewer.py` - **⭐ Main Python application (updated for dual MPU)**

### Arduino/ESP32 Sketches
- `mpu6050_reader.ino` - Basic MPU6050 reader (single sensor)
- `esp32_combined.ino` - **⭐ Dual MPU6050 transmitter (recommended)**
- `esp_transmitter.ino` - ESP32 sketch for PC bridge transmission
- `esp_receiver.ino` - ESP32 sketch for receiving via ESP-NOW

### Utilities
- `i2c_scanner.ino` - **🔍 Diagnostic tool for verifying dual MPU wiring**

### Documentation
- `README.md` - This file
- `DUAL_MPU_WIRING.md` - **📘 Complete wiring guide for dual MPU setup**
- `DUAL_MPU_QUICKSTART.md` - **🚀 Quick setup guide for dual MPU**
- `DUAL_MPU_SUMMARY.md` - **📋 Comprehensive dual MPU reference**
- `PYTHON_DUAL_MPU_UPDATE.md` - **🐍 Python viewer dual MPU update details**
- `CONNECTION_CARD.md` - **🔌 Printable quick reference card**
- `QUICKSTART.md` - Original quick start guide
- `IMPLEMENTATION.md` - Implementation details

## Architecture

```
TRANSMITTER SIDE:
┌─────────┐     ┌──────────┐     ┌─────────┐     ┌──────────┐
│ MPU6050 │────▶│ Arduino/ │────▶│ Python  │────▶│   ESP32  │
│ Sensor  │ I2C │   ESP    │Serial│   App   │Serial│Transmit. │
└─────────┘     └──────────┘     └─────────┘     └────┬─────┘
                                  (Transmitter)        │
                                                       │ESP-NOW
                                                       ▼
RECEIVER SIDE:                                   ┌──────────┐
┌─────────┐     ┌──────────┐                    │   ESP32  │
│ Python  │◀────│   ESP32  │◀───────────────────│ Receiver │
│   App   │Serial│ Receiver │      ESP-NOW      └──────────┘
└─────────┘     └──────────┘
(Receiver)
```

Where:
- `ax, ay, az`: Accelerometer data (g)
- `gx, gy, gz`: Gyroscope data (degrees/second)

Example: `0.12,-0.05,9.81,1.2,-0.8,0.3`

## 3D Visualization

- **3D View**: The visualization shows:
  - Blue board representing the MPU6050
  - Red axis: X
  - Green axis: Y
  - Blue axis: Z
  - Ground plane with grid for reference

## Algorithm

The application uses a complementary filter for sensor fusion:
- Combines accelerometer data (for absolute orientation) with gyroscope data (for rotation rates)
- Alpha value of 0.98 balances between the two sensors
- Provides stable and responsive orientation tracking

## License

This project is open source and available for educational purposes.
