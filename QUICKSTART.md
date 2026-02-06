# Quick Start Guide - ESP-NOW Wireless Setup

## What You Need

### Hardware
- **2 Laptops** (one transmitter, one receiver)
- **2 ESP32 boards** (ESP8266 also supported)
- **1 MPU6050** sensor module
- **USB cables** for ESP32s

### Software
- Arduino IDE with ESP32 board support
- Python 3.7+ with required packages (see README.md)

## Step-by-Step Setup

### Step 1: Prepare Receiver ESP32 (Do This First!)

1. **Upload Receiver Code**:
   - Open `esp_receiver.ino` in Arduino IDE
   - Select your ESP32 board and COM port
   - Upload the sketch

2. **Get MAC Address**:
   - Open Serial Monitor (115200 baud)
   - Copy the MAC address shown (e.g., `24:0A:C4:12:34:56`)
   - Write it down - you'll need it for the transmitter!

### Step 2: Prepare Transmitter ESP32

1. **Configure MAC Address**:
   - Open `esp_transmitter.ino` in Arduino IDE
   - Find line: `uint8_t receiverMAC[] = {0xFF, 0xFF, ...};`
   - Replace with receiver's MAC address:
     ```cpp
     // Example: If receiver MAC is 24:0A:C4:12:34:56
     uint8_t receiverMAC[] = {0x24, 0x0A, 0xC4, 0x12, 0x34, 0x56};
     ```

2. **Upload Transmitter Code**:
   - Select your ESP32 board and COM port
   - Upload the sketch

3. **Connect MPU6050** (if not using combined sketch):
   - VCC → 3.3V
   - GND → GND
   - SDA → GPIO 21 (or SDA pin)
   - SCL → GPIO 22 (or SCL pin)

### Step 3: Setup Receiver Laptop

1. **Connect Receiver ESP32** to Laptop 2 via USB

2. **Start Python App**:
   ```bash
   cd c:\Repos\dashboard
   python mpu6050_viewer.py
   ```

3. **Configure Receiver**:
   - Select **Receiver** radio button
   - Select COM port for receiver ESP32
   - Click **Connect**
   - Status should show "Connected (Receiver)"

### Step 4: Setup Transmitter Laptop

1. **Connect**:
   - Connect MPU6050 Arduino to Laptop 1 via USB
   - Connect Transmitter ESP32 to Laptop 1 via USB (if separate)

2. **Start Python App**:
   ```bash
   cd c:\Repos\dashboard
   python mpu6050_viewer.py
   ```

3. **Configure Transmitter**:
   - Select **Transmitter** radio button
   - Select COM port for MPU6050 device
   - Click **Connect**
   - Status should show "Connected (Transmitter)"

### Step 5: Test the System

1. **Verify Connection**:
   - Tilt the MPU6050 sensor on transmitter side
   - Watch the 3D visualization on transmitter laptop
   - The receiver laptop should mirror the exact same display!
   - Both joysticks should move identically
   - All sensor values should match

2. **Check Data Flow**:
   - Transmitter shows: sensor data → joystick movement → MAVLink values
   - Receiver shows: same joystick positions and MAVLink values
   - 3D visualization should be identical on both sides

## Troubleshooting

### Receiver Shows No Data
- Check ESP32 serial monitor - is it receiving ESP-NOW packets?
- Verify MAC address in transmitter code matches receiver
- Ensure both ESPs are powered on
- Try moving ESPs closer together

### Transmitter Not Sending
- Check MPU6050 connections
- Verify Python app is reading sensor data
- Check transmitter ESP serial monitor for "Data sent" messages
- Try clicking "Zero/Level" button

### Data Mismatch Between Sides
- Both apps must be running same version
- Verify data format matches (40 bytes)
- Check for serial communication errors
- Restart both Python apps

### Python App Won't Connect
- Close Arduino IDE Serial Monitor
- Check correct COM port selected
- Try different baud rate
- Verify USB cable is data-capable (not just power)

## Using Combined ESP32+MPU6050

If using `esp32_combined.ino` (single ESP32 with MPU6050):

1. **Hardware**: Connect MPU6050 directly to ESP32
2. **Upload**: Use `esp32_combined.ino` instead of separate sketches
3. **Configuration**: Set `SEND_TO_PC` and `SEND_TO_ESPNOW` flags as needed
4. **Python**: Connect to this ESP32's COM port in transmitter mode

## Testing Without ESP-NOW

You can test the system with one laptop:

1. Connect MPU6050 to Arduino
2. Select **Transmitter** mode
3. Connect and verify sensor data displays correctly
4. Check that joysticks respond to sensor tilt

## Common Issues

**"Port already in use"**
- Close Arduino Serial Monitor
- Close any other serial terminal programs
- Restart Python app

**"No ports available"**
- Check USB cable
- Verify ESP32 drivers installed
- Try different USB port

**Joysticks don't move in IMU mode**
- Check "Control Mode" is set to "IMU Controlled"
- Verify sensor data is updating (check numeric displays)
- Try increasing "Max Angle" setting
- Adjust deadzone settings

**Throttle not responding**
- Enable "Gesture Throttle" checkbox
- Rotate sensor around Z-axis (yaw)
- Increase "Throttle Sens" slider
- Check "Yaw Deadzone" isn't too high

## Next Steps

Once everything works:
- Experiment with control settings (deadzone, sensitivity)
- Try different max angle values
- Test manual vs IMU control modes
- Adjust invert settings for your preference
- Use "Zero/Level" to compensate for mounting angles

For more details, see the main [README.md](README.md).
