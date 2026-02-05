# MPU6050 Orientation Viewer

A real-time 3D visualization application for MPU6050 IMU sensor data using PySide6 and OpenGL.

## Features

- **COM Port Selection**: Choose from available serial ports
- **Real-time Data Display**: Shows accelerometer and gyroscope values
- **Sensor Fusion**: Uses complementary filter to calculate orientation (roll, pitch, yaw)
- **3D Visualization**: Interactive OpenGL rendering of sensor orientation
- **Animated Updates**: Smooth real-time orientation updates

## Requirements

- Python 3.7+
- PySide6
- pyserial
- numpy
- PyOpenGL
- PyOpenGL-accelerate

## Installation

```bash
pip install PySide6 pyserial numpy PyOpenGL PyOpenGL-accelerate
```

## Usage

1. Connect your MPU6050 to a COM port
2. Ensure the device sends data in the format: `ax,ay,az,gx,gy,gz` (comma-separated values)
3. Run the application:

```bash
python mpu6050_viewer.py
```

4. Select the appropriate COM port from the dropdown
5. Click "Connect" to start receiving data
6. The 3D visualization will update in real-time showing the sensor orientation

## Data Format

The application expects serial data in the following format:
```
ax,ay,az,gx,gy,gz
```

Where:
- `ax, ay, az`: Accelerometer data (m/s² or g)
- `gx, gy, gz`: Gyroscope data (degrees/second)

Example: `0.12,-0.05,9.81,1.2,-0.8,0.3`

## Controls

- **Refresh**: Reload available COM ports
- **Connect/Disconnect**: Toggle connection to the selected port
- **3D View**: The visualization shows:
  - Blue board representing the MPU6050
  - Red axis: X
  - Green axis: Y
  - Blue axis: Z

## Algorithm

The application uses a complementary filter for sensor fusion:
- Combines accelerometer data (for absolute orientation) with gyroscope data (for rotation rates)
- Alpha value of 0.98 balances between the two sensors
- Provides stable and responsive orientation tracking

## Troubleshooting

- **No ports available**: Check if your device is properly connected
- **Connection failed**: Verify the correct baud rate (default: 115200)
- **No data received**: Ensure your MPU6050 is sending data in the correct format
