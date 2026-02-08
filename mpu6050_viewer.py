import sys
import numpy as np
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QComboBox, QPushButton, QLabel, 
                               QGroupBox, QGridLayout, QTextEdit, QRadioButton, QButtonGroup)
from PySide6.QtCore import QTimer, Signal, QThread, Qt, QPointF
from PySide6.QtGui import QFont, QPainter, QColor, QPen, QBrush
import serial
import serial.tools.list_ports
import time
import struct
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2


class SensorFusion:
    """Complementary filter for sensor fusion of accelerometer and gyroscope data."""
    
    def __init__(self, alpha=0.98):
        self.alpha = alpha  # Complementary filter coefficient
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = time.time()
        
        # Calibration offsets for leveling
        self.roll_offset = 0.0
        self.pitch_offset = 0.0
        self.yaw_offset = 0.0
        
    def update(self, ax, ay, az, gx, gy, gz):
        """
        Update orientation using accelerometer and gyroscope data.
        
        Args:
            ax, ay, az: Accelerometer data (m/s^2 or g)
            gx, gy, gz: Gyroscope data (deg/s)
        """
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Normalize accelerometer vector (gravity direction)
        acc_magnitude = np.sqrt(ax**2 + ay**2 + az**2)
        if acc_magnitude > 0.1:  # Avoid division by zero
            ax_norm = ax / acc_magnitude
            ay_norm = ay / acc_magnitude
            az_norm = az / acc_magnitude
            
            # Calculate angles from accelerometer (gravity vector)
            # Roll: rotation around X axis
            accel_roll = np.arctan2(ay_norm, az_norm)
            # Pitch: rotation around Y axis
            accel_pitch = np.arctan2(-ax_norm, np.sqrt(ay_norm**2 + az_norm**2))
        else:
            accel_roll = self.roll
            accel_pitch = self.pitch
        
        # Convert gyroscope data from deg/s to rad/s
        gx_rad = np.radians(gx)
        gy_rad = np.radians(gy)
        gz_rad = np.radians(gz)
        
        # Integrate gyroscope data
        gyro_roll = self.roll + gx_rad * dt
        gyro_pitch = self.pitch + gy_rad * dt
        gyro_yaw = self.yaw + gz_rad * dt
        
        # Complementary filter
        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch
        self.yaw = gyro_yaw  # Yaw cannot be determined from accelerometer alone
        
        return self.roll, self.pitch, self.yaw
    
    def get_angles_degrees(self):
        """Return current angles in degrees."""
        return (np.degrees(self.roll), 
                np.degrees(self.pitch), 
                np.degrees(self.yaw))
    
    def get_calibrated_angles_degrees(self):
        """Return calibrated angles in degrees (with offsets applied)."""
        return (np.degrees(self.roll) - self.roll_offset, 
                np.degrees(self.pitch) - self.pitch_offset, 
                np.degrees(self.yaw) - self.yaw_offset)
    
    def zero_orientation(self):
        """Set current orientation as zero/level reference."""
        self.roll_offset = np.degrees(self.roll)
        self.pitch_offset = np.degrees(self.pitch)
        self.yaw_offset = np.degrees(self.yaw)


class SerialReader(QThread):
    """Thread for reading serial data from MPU6050 or ESP-NOW packets."""
    
    data_received = Signal(float, float, float, float, float, float)  # MPU1 data
    dual_data_received = Signal(dict)  # For dual MPU mode
    mavlink_received = Signal(dict)  # For receiver mode: receive MAVLink data
    error_occurred = Signal(str)
    
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.running = False
        self.mode = 'transmitter'  # 'transmitter' or 'receiver'
        self.dual_mpu_mode = False  # Automatically detected
    
    def set_mode(self, mode):
        """Set operation mode: 'transmitter' or 'receiver'."""
        self.mode = mode
        
    def connect(self, port, baudrate=115200):
        """Connect to serial port."""
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            
            # Try to open the port with proper settings
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            
            # Reset input/output buffers
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            time.sleep(2)  # Wait for connection to stabilize
            return True
        except serial.SerialException as e:
            if "PermissionError" in str(e) or "Access is denied" in str(e):
                self.error_occurred.emit(f"Port {port} is already in use. Please close Arduino IDE Serial Monitor or any other application using this port.")
            else:
                self.error_occurred.emit(f"Serial error: {str(e)}")
            return False
        except Exception as e:
            self.error_occurred.emit(f"Connection error: {type(e).__name__}: {str(e)}")
            return False
    
    def disconnect(self):
        """Disconnect from serial port."""
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
    
    def run(self):
        """Read data from serial port."""
        self.running = True
        
        while self.running:
            if not self.serial_port or not self.serial_port.is_open:
                time.sleep(0.1)
                continue
                
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    
                    # Skip empty lines
                    if not line:
                        continue
                    
                    if self.mode == 'transmitter':
                        # Transmitter mode: read sensor data
                        # Skip status messages from ESP
                        if line.startswith(('ESPNOW_', 'Data sent:', 'Error', 'TX:', 'Transmitter MAC:', 'ESP-NOW', 'Waiting', 'System', 'Initializing', 'Calibrat', 'MPU', '===')):
                            continue
                        
                        # Check for dual MPU format: DUAL:roll,pitch,throttle,yaw,mpu1_gx,mpu1_gy,mpu1_gz,mpu2_gx,mpu2_gy,mpu2_gz
                        if line.startswith('DUAL:'):
                            self.dual_mpu_mode = True
                            csv_data = line[5:].strip()
                            parts = csv_data.split(',')
                            if len(parts) >= 10:
                                try:
                                    roll, pitch, throttle, yaw = map(float, parts[0:4])
                                    mpu1_gx, mpu1_gy, mpu1_gz = map(float, parts[4:7])
                                    mpu2_gx, mpu2_gy, mpu2_gz = map(float, parts[7:10])
                                    
                                    dual_data = {
                                        'roll_stick': roll,
                                        'pitch_stick': pitch,
                                        'throttle': throttle,
                                        'yaw_stick': yaw,
                                        'mpu1_gx': mpu1_gx, 'mpu1_gy': mpu1_gy, 'mpu1_gz': mpu1_gz,
                                        'mpu2_gx': mpu2_gx, 'mpu2_gy': mpu2_gy, 'mpu2_gz': mpu2_gz
                                    }
                                    self.dual_data_received.emit(dual_data)
                                except ValueError:
                                    continue
                            continue
                        
                        # Legacy single MPU format: ax,ay,az,gx,gy,gz
                        parts = line.split(',')
                        if len(parts) == 6:
                            self.dual_mpu_mode = False
                            try:
                                ax, ay, az, gx, gy, gz = map(float, parts)
                                self.data_received.emit(ax, ay, az, gx, gy, gz)
                            except ValueError:
                                # Skip lines that can't be converted to floats
                                continue
                    
                    elif self.mode == 'receiver':
                        # Receiver mode: Parse ESP-NOW packet with MAVLink data
                        # Expected format from ESP: "ESPNOW:<hex_data>"
                        # Or CSV formats: "CSV: value,..." or "DUAL_CSV: value,..."
                        if line.startswith('ESPNOW:'):
                            hex_data = line[7:]
                            self._parse_espnow_packet(hex_data)
                        elif line.startswith('DUAL_CSV:'):
                            # Dual MPU CSV format
                            self.dual_mpu_mode = True
                            csv_data = line[9:].strip()
                            parts = csv_data.split(',')
                            if len(parts) >= 16:
                                try:
                                    roll, pitch, throttle, yaw = map(float, parts[0:4])
                                    mpu1_ax, mpu1_ay, mpu1_az = map(float, parts[4:7])
                                    mpu1_gx, mpu1_gy, mpu1_gz = map(float, parts[7:10])
                                    mpu2_ax, mpu2_ay, mpu2_az = map(float, parts[10:13])
                                    mpu2_gx, mpu2_gy, mpu2_gz = map(float, parts[13:16])
                                    
                                    mavlink_data = {
                                        'roll_stick': roll,
                                        'pitch_stick': pitch,
                                        'throttle': throttle,
                                        'yaw_stick': yaw,
                                        'dual_mpu': True,
                                        'mpu1_ax': mpu1_ax, 'mpu1_ay': mpu1_ay, 'mpu1_az': mpu1_az,
                                        'mpu1_gx': mpu1_gx, 'mpu1_gy': mpu1_gy, 'mpu1_gz': mpu1_gz,
                                        'mpu2_ax': mpu2_ax, 'mpu2_ay': mpu2_ay, 'mpu2_az': mpu2_az,
                                        'mpu2_gx': mpu2_gx, 'mpu2_gy': mpu2_gy, 'mpu2_gz': mpu2_gz
                                    }
                                    self.mavlink_received.emit(mavlink_data)
                                    # Emit MPU1 data for visualization
                                    self.data_received.emit(mpu1_ax, mpu1_ay, mpu1_az, mpu1_gx, mpu1_gy, mpu1_gz)
                                except ValueError:
                                    continue
                            continue
                        elif line.startswith('CSV:'):
                            # Legacy single MPU CSV format
                            self.dual_mpu_mode = False
                            csv_data = line[4:].strip()
                            parts = csv_data.split(',')
                            if len(parts) == 10:
                                try:
                                    roll_stick, pitch_stick, throttle, yaw_stick, ax, ay, az, gx, gy, gz = map(float, parts)
                                    mavlink_data = {
                                        'roll_stick': roll_stick,
                                        'pitch_stick': pitch_stick,
                                        'throttle': throttle,
                                        'yaw_stick': yaw_stick,
                                        'dual_mpu': False,
                                        'ax': ax, 'ay': ay, 'az': az,
                                        'gx': gx, 'gy': gy, 'gz': gz
                                    }
                                    self.mavlink_received.emit(mavlink_data)
                                    # Also emit raw sensor data for visualization
                                    self.data_received.emit(ax, ay, az, gx, gy, gz)
                                except ValueError:
                                    continue
                        # Skip status messages
                        elif line.startswith(('Receiver MAC', 'Copy this MAC', 'Error', 'ESP-NOW', 'Waiting', 'Received wrong')):
                            continue
                    
            except Exception as e:
                self.error_occurred.emit(f"Read error: {str(e)}")
                time.sleep(0.1)
    
    def _parse_espnow_packet(self, hex_data):
        """Parse ESP-NOW hex packet containing MAVLink data."""
        try:
            # Validate hex data is not empty and contains only valid hex characters
            if not hex_data or not hex_data.strip():
                return
            
            # Remove any whitespace
            hex_data = hex_data.strip()
            
            # Validate that it contains only valid hex characters
            if not all(c in '0123456789ABCDEFabcdef' for c in hex_data):
                self.error_occurred.emit(f"ESP-NOW parse error: non-hexadecimal characters in data: {hex_data[:50]}")
                return
            
            # Convert hex string to bytes
            data_bytes = bytes.fromhex(hex_data)
            
            # Check packet size to determine single or dual MPU
            if len(data_bytes) >= 52:  # Dual MPU: 52 bytes (13 floats)
                self.dual_mpu_mode = True
                # Unpack dual MPU data (little-endian floats)
                values = struct.unpack('<13f', data_bytes[:52])
                roll, pitch, throttle, yaw = values[0:4]
                mpu1_ax, mpu1_ay, mpu1_az = values[4:7]
                mpu1_gx, mpu1_gy, mpu1_gz = values[7:10]
                mpu2_ax, mpu2_ay, mpu2_az = values[10:13]
                mpu2_gx, mpu2_gy, mpu2_gz = values[13:16] if len(values) >= 16 else (0, 0, 0)
                
                mavlink_data = {
                    'roll_stick': roll,
                    'pitch_stick': pitch,
                    'throttle': throttle,
                    'yaw_stick': yaw,
                    'dual_mpu': True,
                    'mpu1_ax': mpu1_ax, 'mpu1_ay': mpu1_ay, 'mpu1_az': mpu1_az,
                    'mpu1_gx': mpu1_gx, 'mpu1_gy': mpu1_gy, 'mpu1_gz': mpu1_gz,
                    'mpu2_ax': mpu2_ax, 'mpu2_ay': mpu2_ay, 'mpu2_az': mpu2_az,
                    'mpu2_gx': mpu2_gx, 'mpu2_gy': mpu2_gy, 'mpu2_gz': mpu2_gz
                }
                self.mavlink_received.emit(mavlink_data)
                # Emit MPU1 data for visualization
                self.data_received.emit(mpu1_ax, mpu1_ay, mpu1_az, mpu1_gx, mpu1_gy, mpu1_gz)
                
            elif len(data_bytes) >= 40:  # Single MPU: 40 bytes (10 floats)
                self.dual_mpu_mode = False
                # Unpack single MPU data (little-endian floats)
                values = struct.unpack('<10f', data_bytes[:40])
                roll_stick, pitch_stick, throttle, yaw_stick, ax, ay, az, gx, gy, gz = values
                
                mavlink_data = {
                    'roll_stick': roll_stick,
                    'pitch_stick': pitch_stick,
                    'throttle': throttle,
                    'yaw_stick': yaw_stick,
                    'dual_mpu': False,
                    'ax': ax, 'ay': ay, 'az': az,
                    'gx': gx, 'gy': gy, 'gz': gz
                }
                self.mavlink_received.emit(mavlink_data)
                # Also emit raw sensor data for visualization
                self.data_received.emit(ax, ay, az, gx, gy, gz)
        except ValueError as e:
            # Catch specific hex parsing errors
            self.error_occurred.emit(f"ESP-NOW parse error: {str(e)}")
        except struct.error as e:
            # Catch struct unpacking errors (wrong data size)
            pass  # Silently skip - data size mismatch
        except Exception as e:
            self.error_occurred.emit(f"ESP-NOW parse error: {str(e)}")


class VirtualJoystick(QWidget):
    """Virtual joystick widget for flight control input."""
    
    position_changed = Signal(float, float)
    
    def __init__(self, label="Joystick", parent=None):
        super().__init__(parent)
        self.label = label
        self.setMinimumSize(200, 200)
        self.center = QPointF(100, 100)
        self.stick_pos = QPointF(100, 100)
        self.dragging = False
        self.max_distance = 80
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw background
        painter.fillRect(self.rect(), QColor(30, 30, 35))
        
        # Update center based on actual size
        self.center = QPointF(self.width() / 2, self.height() / 2)
        
        # Draw outer circle
        painter.setPen(QPen(QColor(100, 100, 100), 2))
        painter.setBrush(QBrush(QColor(50, 50, 55)))
        painter.drawEllipse(self.center, self.max_distance, self.max_distance)
        
        # Draw center crosshair
        painter.setPen(QPen(QColor(150, 150, 150), 1))
        painter.drawLine(int(self.center.x() - 10), int(self.center.y()), 
                        int(self.center.x() + 10), int(self.center.y()))
        painter.drawLine(int(self.center.x()), int(self.center.y() - 10), 
                        int(self.center.x()), int(self.center.y() + 10))
        
        # Draw stick
        painter.setPen(QPen(QColor(255, 100, 100), 2))
        painter.setBrush(QBrush(QColor(200, 80, 80)))
        painter.drawEllipse(self.stick_pos, 15, 15)
        
        # Draw label and values
        painter.setPen(QColor(200, 200, 200))
        painter.drawText(10, 20, self.label)
        
        x_val, y_val = self.get_normalized_position()
        painter.drawText(10, self.height() - 10, f"X: {x_val:.2f}, Y: {y_val:.2f}")
        
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.dragging = True
            self.update_stick_position(event.position())
            
    def mouseMoveEvent(self, event):
        if self.dragging:
            self.update_stick_position(event.position())
            
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.dragging = False
            # Return to center
            self.stick_pos = QPointF(self.center)
            self.update()
            self.position_changed.emit(0.0, 0.0)
            
    def update_stick_position(self, pos):
        # Calculate distance from center
        dx = pos.x() - self.center.x()
        dy = pos.y() - self.center.y()
        distance = np.sqrt(dx**2 + dy**2)
        
        # Limit to max_distance
        if distance > self.max_distance:
            angle = np.arctan2(dy, dx)
            dx = self.max_distance * np.cos(angle)
            dy = self.max_distance * np.sin(angle)
            
        self.stick_pos = QPointF(self.center.x() + dx, self.center.y() + dy)
        self.update()
        
        # Emit normalized position (-1 to 1)
        x_norm = dx / self.max_distance
        y_norm = -dy / self.max_distance  # Invert Y for intuitive control
        self.position_changed.emit(x_norm, y_norm)
        
    def get_normalized_position(self):
        """Get stick position normalized to -1.0 to 1.0."""
        dx = self.stick_pos.x() - self.center.x()
        dy = self.stick_pos.y() - self.center.y()
        x_norm = dx / self.max_distance
        y_norm = -dy / self.max_distance
        return x_norm, y_norm
    
    def set_position(self, x_norm, y_norm):
        """Set stick position programmatically from normalized values (-1 to 1)."""
        # Clamp values
        x_norm = max(-1.0, min(1.0, x_norm))
        y_norm = max(-1.0, min(1.0, y_norm))
        
        # Convert to pixel coordinates
        dx = x_norm * self.max_distance
        dy = -y_norm * self.max_distance  # Invert Y
        
        self.stick_pos = QPointF(self.center.x() + dx, self.center.y() + dy)
        self.update()
        
        # Don't emit signal to avoid circular updates


class MainWindow(QMainWindow):
    """Main application window."""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MPU6050 Orientation Viewer with ESP-NOW")
        self.setGeometry(100, 100, 1000, 700)
        
        # Initialize components
        self.sensor_fusion = SensorFusion()
        self.serial_reader = SerialReader()
        self.is_connected = False
        self.operation_mode = 'transmitter'  # 'transmitter' or 'receiver'
        self.dual_mpu_mode = False  # Detected from incoming data
        
        # MAVLink control values
        self.throttle = 0.0  # 0 to 1 - start at bottom
        self.yaw_stick = 0.0  # -1 to 1
        self.pitch_stick = 0.0  # -1 to 1
        self.roll_stick = 0.0  # -1 to 1
        
        # Vertical motion tracking for gesture throttle
        self.throttle_gesture = 0.0  # Start at bottom
        self.last_az_normalized = 0.0  # Last vertical acceleration
        
        # IMU orientation for stabilization
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_yaw = 0.0
        
        # Smoothing for dual MPU mode (exponential moving average)
        self.smoothing_alpha = 0.2  # Smoothing factor (0.0-1.0), lower = more smoothing
        self.smoothed_roll = 0.0
        self.smoothed_pitch = 0.0
        self.smoothed_throttle = 0.0
        self.smoothed_yaw = 0.0
        self.first_dual_data = True  # Flag to initialize smoothed values
        
        # Zero/level offsets for dual MPU mode
        self.dual_roll_offset = 0.0
        self.dual_pitch_offset = 0.0
        self.dual_throttle_offset = 0.0
        self.dual_yaw_offset = 0.0
        
        # Connect signals
        self.serial_reader.data_received.connect(self.on_data_received)
        self.serial_reader.dual_data_received.connect(self.on_dual_data_received)
        self.serial_reader.mavlink_received.connect(self.on_mavlink_received)
        self.serial_reader.error_occurred.connect(self.on_error)
        
        # Setup UI
        self.setup_ui()
        
        # Refresh COM ports
        self.refresh_ports()
        
    def setup_ui(self):
        """Setup the user interface."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout(central_widget)
        
        # Control panel
        control_group = QGroupBox("Connection Settings")
        control_layout = QHBoxLayout()
        
        # Mode selection
        control_layout.addWidget(QLabel("Mode:"))
        self.mode_button_group = QButtonGroup()
        self.transmitter_radio = QRadioButton("Transmitter")
        self.receiver_radio = QRadioButton("Receiver")
        self.transmitter_radio.setChecked(True)
        self.mode_button_group.addButton(self.transmitter_radio)
        self.mode_button_group.addButton(self.receiver_radio)
        self.transmitter_radio.toggled.connect(self.on_mode_changed)
        control_layout.addWidget(self.transmitter_radio)
        control_layout.addWidget(self.receiver_radio)
        control_layout.addSpacing(20)
        
        control_layout.addWidget(QLabel("COM Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        control_layout.addWidget(self.port_combo)
        
        control_layout.addWidget(QLabel("Baud Rate:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200", "230400", "250000"])
        self.baud_combo.setCurrentText("115200")
        self.baud_combo.setMinimumWidth(100)
        control_layout.addWidget(self.baud_combo)
        
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        control_layout.addWidget(self.refresh_btn)
        
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        control_layout.addWidget(self.connect_btn)
        
        self.zero_btn = QPushButton("Zero/Level")
        self.zero_btn.clicked.connect(self.zero_orientation)
        self.zero_btn.setEnabled(False)
        self.zero_btn.setToolTip("Set current position as level reference (place sensor flat first)")
        control_layout.addWidget(self.zero_btn)
        
        self.status_label = QLabel("Status: Disconnected - Select mode (Transmitter sends, Receiver gets ESP-NOW data)")
        self.status_label.setWordWrap(True)
        control_layout.addWidget(self.status_label)
        control_layout.addStretch()
        
        control_group.setLayout(control_layout)
        main_layout.addWidget(control_group)
        
        # Data display
        data_group = QGroupBox("Sensor Data")
        data_layout = QGridLayout()
        
        # Accelerometer
        data_layout.addWidget(QLabel("Accelerometer:"), 0, 0)
        self.ax_label = QLabel("AX: 0.00")
        self.ay_label = QLabel("AY: 0.00")
        self.az_label = QLabel("AZ: 0.00")
        data_layout.addWidget(self.ax_label, 0, 1)
        data_layout.addWidget(self.ay_label, 0, 2)
        data_layout.addWidget(self.az_label, 0, 3)
        
        # Gyroscope
        data_layout.addWidget(QLabel("Gyroscope:"), 1, 0)
        self.gx_label = QLabel("GX: 0.00")
        self.gy_label = QLabel("GY: 0.00")
        self.gz_label = QLabel("GZ: 0.00")
        data_layout.addWidget(self.gx_label, 1, 1)
        data_layout.addWidget(self.gy_label, 1, 2)
        data_layout.addWidget(self.gz_label, 1, 3)
        
        # Dual MPU indicator and controls display
        data_layout.addWidget(QLabel("Controls:"), 3, 0)
        self.control_roll_label = QLabel("Roll: 0.00")
        self.control_pitch_label = QLabel("Pitch: 0.00")
        self.control_throttle_label = QLabel("Throttle: 0.00")
        self.control_yaw_label = QLabel("Yaw: 0.00")
        data_layout.addWidget(self.control_roll_label, 3, 1)
        data_layout.addWidget(self.control_pitch_label, 3, 2)
        data_layout.addWidget(self.control_throttle_label, 3, 3)
        data_layout.addWidget(self.control_yaw_label, 4, 1)
        
        # Orientation
        data_layout.addWidget(QLabel("Orientation:"), 2, 0)
        self.roll_label = QLabel("Roll: 0.00°")
        self.pitch_label = QLabel("Pitch: 0.00°")
        self.yaw_label = QLabel("Yaw: 0.00°")
        data_layout.addWidget(self.roll_label, 2, 1)
        data_layout.addWidget(self.pitch_label, 2, 2)
        data_layout.addWidget(self.yaw_label, 2, 3)
        
        data_group.setLayout(data_layout)
        main_layout.addWidget(data_group)
        
        # Create horizontal layout for controls
        viz_control_layout = QHBoxLayout()
        
        # Flight control panel
        control_panel_layout = QVBoxLayout()
        
        # Joystick controls
        joystick_group = QGroupBox("Flight Control Sticks")
        joystick_layout = QVBoxLayout()
        
        # Control mode selection (only for transmitter)
        self.control_mode_layout = QHBoxLayout()
        self.control_mode_layout.addWidget(QLabel("Control Mode:"))
        self.control_mode_combo = QComboBox()
        self.control_mode_combo.addItems(["Manual (Mouse)", "IMU Controlled"])
        self.control_mode_combo.setCurrentIndex(1)  # Default to IMU controlled
        self.control_mode_combo.currentIndexChanged.connect(self.on_control_mode_changed)
        self.control_mode_layout.addWidget(self.control_mode_combo)
        
        self.control_mode_layout.addWidget(QLabel("Max Angle:"))
        self.max_angle_combo = QComboBox()
        self.max_angle_combo.addItems(["15°", "30°", "45°", "60°"])
        self.max_angle_combo.setCurrentIndex(1)  # Default to 30°
        self.control_mode_layout.addWidget(self.max_angle_combo)
        self.control_mode_layout.addStretch()
        
        joystick_layout.addLayout(self.control_mode_layout)
        
        # Deadzone controls in separate row (only for transmitter)
        self.deadzone_layout = QHBoxLayout()
        self.deadzone_layout.setSpacing(15)  # Add spacing between control groups
        
        self.deadzone_layout.addWidget(QLabel("Roll Deadzone:"))
        from PySide6.QtWidgets import QSlider
        self.roll_deadzone_slider = QSlider(Qt.Horizontal)
        self.roll_deadzone_slider.setMinimum(0)
        self.roll_deadzone_slider.setMaximum(30)  # 0-30 degrees
        self.roll_deadzone_slider.setValue(3)  # Default 3°
        self.roll_deadzone_slider.setMinimumWidth(120)
        self.roll_deadzone_slider.setMaximumWidth(150)
        self.roll_deadzone_slider.setToolTip("Angle below which roll is ignored")
        self.deadzone_layout.addWidget(self.roll_deadzone_slider)
        
        self.roll_deadzone_label = QLabel("3°")
        self.roll_deadzone_label.setMinimumWidth(35)
        self.roll_deadzone_slider.valueChanged.connect(lambda v: self.roll_deadzone_label.setText(f"{v}°"))
        self.deadzone_layout.addWidget(self.roll_deadzone_label)
        
        self.deadzone_layout.addSpacing(20)  # Extra space between groups
        
        self.deadzone_layout.addWidget(QLabel("Pitch Deadzone:"))
        self.pitch_deadzone_slider = QSlider(Qt.Horizontal)
        self.pitch_deadzone_slider.setMinimum(0)
        self.pitch_deadzone_slider.setMaximum(30)  # 0-30 degrees
        self.pitch_deadzone_slider.setValue(3)  # Default 3°
        self.pitch_deadzone_slider.setMinimumWidth(120)
        self.pitch_deadzone_slider.setMaximumWidth(150)
        self.pitch_deadzone_slider.setToolTip("Angle below which pitch is ignored")
        self.deadzone_layout.addWidget(self.pitch_deadzone_slider)
        
        self.pitch_deadzone_label = QLabel("3°")
        self.pitch_deadzone_label.setMinimumWidth(35)
        self.pitch_deadzone_slider.valueChanged.connect(lambda v: self.pitch_deadzone_label.setText(f"{v}°"))
        self.deadzone_layout.addWidget(self.pitch_deadzone_label)
        
        self.deadzone_layout.addSpacing(20)  # Extra space between groups
        
        self.deadzone_layout.addWidget(QLabel("Throttle Sens:"))
        self.throttle_sensitivity_slider = QSlider(Qt.Horizontal)
        self.throttle_sensitivity_slider.setMinimum(1)
        self.throttle_sensitivity_slider.setMaximum(20)  # 1-20x sensitivity
        self.throttle_sensitivity_slider.setValue(5)  # Default 5x
        self.throttle_sensitivity_slider.setMinimumWidth(120)
        self.throttle_sensitivity_slider.setMaximumWidth(150)
        self.throttle_sensitivity_slider.setToolTip("How quickly throttle responds to rotation")
        self.deadzone_layout.addWidget(self.throttle_sensitivity_slider)
        
        self.throttle_sensitivity_label = QLabel("5x")
        self.throttle_sensitivity_label.setMinimumWidth(35)
        self.throttle_sensitivity_slider.valueChanged.connect(lambda v: self.throttle_sensitivity_label.setText(f"{v}x"))
        self.deadzone_layout.addWidget(self.throttle_sensitivity_label)
        
        self.deadzone_layout.addSpacing(20)  # Extra space between groups
        
        self.deadzone_layout.addWidget(QLabel("Yaw Deadzone:"))
        self.yaw_deadzone_slider = QSlider(Qt.Horizontal)
        self.yaw_deadzone_slider.setMinimum(0)
        self.yaw_deadzone_slider.setMaximum(50)  # 0-50 deg/s
        self.yaw_deadzone_slider.setValue(5)  # Default 5 deg/s
        self.yaw_deadzone_slider.setMinimumWidth(120)
        self.yaw_deadzone_slider.setMaximumWidth(150)
        self.yaw_deadzone_slider.setToolTip("Ignore small yaw rotation rates (gz) for throttle")
        self.deadzone_layout.addWidget(self.yaw_deadzone_slider)
        
        self.yaw_deadzone_label = QLabel("5°/s")
        self.yaw_deadzone_label.setMinimumWidth(40)
        self.yaw_deadzone_slider.valueChanged.connect(lambda v: self.yaw_deadzone_label.setText(f"{v}°/s"))
        self.deadzone_layout.addWidget(self.yaw_deadzone_label)
        
        self.deadzone_layout.addStretch()
        
        joystick_layout.addLayout(self.deadzone_layout)
        
        # Invert controls (functional in both transmitter and receiver modes)
        self.invert_layout = QHBoxLayout()
        from PySide6.QtWidgets import QCheckBox
        self.invert_roll_check = QCheckBox("Invert Roll")
        self.invert_roll_check.setChecked(True)  # Inverted by default
        self.invert_layout.addWidget(self.invert_roll_check)
        
        self.invert_pitch_check = QCheckBox("Invert Pitch")
        self.invert_pitch_check.setChecked(True)  # Inverted by default
        self.invert_layout.addWidget(self.invert_pitch_check)
        
        self.gesture_throttle_check = QCheckBox("Gesture Throttle")
        self.gesture_throttle_check.setChecked(True)  # Enabled by default
        self.gesture_throttle_check.setToolTip("Control throttle by rotating sensor (gz - yaw rate)")
        self.invert_layout.addWidget(self.gesture_throttle_check)
        self.invert_layout.addStretch()
        
        joystick_layout.addLayout(self.invert_layout)
        
        # Joystick widgets
        sticks_layout = QHBoxLayout()
        
        # Throttle/Yaw stick (left)
        self.throttle_yaw_stick = VirtualJoystick("Throttle/Yaw")
        self.throttle_yaw_stick.position_changed.connect(self.on_throttle_yaw_changed)
        sticks_layout.addWidget(self.throttle_yaw_stick)
        
        # Pitch/Roll stick (right)
        self.pitch_roll_stick = VirtualJoystick("Pitch/Roll")
        self.pitch_roll_stick.position_changed.connect(self.on_pitch_roll_changed)
        sticks_layout.addWidget(self.pitch_roll_stick)
        
        joystick_layout.addLayout(sticks_layout)
        joystick_group.setLayout(joystick_layout)
        control_panel_layout.addWidget(joystick_group)
        
        # MAVLink command display
        mavlink_group = QGroupBox("MAVLink Commands")
        mavlink_layout = QVBoxLayout()
        
        self.mavlink_display = QTextEdit()
        self.mavlink_display.setReadOnly(True)
        self.mavlink_display.setMinimumHeight(300)
        self.mavlink_display.setFont(QFont("Courier", 9))
        mavlink_layout.addWidget(self.mavlink_display)
        
        mavlink_group.setLayout(mavlink_layout)
        control_panel_layout.addWidget(mavlink_group)
        
        viz_control_layout.addLayout(control_panel_layout, 1)
        main_layout.addLayout(viz_control_layout)
        
        # Set font for labels
        font = QFont()
        font.setPointSize(10)
        for label in [self.ax_label, self.ay_label, self.az_label,
                     self.gx_label, self.gy_label, self.gz_label,
                     self.roll_label, self.pitch_label, self.yaw_label]:
            label.setFont(font)
    def show_control_widgets(self, show):
        """Show or hide control widgets based on mode."""
        # Find all widgets in control layouts and set visibility
        for i in range(self.control_mode_layout.count()):
            widget = self.control_mode_layout.itemAt(i).widget()
            if widget:
                widget.setVisible(show)
                
        for i in range(self.deadzone_layout.count()):
            widget = self.deadzone_layout.itemAt(i).widget()
            if widget:
                widget.setVisible(show)
                
        # Note: Invert controls are now always visible since they work in both transmitter and receiver modes
        # Keeping this code for potential future use
        # for i in range(self.invert_layout.count()):
        #     widget = self.invert_layout.itemAt(i).widget()
        #     if widget:
        #         widget.setVisible(show)
            
    def refresh_ports(self):
        """Refresh the list of available COM ports."""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}", port.device)
            
        if self.port_combo.count() == 0:
            self.port_combo.addItem("No ports available", None)
    
    def on_mode_changed(self):
        """Handle mode change between transmitter and receiver."""
        if self.transmitter_radio.isChecked():
            self.operation_mode = 'transmitter'
            self.status_label.setText("Status: Transmitter Mode - Reads sensor and sends MAVLink commands")
            # Show control widgets
            self.show_control_widgets(True)
        else:
            self.operation_mode = 'receiver'
            self.status_label.setText("Status: Receiver Mode - Receives ESP-NOW data and displays")
            # Hide control widgets
            self.show_control_widgets(False)
        
        # Update serial reader mode
        self.serial_reader.set_mode(self.operation_mode)
        
        # Disable mode change when connected
        if self.is_connected:
            self.status_label.setText("Status: Cannot change mode while connected. Disconnect first.")
            # Revert selection
            if self.operation_mode == 'transmitter':
                self.receiver_radio.setChecked(True)
            else:
                self.transmitter_radio.setChecked(True)
            
    def toggle_connection(self):
        """Toggle connection to serial port."""
        if not self.is_connected:
            # Connect
            port = self.port_combo.currentData()
            if port is None:
                self.status_label.setText("Status: No port selected")
                return
            
            # Get baud rate
            try:
                baudrate = int(self.baud_combo.currentText())
            except ValueError:
                baudrate = 115200
                
            if self.serial_reader.connect(port, baudrate):
                self.serial_reader.start()
                self.is_connected = True
                self.connect_btn.setText("Disconnect")
                mode_text = "Transmitter" if self.operation_mode == 'transmitter' else "Receiver"
                self.status_label.setText(f"Status: Connected ({mode_text}) to {port} at {baudrate} baud")
                self.port_combo.setEnabled(False)
                self.baud_combo.setEnabled(False)
                self.refresh_btn.setEnabled(False)
                self.transmitter_radio.setEnabled(False)
                self.receiver_radio.setEnabled(False)
                self.zero_btn.setEnabled(True)
            else:
                self.status_label.setText("Status: Connection failed - Check console for details")
        else:
            # Disconnect
            self.serial_reader.disconnect()
            self.serial_reader.wait()
            self.is_connected = False
            self.connect_btn.setText("Connect")
            self.status_label.setText("Status: Disconnected")
            self.port_combo.setEnabled(True)
            self.baud_combo.setEnabled(True)
            self.refresh_btn.setEnabled(True)
            self.transmitter_radio.setEnabled(True)
            self.receiver_radio.setEnabled(True)
            self.zero_btn.setEnabled(False)
            
    def on_control_mode_changed(self, index):
        """Handle control mode change."""
        is_manual = (index == 0)
        # Enable/disable mouse interaction on joysticks
        self.throttle_yaw_stick.setEnabled(is_manual)
        self.pitch_roll_stick.setEnabled(is_manual)
        
        if not is_manual:
            self.status_label.setText("Status: IMU Control Mode - Sticks follow sensor orientation")
        else:
            self.status_label.setText("Status: Manual Mode - Drag sticks with mouse")
            
    def on_throttle_yaw_changed(self, x, y):
        """Handle throttle/yaw stick movement."""
        self.yaw_stick = x  # Left/right is yaw
        self.throttle = (y + 1.0) / 2.0  # Convert -1..1 to 0..1 for throttle
        self.update_mavlink_commands()
        
    def on_pitch_roll_changed(self, x, y):
        """Handle pitch/roll stick movement."""
        self.roll_stick = x  # Left/right is roll
        self.pitch_stick = y  # Up/down is pitch
        self.update_mavlink_commands()
        
    def update_mavlink_commands(self):
        """Generate and display MAVLink commands."""
        # Convert stick values to RC PWM values (1000-2000 microseconds)
        # Center is 1500, range is ±500
        def to_pwm(normalized_value, is_throttle=False):
            if is_throttle:
                # Throttle: 0-1 maps to 1000-2000
                return int(1000 + normalized_value * 1000)
            else:
                # Other channels: -1 to 1 maps to 1000-2000
                return int(1500 + normalized_value * 500)
        
        # Calculate stabilized values using IMU feedback
        # For demonstration: combining stick input with IMU orientation
        roll_pwm = to_pwm(self.roll_stick)
        pitch_pwm = to_pwm(self.pitch_stick)
        throttle_pwm = to_pwm(self.throttle, is_throttle=True)
        yaw_pwm = to_pwm(self.yaw_stick)
        
        # IMU attitude in degrees
        imu_roll_deg = self.imu_roll
        imu_pitch_deg = self.imu_pitch
        imu_yaw_deg = self.imu_yaw
        
        # Generate MAVLink RC_CHANNELS_OVERRIDE message format
        mode_indicator = "🎮 DUAL MPU MODE" if self.dual_mpu_mode else "Single MPU Mode"
        mavlink_text = f"=== MAVLink RC_CHANNELS_OVERRIDE ({mode_indicator}) ===\n"
        mavlink_text += f"Channel 1 (Roll):     {roll_pwm} µs\n"
        mavlink_text += f"Channel 2 (Pitch):    {pitch_pwm} µs\n"
        mavlink_text += f"Channel 3 (Throttle): {throttle_pwm} µs\n"
        mavlink_text += f"Channel 4 (Yaw):      {yaw_pwm} µs\n\n"
        
        mavlink_text += "=== MANUAL_CONTROL ===\n"
        mavlink_text += f"X (Pitch):   {int(self.pitch_stick * 1000):5d} (-1000 to +1000)\n"
        mavlink_text += f"Y (Roll):    {int(self.roll_stick * 1000):5d} (-1000 to +1000)\n"
        mavlink_text += f"Z (Throttle): {int(self.throttle * 1000):5d} (0 to +1000)\n"
        mavlink_text += f"R (Yaw):     {int(self.yaw_stick * 1000):5d} (-1000 to +1000)\n\n"
        
        if self.dual_mpu_mode:
            mavlink_text += "=== Dual MPU6050 Control Mapping ===\n"
            mavlink_text += "MPU #1 (0x68): Roll & Pitch\n"
            mavlink_text += "MPU #2 (0x69): Throttle & Yaw\n\n"
        else:
            mavlink_text += "=== IMU Attitude (from MPU6050) ===\n"
            mavlink_text += f"Roll:  {imu_roll_deg:7.2f}°\n"
            mavlink_text += f"Pitch: {imu_pitch_deg:7.2f}°\n"
            mavlink_text += f"Yaw:   {imu_yaw_deg:7.2f}°\n\n"
            
            # Calculate error for stabilization feedback
            roll_error = self.roll_stick * 30 - imu_roll_deg  # Target ±30° max
            pitch_error = self.pitch_stick * 30 - imu_pitch_deg
            
            mavlink_text += "=== Stabilization Feedback ===\n"
            mavlink_text += f"Roll Error:  {roll_error:7.2f}° (stick target - actual)\n"
            mavlink_text += f"Pitch Error: {pitch_error:7.2f}° (stick target - actual)\n"
        
        self.mavlink_display.setText(mavlink_text)
    
    def on_data_received(self, ax, ay, az, gx, gy, gz):
        """Handle received sensor data."""
        # Update sensor data labels
        self.ax_label.setText(f"AX: {ax:.2f}")
        self.ay_label.setText(f"AY: {ay:.2f}")
        self.az_label.setText(f"AZ: {az:.2f}")
        self.gx_label.setText(f"GX: {gx:.2f}")
        self.gy_label.setText(f"GY: {gy:.2f}")
        self.gz_label.setText(f"GZ: {gz:.2f}")
        
        # Calculate orientation
        roll, pitch, yaw = self.sensor_fusion.update(ax, ay, az, gx, gy, gz)
        roll_deg, pitch_deg, yaw_deg = self.sensor_fusion.get_calibrated_angles_degrees()
        
        # Store IMU values for MAVLink feedback
        self.imu_roll = roll_deg
        self.imu_pitch = pitch_deg
        self.imu_yaw = yaw_deg
        
        # Update orientation labels (showing calibrated angles)
        self.roll_label.setText(f"Roll: {roll_deg:.2f}°")
        self.pitch_label.setText(f"Pitch: {pitch_deg:.2f}°")
        self.yaw_label.setText(f"Yaw: {yaw_deg:.2f}°")
        
        # ONLY process controls in TRANSMITTER mode
        if self.operation_mode == 'transmitter' and self.control_mode_combo.currentIndex() == 1:  # IMU Controlled
            # Get max angle setting
            max_angle_text = self.max_angle_combo.currentText()
            max_angle = float(max_angle_text.replace('°', ''))
            
            # Map angles to stick positions (-1 to 1)
            # Apply invert settings
            roll_multiplier = -1 if self.invert_roll_check.isChecked() else 1
            pitch_multiplier = -1 if self.invert_pitch_check.isChecked() else 1
            
            # Get deadzone values in degrees
            roll_deadzone = self.roll_deadzone_slider.value()
            pitch_deadzone = self.pitch_deadzone_slider.value()
            
            # Apply deadzone to angles before normalization
            roll_deg_filtered = roll_deg
            pitch_deg_filtered = pitch_deg
            
            if abs(roll_deg) < roll_deadzone:
                roll_deg_filtered = 0.0
            else:
                # Remove deadzone and rescale
                sign = 1 if roll_deg > 0 else -1
                roll_deg_filtered = sign * (abs(roll_deg) - roll_deadzone)
            
            if abs(pitch_deg) < pitch_deadzone:
                pitch_deg_filtered = 0.0
            else:
                # Remove deadzone and rescale
                sign = 1 if pitch_deg > 0 else -1
                pitch_deg_filtered = sign * (abs(pitch_deg) - pitch_deadzone)
            
            # Roll: negative roll (left tilt) = negative stick (left)
            # Pitch: positive pitch (forward tilt) = positive stick (up)
            roll_stick_pos = roll_multiplier * (-roll_deg_filtered / max_angle)  # Base invert for intuitive control
            pitch_stick_pos = pitch_multiplier * (pitch_deg_filtered / max_angle)
            
            # Clamp to -1 to 1
            roll_stick_pos = max(-1.0, min(1.0, roll_stick_pos))
            pitch_stick_pos = max(-1.0, min(1.0, pitch_stick_pos))
            
            # Update pitch/roll stick position
            self.pitch_roll_stick.set_position(roll_stick_pos, pitch_stick_pos)
            
            # Update internal values for MAVLink generation
            self.roll_stick = roll_stick_pos
            self.pitch_stick = pitch_stick_pos
            
            # For throttle/yaw stick control
            # Yaw stick: keep at center (no yaw control)
            yaw_stick_pos = 0.0
            
            # Throttle control with gz (yaw rotation rate) gesture
            if self.gesture_throttle_check.isChecked():
                # Use gz (gyroscope Z-axis) to control throttle
                # Positive gz (clockwise rotation) = increase throttle
                # Negative gz (counter-clockwise) = decrease throttle
                
                # Get sensitivity multiplier and yaw deadzone
                sensitivity = self.throttle_sensitivity_slider.value()
                yaw_deadzone = self.yaw_deadzone_slider.value()
                
                # Apply deadzone to gz
                gz_filtered = gz
                if abs(gz) < yaw_deadzone:
                    gz_filtered = 0.0
                else:
                    # Remove deadzone and rescale
                    sign = 1 if gz > 0 else -1
                    gz_filtered = sign * (abs(gz) - yaw_deadzone)
                
                # Scale rotation rate to throttle change
                # Base: ±50 deg/s = moderate throttle change
                throttle_change = (gz_filtered / 500.0) * sensitivity
                
                # Update throttle accumulator - stays where you leave it
                self.throttle_gesture += throttle_change
                
                # Apply to actual throttle with clamping
                self.throttle_gesture = max(0.0, min(1.0, self.throttle_gesture))
                self.throttle = self.throttle_gesture
                
                # Convert to stick position (-1 to 1 for display, but bottom is -1)
                throttle_pos = (self.throttle * 2.0) - 1.0  # 0 to 1 → -1 to 1
            else:
                # No gesture control - keep at bottom
                throttle_pos = -1.0  # Bottom position
                self.throttle = 0.0
            
            self.throttle_yaw_stick.set_position(yaw_stick_pos, throttle_pos)
            self.yaw_stick = yaw_stick_pos
        
            # Update MAVLink commands with latest IMU data
            self.update_mavlink_commands()
            
            # In transmitter mode, send data to ESP for ESP-NOW transmission
            if self.is_connected:
                self.send_espnow_data(ax, ay, az, gx, gy, gz)
        elif self.operation_mode == 'receiver':
            # In receiver mode, just update display - controls are set by on_mavlink_received
            self.update_mavlink_commands()
        
    def send_espnow_data(self, ax, ay, az, gx, gy, gz):
        """Send MAVLink control data to ESP for ESP-NOW transmission."""
        try:
            if self.serial_reader.serial_port and self.serial_reader.serial_port.is_open:
                # Dual MPU mode doesn't need PC transmission - controls are from sensors
                if self.dual_mpu_mode:
                    return
                    
                # Pack data: 4 stick values + 6 IMU values = 10 floats (single MPU)
                data = struct.pack('<10f', 
                    self.roll_stick, self.pitch_stick, self.throttle, self.yaw_stick,
                    ax, ay, az, gx, gy, gz)
                
                # Send as hex string with header
                hex_data = data.hex().upper()
                message = f"SEND:{hex_data}\n"
                self.serial_reader.serial_port.write(message.encode('utf-8'))
        except Exception as e:
            # Silently fail to avoid flooding error messages
            pass
        
    def zero_orientation(self):
        """Zero/level the current orientation."""
        if self.dual_mpu_mode:
            # In dual MPU mode, store current smoothed values as offsets
            self.dual_roll_offset = self.smoothed_roll
            self.dual_pitch_offset = self.smoothed_pitch
            self.dual_throttle_offset = self.smoothed_throttle
            self.dual_yaw_offset = self.smoothed_yaw
            self.status_label.setText("Status: Dual MPU zero point set")
        else:
            # In single MPU mode, use sensor fusion zeroing
            self.sensor_fusion.zero_orientation()
            # Reset throttle to bottom
            self.throttle = 0.0
            self.throttle_gesture = 0.0
            self.yaw_stick = 0.0
            self.status_label.setText("Status: Orientation and throttle zeroed")
    
    def on_dual_data_received(self, dual_data):
        """Handle dual MPU6050 data in transmitter mode."""
        self.dual_mpu_mode = True
        
        # Update window title to show dual MPU mode
        if not self.windowTitle().endswith("- Dual MPU"):
            self.setWindowTitle(self.windowTitle() + " - Dual MPU")
            self.status_label.setText("Status: Connected - Dual MPU6050 Mode Active")
        
        # Apply exponential moving average smoothing to reduce noise
        raw_roll = dual_data['roll_stick'] / 100.0
        raw_pitch = dual_data['pitch_stick'] / 100.0
        raw_throttle = dual_data['throttle'] / 100.0
        raw_yaw = dual_data['yaw_stick'] / 100.0
        
        # Initialize on first data point
        if self.first_dual_data:
            self.smoothed_roll = raw_roll
            self.smoothed_pitch = raw_pitch
            self.smoothed_throttle = raw_throttle
            self.smoothed_yaw = raw_yaw
            self.first_dual_data = False
        else:
            # Apply smoothing: new_value = alpha * raw + (1 - alpha) * previous
            self.smoothed_roll = (self.smoothing_alpha * raw_roll) + ((1 - self.smoothing_alpha) * self.smoothed_roll)
            self.smoothed_pitch = (self.smoothing_alpha * raw_pitch) + ((1 - self.smoothing_alpha) * self.smoothed_pitch)
            self.smoothed_throttle = (self.smoothing_alpha * raw_throttle) + ((1 - self.smoothing_alpha) * self.smoothed_throttle)
            self.smoothed_yaw = (self.smoothing_alpha * raw_yaw) + ((1 - self.smoothing_alpha) * self.smoothed_yaw)
        
        # Apply zero/level offsets
        adjusted_roll = self.smoothed_roll - self.dual_roll_offset
        adjusted_pitch = self.smoothed_pitch - self.dual_pitch_offset
        adjusted_throttle = max(0.0, self.smoothed_throttle - self.dual_throttle_offset)  # Keep throttle >= 0
        adjusted_yaw = self.smoothed_yaw - self.dual_yaw_offset
        
        # Update from hardware controls with invert settings applied
        roll_multiplier = -1 if self.invert_roll_check.isChecked() else 1
        pitch_multiplier = -1 if self.invert_pitch_check.isChecked() else 1
        
        self.roll_stick = adjusted_roll * roll_multiplier
        self.pitch_stick = adjusted_pitch * pitch_multiplier
        self.throttle = adjusted_throttle
        self.yaw_stick = adjusted_yaw
        
        # Update control display labels (show adjusted values)
        self.control_roll_label.setText(f"Roll: {adjusted_roll * 100.0:.1f}")
        self.control_pitch_label.setText(f"Pitch: {adjusted_pitch * 100.0:.1f}")
        self.control_throttle_label.setText(f"Throttle: {adjusted_throttle * 100.0:.1f}")
        self.control_yaw_label.setText(f"Yaw: {adjusted_yaw * 100.0:.1f}")
        
        # Update joystick visualizations
        throttle_pos = (self.throttle * 2.0) - 1.0  # 0-1 to -1-1
        self.throttle_yaw_stick.set_position(self.yaw_stick, throttle_pos)
        self.pitch_roll_stick.set_position(self.roll_stick, self.pitch_stick)
        
        # Update MPU data display (show gyro from both MPUs)
        self.gx_label.setText(f"MPU1-GX: {dual_data['mpu1_gx']:.2f}")
        self.gy_label.setText(f"MPU1-GY: {dual_data['mpu1_gy']:.2f}")
        self.gz_label.setText(f"MPU2-GX: {dual_data['mpu2_gx']:.2f} (T) / GY: {dual_data['mpu2_gy']:.2f} (Y)")
        
        # Update MAVLink display
        self.update_mavlink_commands()
    
    def on_mavlink_received(self, mavlink_data):
        """Handle received MAVLink data from ESP-NOW (receiver mode)."""
        # Check if dual MPU mode
        is_dual = mavlink_data.get('dual_mpu', False)
        self.dual_mpu_mode = is_dual
        
        # Extract control values from received data
        self.roll_stick = mavlink_data['roll_stick']
        self.pitch_stick = mavlink_data['pitch_stick']
        self.throttle = mavlink_data['throttle']
        self.yaw_stick = mavlink_data['yaw_stick']
        
        # Normalize if dual MPU (values are -100..100, 0..100)
        if is_dual:
            self.roll_stick = self.roll_stick / 100.0
            self.pitch_stick = self.pitch_stick / 100.0
            self.throttle = self.throttle / 100.0
            self.yaw_stick = self.yaw_stick / 100.0
        
        # Apply invert settings in receiver mode
        roll_multiplier = -1 if self.invert_roll_check.isChecked() else 1
        pitch_multiplier = -1 if self.invert_pitch_check.isChecked() else 1
        
        self.roll_stick = self.roll_stick * roll_multiplier
        self.pitch_stick = self.pitch_stick * pitch_multiplier
        
        # Update control display (show inverted values)
        if is_dual:
            self.control_roll_label.setText(f"Roll: {self.roll_stick * 100.0:.1f}")
            self.control_pitch_label.setText(f"Pitch: {self.pitch_stick * 100.0:.1f}")
            self.control_throttle_label.setText(f"Throttle: {mavlink_data['throttle']:.1f}")
            self.control_yaw_label.setText(f"Yaw: {mavlink_data['yaw_stick']:.1f}")
        
        # Update joystick visualizations to match received data
        throttle_pos = (self.throttle * 2.0) - 1.0  # 0-1 to -1-1
        self.throttle_yaw_stick.set_position(self.yaw_stick, throttle_pos)
        self.pitch_roll_stick.set_position(self.roll_stick, self.pitch_stick)
        
        # Update MAVLink display with received data
        self.update_mavlink_commands()
        
        # Note: IMU data is already handled by on_data_received
    
    def on_error(self, error_msg):
        """Handle errors from serial reader."""
        self.status_label.setText(f"Status: {error_msg}")
        
    def closeEvent(self, event):
        """Clean up when closing the application."""
        if self.is_connected:
            self.serial_reader.disconnect()
            self.serial_reader.wait()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Modern look
    
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
