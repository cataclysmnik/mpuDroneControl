import sys
import numpy as np
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QComboBox, QPushButton, QLabel, 
                               QGroupBox, QGridLayout, QTextEdit)
from PySide6.QtCore import QTimer, Signal, QThread, Qt, QPointF
from PySide6.QtGui import QFont, QPainter, QColor, QPen, QBrush
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from OpenGL.GL import *
from OpenGL.GLU import *
import serial
import serial.tools.list_ports
import time
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
    """Thread for reading serial data from MPU6050."""
    
    data_received = Signal(float, float, float, float, float, float)
    error_occurred = Signal(str)
    
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.running = False
        
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
                    
                    # Expected format: ax,ay,az,gx,gy,gz
                    parts = line.split(',')
                    if len(parts) == 6:
                        ax, ay, az, gx, gy, gz = map(float, parts)
                        self.data_received.emit(ax, ay, az, gx, gy, gz)
                    
            except Exception as e:
                self.error_occurred.emit(f"Read error: {str(e)}")
                time.sleep(0.1)


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


class OpenGLWidget(QOpenGLWidget):
    """OpenGL widget for 3D visualization of MPU6050 orientation."""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
    def initializeGL(self):
        """Initialize OpenGL settings."""
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        
        # Set up light
        glLightfv(GL_LIGHT0, GL_POSITION, [1.0, 1.0, 1.0, 0.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.3, 0.3, 0.3, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.8, 0.8, 0.8, 1.0])
        
        glClearColor(0.1, 0.1, 0.15, 1.0)
        
    def resizeGL(self, w, h):
        """Handle window resize."""
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, w / h if h > 0 else 1, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        
    def paintGL(self):
        """Render the 3D scene."""
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        
        # Position camera
        glTranslatef(0.0, 0.0, -6.0)
        
        # Draw ground plane (before rotation)
        self.draw_ground_plane()
        
        # Apply rotations (in order: yaw, pitch, roll)
        glRotatef(self.yaw, 0.0, 1.0, 0.0)
        glRotatef(self.pitch, 1.0, 0.0, 0.0)
        glRotatef(self.roll, 0.0, 0.0, 1.0)
        
        # Draw MPU6050 board representation
        self.draw_board()
        
        # Draw axis indicators
        self.draw_axes()
        
    def draw_board(self):
        """Draw a rectangular board representing the MPU6050."""
        # Main board (blue-ish)
        glColor3f(0.2, 0.4, 0.8)
        glBegin(GL_QUADS)
        
        # Top face
        glNormal3f(0.0, 1.0, 0.0)
        glVertex3f(-1.5, 0.1, -1.0)
        glVertex3f(1.5, 0.1, -1.0)
        glVertex3f(1.5, 0.1, 1.0)
        glVertex3f(-1.5, 0.1, 1.0)
        
        # Bottom face
        glNormal3f(0.0, -1.0, 0.0)
        glVertex3f(-1.5, -0.1, -1.0)
        glVertex3f(-1.5, -0.1, 1.0)
        glVertex3f(1.5, -0.1, 1.0)
        glVertex3f(1.5, -0.1, -1.0)
        
        glEnd()
        
        # Sides
        glColor3f(0.15, 0.3, 0.6)
        glBegin(GL_QUADS)
        
        # Front
        glNormal3f(0.0, 0.0, 1.0)
        glVertex3f(-1.5, -0.1, 1.0)
        glVertex3f(-1.5, 0.1, 1.0)
        glVertex3f(1.5, 0.1, 1.0)
        glVertex3f(1.5, -0.1, 1.0)
        
        # Back
        glNormal3f(0.0, 0.0, -1.0)
        glVertex3f(-1.5, -0.1, -1.0)
        glVertex3f(1.5, -0.1, -1.0)
        glVertex3f(1.5, 0.1, -1.0)
        glVertex3f(-1.5, 0.1, -1.0)
        
        # Left
        glNormal3f(-1.0, 0.0, 0.0)
        glVertex3f(-1.5, -0.1, -1.0)
        glVertex3f(-1.5, 0.1, -1.0)
        glVertex3f(-1.5, 0.1, 1.0)
        glVertex3f(-1.5, -0.1, 1.0)
        
        # Right
        glNormal3f(1.0, 0.0, 0.0)
        glVertex3f(1.5, -0.1, -1.0)
        glVertex3f(1.5, -0.1, 1.0)
        glVertex3f(1.5, 0.1, 1.0)
        glVertex3f(1.5, 0.1, -1.0)
        
        glEnd()
        
        # Draw a small chip on top
        glColor3f(0.1, 0.1, 0.1)
        glPushMatrix()
        glTranslatef(0.0, 0.15, 0.0)
        glScalef(0.5, 0.05, 0.4)
        self.draw_cube()
        glPopMatrix()
        
    def draw_cube(self):
        """Draw a simple cube."""
        glBegin(GL_QUADS)
        
        # Front
        glVertex3f(-1, -1, 1)
        glVertex3f(1, -1, 1)
        glVertex3f(1, 1, 1)
        glVertex3f(-1, 1, 1)
        
        # Back
        glVertex3f(-1, -1, -1)
        glVertex3f(-1, 1, -1)
        glVertex3f(1, 1, -1)
        glVertex3f(1, -1, -1)
        
        # Top
        glVertex3f(-1, 1, -1)
        glVertex3f(-1, 1, 1)
        glVertex3f(1, 1, 1)
        glVertex3f(1, 1, -1)
        
        # Bottom
        glVertex3f(-1, -1, -1)
        glVertex3f(1, -1, -1)
        glVertex3f(1, -1, 1)
        glVertex3f(-1, -1, 1)
        
        # Right
        glVertex3f(1, -1, -1)
        glVertex3f(1, 1, -1)
        glVertex3f(1, 1, 1)
        glVertex3f(1, -1, 1)
        
        # Left
        glVertex3f(-1, -1, -1)
        glVertex3f(-1, -1, 1)
        glVertex3f(-1, 1, 1)
        glVertex3f(-1, 1, -1)
        
        glEnd()
        
    def draw_ground_plane(self):
        """Draw a ground plane for reference."""
        glDisable(GL_LIGHTING)
        glLineWidth(1.0)
        
        # Draw grid
        glColor3f(0.3, 0.3, 0.3)
        glBegin(GL_LINES)
        
        grid_size = 10
        grid_step = 1.0
        
        for i in range(-grid_size, grid_size + 1):
            pos = i * grid_step
            # Lines parallel to X axis
            glVertex3f(-grid_size * grid_step, -2.5, pos)
            glVertex3f(grid_size * grid_step, -2.5, pos)
            # Lines parallel to Z axis
            glVertex3f(pos, -2.5, -grid_size * grid_step)
            glVertex3f(pos, -2.5, grid_size * grid_step)
        
        glEnd()
        
        # Draw horizon reference lines (brighter)
        glColor3f(0.5, 0.5, 0.5)
        glLineWidth(2.0)
        glBegin(GL_LINES)
        # X axis on ground
        glVertex3f(-grid_size * grid_step, -2.5, 0)
        glVertex3f(grid_size * grid_step, -2.5, 0)
        # Z axis on ground
        glVertex3f(0, -2.5, -grid_size * grid_step)
        glVertex3f(0, -2.5, grid_size * grid_step)
        glEnd()
        
        glEnable(GL_LIGHTING)
        glLineWidth(1.0)
    
    def draw_axes(self):
        """Draw coordinate axes."""
        glDisable(GL_LIGHTING)
        glLineWidth(3.0)
        glBegin(GL_LINES)
        
        # X axis (Red)
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(2.0, 0.0, 0.0)
        
        # Y axis (Green)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 2.0, 0.0)
        
        # Z axis (Blue)
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 2.0)
        
        glEnd()
        glEnable(GL_LIGHTING)
        glLineWidth(1.0)
        
    def update_orientation(self, roll, pitch, yaw):
        """Update the orientation angles and trigger repaint."""
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.update()


class MainWindow(QMainWindow):
    """Main application window."""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MPU6050 Orientation Viewer")
        self.setGeometry(100, 100, 1000, 700)
        
        # Initialize components
        self.sensor_fusion = SensorFusion()
        self.serial_reader = SerialReader()
        self.is_connected = False
        
        # MAVLink control values
        self.throttle = 0.0  # 0 to 1 - start at bottom
        self.yaw_stick = 0.0  # -1 to 1
        self.pitch_stick = 0.0  # -1 to 1
        self.roll_stick = 0.0  # -1 to 1
        
        # Vertical motion tracking for gesture throttle
        self.throttle_gesture = 0.0  # Accumulated throttle from gestures
        self.last_az_normalized = 0.0  # Last vertical acceleration
        
        # IMU orientation for stabilization
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_yaw = 0.0
        
        # Connect signals
        self.serial_reader.data_received.connect(self.on_data_received)
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
        
        self.status_label = QLabel("Status: Disconnected - Close Arduino IDE Serial Monitor before connecting")
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
        
        # Create horizontal layout for 3D viz and controls
        viz_control_layout = QHBoxLayout()
        
        # 3D Visualization
        viz_group = QGroupBox("3D Orientation Visualization")
        viz_layout = QVBoxLayout()
        
        self.gl_widget = OpenGLWidget()
        self.gl_widget.setMinimumHeight(400)
        viz_layout.addWidget(self.gl_widget)
        
        viz_group.setLayout(viz_layout)
        viz_control_layout.addWidget(viz_group, 2)
        
        # Flight control panel
        control_panel_layout = QVBoxLayout()
        
        # Joystick controls
        joystick_group = QGroupBox("Flight Control Sticks")
        joystick_layout = QVBoxLayout()
        
        # Control mode selection
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Control Mode:"))
        self.control_mode_combo = QComboBox()
        self.control_mode_combo.addItems(["Manual (Mouse)", "IMU Controlled"])
        self.control_mode_combo.setCurrentIndex(1)  # Default to IMU controlled
        self.control_mode_combo.currentIndexChanged.connect(self.on_control_mode_changed)
        mode_layout.addWidget(self.control_mode_combo)
        
        mode_layout.addWidget(QLabel("Max Angle:"))
        self.max_angle_combo = QComboBox()
        self.max_angle_combo.addItems(["15°", "30°", "45°", "60°"])
        self.max_angle_combo.setCurrentIndex(1)  # Default to 30°
        mode_layout.addWidget(self.max_angle_combo)
        
        # Invert controls
        from PySide6.QtWidgets import QCheckBox
        self.invert_roll_check = QCheckBox("Invert Roll")
        self.invert_roll_check.setChecked(False)
        mode_layout.addWidget(self.invert_roll_check)
        
        self.invert_pitch_check = QCheckBox("Invert Pitch")
        self.invert_pitch_check.setChecked(True)  # Inverted by default
        mode_layout.addWidget(self.invert_pitch_check)
        
        self.gesture_throttle_check = QCheckBox("Gesture Throttle")
        self.gesture_throttle_check.setChecked(True)  # Enabled by default
        self.gesture_throttle_check.setToolTip("Control throttle by moving sensor up/down (vertical motion)")
        mode_layout.addWidget(self.gesture_throttle_check)
        
        mode_layout.addStretch()
        
        joystick_layout.addLayout(mode_layout)
        
        # Joystick widgets
        sticks_layout = QHBoxLayout()
        
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
            
    def refresh_ports(self):
        """Refresh the list of available COM ports."""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}", port.device)
            
        if self.port_combo.count() == 0:
            self.port_combo.addItem("No ports available", None)
            
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
                self.status_label.setText(f"Status: Connected to {port} at {baudrate} baud")
                self.port_combo.setEnabled(False)
                self.baud_combo.setEnabled(False)
                self.refresh_btn.setEnabled(False)
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
        mavlink_text = "=== MAVLink RC_CHANNELS_OVERRIDE ===\n"
        mavlink_text += f"Channel 1 (Roll):     {roll_pwm} µs\n"
        mavlink_text += f"Channel 2 (Pitch):    {pitch_pwm} µs\n"
        mavlink_text += f"Channel 3 (Throttle): {throttle_pwm} µs\n"
        mavlink_text += f"Channel 4 (Yaw):      {yaw_pwm} µs\n\n"
        
        mavlink_text += "=== MANUAL_CONTROL ===\n"
        mavlink_text += f"X (Pitch):   {int(self.pitch_stick * 1000):5d} (-1000 to +1000)\n"
        mavlink_text += f"Y (Roll):    {int(self.roll_stick * 1000):5d} (-1000 to +1000)\n"
        mavlink_text += f"Z (Throttle): {int(self.throttle * 1000):5d} (0 to +1000)\n"
        mavlink_text += f"R (Yaw):     {int(self.yaw_stick * 1000):5d} (-1000 to +1000)\n\n"
        
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
        
        # Update 3D visualization with calibrated angles
        self.gl_widget.update_orientation(roll_deg, pitch_deg, yaw_deg)
        
        # If in IMU control mode, update stick positions based on angles
        if self.control_mode_combo.currentIndex() == 1:  # IMU Controlled
            # Get max angle setting
            max_angle_text = self.max_angle_combo.currentText()
            max_angle = float(max_angle_text.replace('°', ''))
            
            # Map angles to stick positions (-1 to 1)
            # Apply invert settings
            roll_multiplier = -1 if self.invert_roll_check.isChecked() else 1
            pitch_multiplier = -1 if self.invert_pitch_check.isChecked() else 1
            
            # Roll: negative roll (left tilt) = negative stick (left)
            # Pitch: positive pitch (forward tilt) = positive stick (up)
            roll_stick_pos = roll_multiplier * (-roll_deg / max_angle)  # Base invert for intuitive control
            pitch_stick_pos = pitch_multiplier * (pitch_deg / max_angle)
            
            # Clamp to -1 to 1
            roll_stick_pos = max(-1.0, min(1.0, roll_stick_pos))
            pitch_stick_pos = max(-1.0, min(1.0, pitch_stick_pos))
            
            # Update pitch/roll stick position
            self.pitch_roll_stick.set_position(roll_stick_pos, pitch_stick_pos)
            
            # Update internal values for MAVLink generation
            self.roll_stick = roll_stick_pos
            self.pitch_stick = pitch_stick_pos
            
            # For throttle/yaw stick control
            # Yaw: use yaw rate from gyroscope
            yaw_rate = gz  # deg/s from gyroscope
            yaw_stick_pos = yaw_rate / 100.0  # Scale: 100 deg/s = full stick
            yaw_stick_pos = max(-1.0, min(1.0, yaw_stick_pos))
            
            # Throttle control with vertical motion gesture
            if self.gesture_throttle_check.isChecked():
                # Calculate the vertical component of acceleration
                # We need to account for sensor orientation to get true vertical
                # Use the acceleration magnitude and compare to gravity
                
                # Get the normalized vertical acceleration (should be ~1g when stationary)
                acc_magnitude = np.sqrt(ax**2 + ay**2 + az**2)
                
                if acc_magnitude > 0.1:
                    # Project acceleration onto the world vertical axis
                    # When sensor is level, az ≈ gravity
                    # When tilted, we need to rotate the acceleration vector
                    
                    # Simple approach: use the z-component adjusted for tilt
                    # Normalize accelerometer readings
                    ax_n = ax / acc_magnitude
                    ay_n = ay / acc_magnitude
                    az_n = az / acc_magnitude
                    
                    # Calculate rotation matrix components to get world-frame vertical
                    # For small angles, we can approximate
                    roll_rad = np.radians(roll_deg)
                    pitch_rad = np.radians(pitch_deg)
                    
                    # Rotate acceleration vector to world frame (simplified)
                    # Vertical component in world frame
                    az_world = (az_n * np.cos(roll_rad) * np.cos(pitch_rad) - 
                               ax_n * np.sin(pitch_rad) - 
                               ay_n * np.sin(roll_rad) * np.cos(pitch_rad))
                    
                    # Scale by magnitude to get actual vertical acceleration
                    vertical_accel = az_world * acc_magnitude
                    
                    # Detect up/down motion by comparing to gravity (9.8 m/s² or ~1g)
                    # Positive deviation = moving up, negative = moving down
                    gravity = 9.8  # m/s²
                    accel_deviation = vertical_accel - gravity
                    
                    # Scale the deviation to throttle change
                    # ±5 m/s² deviation = ±0.02 throttle change per update
                    throttle_change = accel_deviation / 250.0
                    
                    # Update throttle with smoothing
                    self.throttle_gesture += throttle_change
                    
                    # Decay towards zero slowly (auto-center)
                    decay_rate = 0.995
                    self.throttle_gesture *= decay_rate
                    
                    # Apply to actual throttle
                    self.throttle = max(0.0, min(1.0, self.throttle_gesture))
                    
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
        
    def zero_orientation(self):
        """Zero/level the current orientation."""
        self.sensor_fusion.zero_orientation()
        self.status_label.setText("Status: Orientation zeroed - Current position set as level reference")
    
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
