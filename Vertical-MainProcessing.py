import serial
import sys
import time
import numpy as np
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import Qt
from pyqtgraph.opengl import GLViewWidget, GLScatterPlotItem
from rplidar import RPLidar

PORT_NAME = 'COM4'

class SerialThread(QtCore.QThread):
    newAngle = QtCore.pyqtSignal(float)

    def __init__(self, port, baud_rate):
        super().__init__()
        self.serial = serial.Serial(port, baud_rate, timeout=1)
        self.running = True

    def run(self):
        time.sleep(2)
        while self.running:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').rstrip()
                try:
                    angle = float(line)
                    self.newAngle.emit(angle)
                except ValueError:
                    pass

class LidarThread(QtCore.QThread):
    newData = QtCore.pyqtSignal(object)

    def __init__(self, port_name):
        super().__init__()
        self.lidar = RPLidar(port_name)
        self.running = True

    def run(self):
        time.sleep(1)
        try:
            self.lidar.connect()
            status, error_code = self.lidar.get_health()
            if status != 'Good':
                print(f"Lidar health check failed: {status}, {error_code}")
                return

            self.lidar.express_data = True
            self.lidar.start_motor()
            while self.running:
                for scan in self.lidar.iter_scans(scan_type='express'):
                    self.newData.emit(scan)
                    if not self.running:
                        break
        except ValueError as e:
            print(f"Error during Lidar initialization: {e}")
        finally:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()

class CustomGLViewWidget(GLViewWidget):
    keyPressed = QtCore.pyqtSignal(QtCore.QEvent)

    def keyPressEvent(self, event):
        super(CustomGLViewWidget, self).keyPressEvent(event)
        self.keyPressed.emit(event)

class LidarPlotter:
    def __init__(self):
        self.app = QtWidgets.QApplication(sys.argv)
        self.win = CustomGLViewWidget()
        self.layout = QtWidgets.QVBoxLayout()
        self.container = QtWidgets.QWidget()

        self.scatter = GLScatterPlotItem()
        self.win.addItem(self.scatter)
        self.win.setWindowTitle('Lidar 3D Data')
        self.win.setCameraPosition(distance=500)
        self.win.keyPressed.connect(self.handle_key_press)

        self.reset_button = QtWidgets.QPushButton('Reset')
        self.reset_button.clicked.connect(self.reset_display)
        self.layout.addWidget(self.win)
        self.layout.addWidget(self.reset_button)

        self.mode_button = QtWidgets.QPushButton('Switch Mode')
        self.mode_button.clicked.connect(self.toggle_mode)
        self.layout.addWidget(self.mode_button)

        self.container.setLayout(self.layout)
        self.container.show()
        self.win.setWindowTitle('Lidar 3D Data')
        self.win.setCameraPosition(distance=500)

        self.lidarThread = LidarThread(PORT_NAME)
        self.lidarThread.newData.connect(self.update)
        self.lidarThread.start()

        self.serialThread = SerialThread("COM5", 115200)
        self.serialThread.newAngle.connect(self.update_current_x)
        self.serialThread.start()

        self.current_x = 0
        self.all_points = {}
        self.new_scan_data = None
        self.constant_mode = True

    def handle_key_press(self, event):
        if event.key() == Qt.Key_Enter or event.key() == Qt.Key_Return:
            if self.new_scan_data is not None:
                self.take_snapshot(self.new_scan_data, self.current_x)
                self.display_snapshot()

    def take_snapshot(self, scan, x_position):
        angles = np.deg2rad(np.array([item[1] for item in scan]))
        distances = np.array([item[2] for item in scan])
        y = distances * np.cos(angles)
        z = distances * np.sin(angles)
        new_points = np.vstack([np.full(len(scan), x_position), y, z]).T
        self.all_points[x_position] = new_points

    def display_snapshot(self):
        combined_points = np.vstack(list(self.all_points.values()))
        self.scatter.setData(
            pos=combined_points,
            size=5
        )

    def update(self, scan):
        self.new_scan_data = scan
        if self.constant_mode:
            self.take_snapshot(scan, self.current_x)
            self.display_snapshot()

    def reset_display(self):
        self.all_points = {}
        self.display_snapshot()

    def toggle_mode(self):
        self.constant_mode = not self.constant_mode
        self.mode_button.setText('Constant Mode' if self.constant_mode else 'Current Mode')
        if self.constant_mode and self.new_scan_data is not None:
            self.display_snapshot()

    def update_current_x(self, angle):
        self.current_x = angle
        if self.new_scan_data is not None and self.constant_mode:
            self.take_snapshot(self.new_scan_data, self.current_x)
            self.display_snapshot()

    def run(self):
        sys.exit(self.app.exec_())

def main():
    plotter = LidarPlotter()
    try:
        plotter.run()
    except KeyboardInterrupt:
        plotter.lidarThread.running = False
        plotter.serialThread.running = False

if __name__ == '__main__':
    main()
