# rplidar.py
# if not self.express_data:
# to
# if not self.express_data or type(self.express_data) == bool:

# stepper center to laser center is 28.5mm
from rplidar import RPLidar
import numpy as np
import pyqtgraph as pg
from pyqtgraph.opengl import GLViewWidget, GLScatterPlotItem
from PyQt5 import QtWidgets, QtCore
import sys
import time
from PyQt5.QtCore import Qt

PORT_NAME = 'COM4'

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

        self.current_x = 0
        self.all_points = np.empty((0, 3))
        self.new_scan_data = None
        self.constant_mode = True

    def handle_key_press(self, event):
        if event.key() == Qt.Key_Enter or event.key() == Qt.Key_Return:
            if self.new_scan_data is not None:
                self.take_snapshot(self.new_scan_data, accumulate=True)
                self.display_snapshot()
                self.current_x += 5

    def take_snapshot(self, scan, accumulate=True):
        angles = np.deg2rad(np.array([item[1] for item in scan]))
        distances = np.array([item[2] for item in scan])

        x = np.full(len(scan), self.current_x)
        y = distances * np.cos(angles)
        z = distances * np.sin(angles)

        new_points = np.vstack([x, y, z]).T

        if accumulate:
            self.all_points = np.vstack([self.all_points, new_points])
        else:
            self.all_points = new_points


    def display_snapshot(self):
        self.scatter.setData(
            pos=self.all_points,
            size=5
        )

    def update(self, scan):
        self.new_scan_data = scan
        if self.constant_mode:
            self.take_snapshot(scan, accumulate=False)
            self.display_snapshot()

    def reset_display(self):
        self.all_points = np.empty((0, 3))
        self.current_x = 0
        self.display_snapshot()

    def toggle_mode(self):
        self.constant_mode = not self.constant_mode
        self.mode_button.setText('Constant Mode' if self.constant_mode else 'Current Mode')
        if self.constant_mode and self.new_scan_data is not None:
            self.display_snapshot()

    def run(self):
        sys.exit(self.app.exec_())


def main():
    plotter = LidarPlotter()
    try:
        plotter.run()
    except KeyboardInterrupt:
        plotter.lidarThread.running = False

if __name__ == '__main__':
    main()
