import sys
import serial
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from collections import deque
import threading  
import queue      
import time      


#Config
#    SERIAL_PORT = sys.argv[1]
#    print("Error: Please provide the serial port as an argument.")
#    sys.exit(1)
SERIAL_PORT = '/dev/ttyACM0'

#Serial
SERIAL_BAUD = 4000000

SYNC_WORD = 0xAAAA

#Visualization
SAMPLES_TO_DISPLAY = 256 * 6
MAX_VOLTAGE_MV = 2600
MIN_VOLTAGE_MV = 0

#Trigger
TRIGGER_LEVEL_MV = 1275
TIMER_INTERVAL = 0 
TRIGGER_RANGE =3 

class myPlotter:
    def __init__(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1, parity=serial.PARITY_EVEN)
            print(f"Connected to {SERIAL_PORT} at {SERIAL_BAUD} baud.")
        except serial.SerialException as e:
            sys.exit(f"Error: Could not open serial port {SERIAL_PORT}. {e}")

        self.app = QtWidgets.QApplication([])
        self.win = pg.GraphicsLayoutWidget(title="ESP32 Oscilloscope (Multithreaded)")
        self.win.resize(1200, 600)
        self.plot = self.win.addPlot(title="Live ADC Data")
        self.plot.setYRange(MIN_VOLTAGE_MV, MAX_VOLTAGE_MV)
        self.plot.setXRange(0, SAMPLES_TO_DISPLAY)
        self.plot.setLabel('left', 'Voltage (mV)')
        self.plot.setLabel('bottom', 'Sample')
        self.plot.showGrid(x=True, y=True)
        self.plot.addLine(y=TRIGGER_LEVEL_MV, pen=pg.mkPen('g', style=QtCore.Qt.PenStyle.DashLine))
        self.curve = self.plot.plot(pen='y')

        max_buf_len = SAMPLES_TO_DISPLAY * 3
        self.data_buffer = deque(maxlen=max_buf_len)

        self.data_queue = queue.Queue() 
        self.stop_event = threading.Event() 
        self.reader_thread = threading.Thread(target=self.serial_reader)
        self.reader_thread.daemon = True 
        self.reader_thread.start()

        self.app.aboutToQuit.connect(self.cleanup)

        self.timer = QtCore.QTimer()
        self.timer.setInterval(TIMER_INTERVAL)
        self.timer.timeout.connect(self.update)
        self.timer.start()

        self.win.show()
        self.app.exec()

    def serial_reader(self):
        while not self.stop_event.is_set():
            bytes_to_read = self.ser.in_waiting
            if bytes_to_read > 0:
                data = self.ser.read(bytes_to_read)
                self.data_queue.put(data)
            else:
                time.sleep(0.001)

    def update(self):
        """Pulls data from the queue and updates the plot."""
        try:
            while not self.data_queue.empty():
                data_bytes = self.data_queue.get_nowait()
                if len(data_bytes) % 2 == 0:
                        raw_values = np.frombuffer(data_bytes, dtype=np.uint16)
                        if len(raw_values) > 0:
                            self.data_buffer.extend(raw_values)
        except queue.Empty:
            pass

        if len(self.data_buffer) < SAMPLES_TO_DISPLAY + 10:
            return

        buffer_np = np.array(self.data_buffer, dtype=np.uint16)
        candidates = np.where((buffer_np[:-TRIGGER_RANGE] < TRIGGER_LEVEL_MV) & (buffer_np[TRIGGER_RANGE:] >= TRIGGER_LEVEL_MV))[0]

        if len(candidates) > 0:
            trigger_point = candidates[0] + 1
            if len(buffer_np) >= trigger_point + SAMPLES_TO_DISPLAY:
                waveform = buffer_np[trigger_point : trigger_point + SAMPLES_TO_DISPLAY]
                self.curve.setData(waveform)
                for _ in range(trigger_point):
                    self.data_buffer.popleft()

    def cleanup(self):
        """Signals the reader thread to stop and closes the serial port."""
        print("Cleaning up...")
        self.stop_event.set()
        self.reader_thread.join(timeout=1)
        self.ser.close()
        print("Cleanup complete.")

if __name__ == '__main__':
    plotter = myPlotter()
    sys.exit() 
