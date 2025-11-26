import sys
import serial
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from collections import deque
import threading
import queue
import time
import struct

# Config
SERIAL_PORT = '/dev/ttyACM0'

# Serial
SERIAL_BAUD = 4000000
START_WORD = 0xAAAA
END_WORD = 0xBBBB
HEADER_FORMAT = '<HBH' # Start word (uint16), Channel ID (uint8), Data size (uint16)
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
END_WORD_SIZE = struct.calcsize('<H')


# Visualization
SAMPLES_TO_DISPLAY = 256 * 6
MAX_VOLTAGE_MV = 3300
MIN_VOLTAGE_MV = 0

# Trigger
TRIGGER_LEVEL_MV = 1275
TIMER_INTERVAL = 0
TRIGGER_RANGE = 3

class myPlotter:
    def __init__(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1, parity=serial.PARITY_EVEN)
            print(f"Connected to {SERIAL_PORT} at {SERIAL_BAUD} baud.")
        except serial.SerialException as e:
            sys.exit(f"Error: Could not open serial port {SERIAL_PORT}. {e}")

        self.app = QtWidgets.QApplication([])
        self.win = pg.GraphicsLayoutWidget(title="ESP32 Oscilloscope (Multithreaded)")
        self.win.resize(1200, 800)

        # Plot for Channel 0
        self.plot0 = self.win.addPlot(title="Channel 0", row=0, col=0)
        self.plot0.setYRange(MIN_VOLTAGE_MV, MAX_VOLTAGE_MV)
        self.plot0.setXRange(0, SAMPLES_TO_DISPLAY)
        self.plot0.setLabel('left', 'Voltage (mV)')
        self.plot0.setLabel('bottom', 'Sample')
        self.plot0.showGrid(x=True, y=True)
        self.plot0.addLine(y=TRIGGER_LEVEL_MV, pen=pg.mkPen('g', style=QtCore.Qt.PenStyle.DashLine))
        self.curve0 = self.plot0.plot(pen='y')

        # Plot for Channel 1
        self.plot1 = self.win.addPlot(title="Channel 1", row=1, col=0)
        self.plot1.setYRange(MIN_VOLTAGE_MV, MAX_VOLTAGE_MV)
        self.plot1.setXRange(0, SAMPLES_TO_DISPLAY)
        self.plot1.setLabel('left', 'Voltage (mV)')
        self.plot1.setLabel('bottom', 'Sample')
        self.plot1.showGrid(x=True, y=True)
        self.plot1.addLine(y=TRIGGER_LEVEL_MV, pen=pg.mkPen('c', style=QtCore.Qt.PenStyle.DashLine))
        self.curve1 = self.plot1.plot(pen='c')
        
        # Plot for Channel 2
        self.plot2 = self.win.addPlot(title="Channel 2", row=0, col=1)
        self.plot2.setYRange(MIN_VOLTAGE_MV, MAX_VOLTAGE_MV)
        self.plot2.setLabel('left', 'Voltage (mV)')
        self.plot2.setLabel('bottom', 'Sample')
        self.plot2.showGrid(x=True, y=True)
        self.curve2 = self.plot2.plot(pen='m')

        # Plot for Channel 3
        self.plot3 = self.win.addPlot(title="Channel 3", row=1, col=1)
        self.plot3.setYRange(MIN_VOLTAGE_MV, MAX_VOLTAGE_MV)
        self.plot3.setLabel('left', 'Voltage (mV)')
        self.plot3.setLabel('bottom', 'Sample')
        self.plot3.showGrid(x=True, y=True)
        self.curve3 = self.plot3.plot(pen='w')

        self.win.ci.layout.setColumnStretchFactor(0, 2)
        self.win.ci.layout.setColumnStretchFactor(1, 1)


        max_buf_len = SAMPLES_TO_DISPLAY * 3
        self.data_buffer0 = deque(maxlen=max_buf_len)
        self.data_buffer1 = deque(maxlen=max_buf_len)
        self.data_buffer2 = deque(maxlen=100) # Smaller buffer for low frequency channels
        self.data_buffer3 = deque(maxlen=100)


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
        read_buffer = bytearray()
        while not self.stop_event.is_set():
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    read_buffer.extend(data)
                    
                    while True:
                        start_index = read_buffer.find(struct.pack('<H', START_WORD))
                        if start_index == -1:
                            break 

                        read_buffer = read_buffer[start_index:]

                        if len(read_buffer) < HEADER_SIZE:
                            break

                        header = read_buffer[:HEADER_SIZE]
                        _, channel_id, data_size = struct.unpack(HEADER_FORMAT, header)

                        if len(read_buffer) < HEADER_SIZE + data_size + END_WORD_SIZE:
                            break

                        end_word_index = HEADER_SIZE + data_size
                        end_word_bytes = read_buffer[end_word_index : end_word_index + END_WORD_SIZE]
                        
                        if len(end_word_bytes) < END_WORD_SIZE:
                           break

                        end_word = struct.unpack('<H', end_word_bytes)[0]

                        if end_word == END_WORD:
                            packet_data = read_buffer[HEADER_SIZE:end_word_index]
                            raw_values = np.frombuffer(packet_data, dtype=np.uint16)
                            self.data_queue.put((channel_id, raw_values))
                            read_buffer = read_buffer[end_word_index + END_WORD_SIZE:]
                        else:
                            read_buffer = read_buffer[1:]


            except Exception as e:
                print(f"Error in serial_reader: {e}")
                time.sleep(0.01)

    def update(self):
        try:
            while not self.data_queue.empty():
                channel_id, raw_values = self.data_queue.get_nowait()
                if channel_id == 0:
                    self.data_buffer0.extend(raw_values)
                elif channel_id == 1:
                    self.data_buffer1.extend(raw_values)
                elif channel_id == 2:
                    self.data_buffer2.extend(raw_values)
                elif channel_id == 3:
                    self.data_buffer3.extend(raw_values)

        except queue.Empty:
            pass

        self.update_plot_triggered(self.data_buffer0, self.curve0)
        self.update_plot_triggered(self.data_buffer1, self.curve1)
        self.update_plot_simple(self.data_buffer2, self.curve2)
        self.update_plot_simple(self.data_buffer3, self.curve3)

    def update_plot_triggered(self, data_buffer, curve):
        if len(data_buffer) < SAMPLES_TO_DISPLAY + TRIGGER_RANGE:
            return

        buffer_np = np.array(data_buffer, dtype=np.uint16)
        
        # Simple trigger - find first rising edge
        candidates = np.where((buffer_np[:-TRIGGER_RANGE] < TRIGGER_LEVEL_MV) & (buffer_np[TRIGGER_RANGE:] >= TRIGGER_LEVEL_MV))[0]
        
        if len(candidates) > 0:
            trigger_point = candidates[0] + 1
            if len(buffer_np) >= trigger_point + SAMPLES_TO_DISPLAY:
                waveform = buffer_np[trigger_point:trigger_point + SAMPLES_TO_DISPLAY]
                curve.setData(waveform)
                # Remove used data from buffer
                for _ in range(trigger_point):
                    data_buffer.popleft()

    def update_plot_simple(self, data_buffer, curve):
        if len(data_buffer) > 0:
            curve.setData(np.array(data_buffer))


    def cleanup(self):
        print("Cleaning up...")
        self.stop_event.set()
        self.reader_thread.join(timeout=1)
        self.ser.close()
        print("Cleanup complete.")

if __name__ == '__main__':
    plotter = myPlotter()
    sys.exit()
