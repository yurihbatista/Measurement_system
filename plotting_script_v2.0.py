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

# ==========================================
#              CONFIGURATION
# ==========================================

SERIAL_PORT = '/dev/ttyACM0' 
SERIAL_BAUD = 4000000

# --- POLARITY CORRECTION ---
INVERT_CURRENT = False

# --- TIMING ---
SAMPLE_RATE_HZ = 10225 
DT_MS = 1000.0 / SAMPLE_RATE_HZ 

# Display Window
DISPLAY_WINDOW_MS = 60  
SAMPLES_TO_DISPLAY = int(DISPLAY_WINDOW_MS / DT_MS)

# --- CALIBRATION ---
DC_OFFSET_MV = 1250 

GAIN_CH0_V_PER_MV = 0.282    
GAIN_CH1_MA_PER_MV = 0.09   
GAIN_CH2_C_PER_MV = 0.1     
GAIN_CH3_LX_PER_MV = 2.5 

# ADC Limits
MIN_ADC_MV = 0
MAX_ADC_MV = 2540

# --- VISUALS ---
Y_RANGE_V = (-350, 350)       
MAX_SLIDER_MA = 500         
INITIAL_SLIDER_MA = 100      

# --- TRIGGER ---
TRIGGER_RANGE = 5
TRIGGER_LEVEL_V_MV = 1275 
TRIGGER_LEVEL_I_MV = 1280 

# --- SERIAL PROTOCOL ---
START_WORD = 0xAAAA
END_WORD = 0xBBBB
HEADER_FORMAT = '<HBH' 
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
END_WORD_SIZE = struct.calcsize('<H')

# ==========================================
#           CUSTOM UI COMPONENTS
# ==========================================

class ClickablePlotWidget(pg.PlotWidget):
    sigDoubleClicked = QtCore.pyqtSignal(object)
    def __init__(self, name, parent=None, **kargs):
        super().__init__(parent, **kargs)
        self.plot_name = name
    def mouseDoubleClickEvent(self, ev):
        if ev.button() == QtCore.Qt.MouseButton.LeftButton:
            self.sigDoubleClicked.emit(self)
            ev.accept()
        else: super().mouseDoubleClickEvent(ev)

# ==========================================
#              MAIN APPLICATION
# ==========================================

class PowerAnalyzer(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        
        # --- Serial Connection ---
        try:
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1, parity=serial.PARITY_EVEN)
            print(f"Connected to {SERIAL_PORT} @ {SERIAL_BAUD}")
        except serial.SerialException as e:
            sys.exit(f"Serial Error: {e}")

        # --- Window Setup ---
        self.setWindowTitle("ESP32 Power Analyzer - Pro")
        self.resize(1400, 900)
        self.setStyleSheet("background-color: #101010; color: #EEE;")

        # --- Buffers ---
        max_buf = SAMPLES_TO_DISPLAY * 50 
        self.buf0 = deque(maxlen=max_buf)
        self.buf1 = deque(maxlen=max_buf)
        self.buf2 = deque(maxlen=200)
        self.buf3 = deque(maxlen=200)
        
        self.hist_v = deque(maxlen=500)
        self.hist_i = deque(maxlen=500)
        self.hist_p = deque(maxlen=500)
        
        # --- State Tracking ---
        self.fft_mode_active = False  # Track state to avoid resetting view every frame
        
        # --- Layouts ---
        self.main_layout = QtWidgets.QVBoxLayout(self)
        self.main_layout.setContentsMargins(5,5,5,5)
        
        # --- HEADER ---
        self.header_layout = QtWidgets.QHBoxLayout()
        self.main_layout.addLayout(self.header_layout)
        
        # 1. Info Label
        self.header_label = QtWidgets.QLabel("Initializing...")
        self.header_label.setStyleSheet("font-family: Monospace; font-size: 14px; padding: 5px; background-color: #202020; border: 1px solid #444;")
        self.header_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.header_label.setFixedHeight(40)
        self.header_layout.addWidget(self.header_label, stretch=4) 

        # 2. Controls
        ctrl_widget = QtWidgets.QWidget()
        ctrl_layout = QtWidgets.QHBoxLayout(ctrl_widget)
        ctrl_layout.setContentsMargins(0,0,0,0)
        ctrl_widget.setStyleSheet("background-color: #202020; border: 1px solid #444;")
        
        # FFT Toggle
        self.fft_cb = QtWidgets.QCheckBox("FFT")
        self.fft_cb.setStyleSheet("color: magenta; font-weight: bold; margin-right: 15px; margin-left: 10px;")
        ctrl_layout.addWidget(self.fft_cb)

        # Overlay Toggle
        self.overlay_cb = QtWidgets.QCheckBox("Overlay I")
        self.overlay_cb.setStyleSheet("color: cyan; font-weight: bold; margin-right: 10px;")
        self.overlay_cb.toggled.connect(self.on_overlay_toggled)
        ctrl_layout.addWidget(self.overlay_cb)

        self.range_label = QtWidgets.QLabel(f"±{INITIAL_SLIDER_MA} mA")
        self.range_label.setStyleSheet("color: white; font-family: Monospace; min-width: 80px;")
        ctrl_layout.addWidget(self.range_label)

        self.range_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.range_slider.setMinimum(0)
        self.range_slider.setMaximum(MAX_SLIDER_MA)
        self.range_slider.setValue(INITIAL_SLIDER_MA)
        self.range_slider.setFixedWidth(150)
        self.range_slider.valueChanged.connect(self.on_slider_changed)
        ctrl_layout.addWidget(self.range_slider)

        self.header_layout.addWidget(ctrl_widget, stretch=1)

        # --- PLOT GRID ---
        self.plot_layout = QtWidgets.QGridLayout()
        self.plot_layout.setSpacing(5)
        self.main_layout.addLayout(self.plot_layout)

        # --- Initialize Plots ---
        self.plots = {}
        self.curves = {}
        
        def create_plot(name, title, color, unit, y_range=None):
            p = ClickablePlotWidget(name, title=f"<b>{title}</b>")
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel('left', unit)
            if y_range: p.setYRange(y_range[0], y_range[1])
            p.sigDoubleClicked.connect(self.toggle_view_mode)
            curve = p.plot(pen=pg.mkPen(color, width=1.5))
            self.plots[name] = p
            self.curves[name] = curve
            return p

        # Voltage
        p_v = create_plot('volt', 'Tensão', 'y', 'V', Y_RANGE_V)
        # We start in Time Mode
        p_v.setXRange(0, DISPLAY_WINDOW_MS)
        p_v.getAxis('bottom').setLabel('Time (ms)')
        p_v.addLine(y=(TRIGGER_LEVEL_V_MV-DC_OFFSET_MV)*GAIN_CH0_V_PER_MV, pen=pg.mkPen('g', style=QtCore.Qt.PenStyle.DashLine))

        # Secondary Axis
        self.vb_main = p_v.getPlotItem().getViewBox() 
        self.vb_overlay = pg.ViewBox()
        p_v.showAxis('right')
        p_v.scene().addItem(self.vb_overlay)
        p_v.getAxis('right').linkToView(self.vb_overlay)
        self.vb_overlay.setXLink(self.vb_main)
        p_v.getAxis('right').setLabel('Current (mA)', color='#00FFFF')
        
        self.curve_overlay = pg.PlotCurveItem(pen=pg.mkPen('c', width=1, style=QtCore.Qt.PenStyle.DashLine))
        self.vb_overlay.addItem(self.curve_overlay)
        self.vb_main.sigResized.connect(self.update_overlay_geometry)
        
        # Current
        p_i = create_plot('curr', 'Corrente', 'c', 'mA', (-INITIAL_SLIDER_MA, INITIAL_SLIDER_MA))
        p_i.setXRange(0, DISPLAY_WINDOW_MS)
        p_i.getAxis('bottom').setLabel('Time (ms)')
        p_i.addLine(y=(TRIGGER_LEVEL_I_MV-DC_OFFSET_MV)*GAIN_CH1_MA_PER_MV, pen=pg.mkPen('c', style=QtCore.Qt.PenStyle.DashLine))

        # Power
        p_p = create_plot('power', 'Potência Instantânea', 'r', 'VA')
        p_p.setXRange(0, DISPLAY_WINDOW_MS)
        p_p.getAxis('bottom').setLabel('Time (ms)')

        # Trends
        p_trend = create_plot('trend', 'Vrms, Irms e Potência ativa', 'w', 'Magnitude')
        self.curves['trend_v'] = p_trend.plot(pen='y', name="RMS V")
        self.curves['trend_i'] = p_trend.plot(pen='c', name="RMS I")
        self.curves['trend_p'] = p_trend.plot(pen='r', name="Active P (W)") 

        # Slow Channels
        create_plot('temp', 'Temperatura', 'm', 'C', (0, 100))
        create_plot('lux', 'Iluminância', 'w', 'lx', (0, 2000))

        self.expanded_plot = None
        self.setup_default_layout()
        
        self.on_overlay_toggled(False)
        self.on_slider_changed(INITIAL_SLIDER_MA) 
        self.update_overlay_geometry() 

        # --- Worker ---
        self.data_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.reader_thread = threading.Thread(target=self.serial_reader)
        self.reader_thread.daemon = True
        self.reader_thread.start()

        self.time_axis = np.linspace(0, DISPLAY_WINDOW_MS, SAMPLES_TO_DISPLAY)
        
        # --- Timer ---
        self.timer = QtCore.QTimer()
        self.timer.setInterval(0) 
        self.timer.timeout.connect(self.update_loop)
        self.timer.start()

    # ================= LAYOUT MANAGEMENT =================
    
    def update_overlay_geometry(self):
        self.vb_overlay.setGeometry(self.vb_main.sceneBoundingRect())
        self.vb_overlay.linkedViewChanged(self.vb_main, self.vb_overlay.XAxis)

    def on_overlay_toggled(self, checked):
        if checked:
            self.plots['volt'].showAxis('right')
            self.curve_overlay.setVisible(True)
            self.update_overlay_geometry()
        else:
            self.plots['volt'].hideAxis('right')
            self.curve_overlay.setVisible(False)

    def on_slider_changed(self, value):
        self.range_label.setText(f"±{value} mA")
        limit = value if value > 0 else 1 
        self.plots['curr'].setYRange(-limit, limit)
        self.vb_overlay.setYRange(-limit, limit)

    def reset_grid_stretches(self):
        for i in range(10): 
            self.plot_layout.setColumnStretch(i, 0)
            self.plot_layout.setRowStretch(i, 0)

    def setup_default_layout(self):
        self.reset_grid_stretches()
        for p in self.plots.values():
            self.plot_layout.removeWidget(p)
            p.hide()
        
        self.plot_layout.addWidget(self.plots['volt'], 0, 0); self.plots['volt'].show()
        self.plot_layout.addWidget(self.plots['curr'], 0, 1); self.plots['curr'].show()
        self.plot_layout.addWidget(self.plots['power'], 0, 2); self.plots['power'].show()
        self.plot_layout.addWidget(self.plots['trend'], 1, 0); self.plots['trend'].show()
        self.plot_layout.addWidget(self.plots['temp'], 1, 1); self.plots['temp'].show()
        self.plot_layout.addWidget(self.plots['lux'], 1, 2); self.plots['lux'].show()
        
        self.plot_layout.setRowStretch(0, 3) 
        self.plot_layout.setRowStretch(1, 1)
        self.plot_layout.setColumnStretch(0, 1)
        self.plot_layout.setColumnStretch(1, 1)
        self.plot_layout.setColumnStretch(2, 1)
        
        self.update_overlay_geometry()

    def setup_expanded_layout(self, big_plot):
        self.reset_grid_stretches()
        for p in self.plots.values():
            self.plot_layout.removeWidget(p)
            p.hide()
            
        others = [p for p in self.plots.values() if p != big_plot]
        self.plot_layout.addWidget(big_plot, 0, 0, 1, len(others)); big_plot.show()
        for i, p in enumerate(others):
            self.plot_layout.addWidget(p, 1, i); p.show()
            
        self.plot_layout.setRowStretch(0, 10)
        self.plot_layout.setRowStretch(1, 1)
        for i in range(len(others)):
            self.plot_layout.setColumnStretch(i, 1)
            
        self.update_overlay_geometry()

    def toggle_view_mode(self, clicked_plot):
        QtCore.QTimer.singleShot(50, lambda: self._do_toggle(clicked_plot))

    def _do_toggle(self, clicked_plot):
        if self.expanded_plot == clicked_plot:
            self.expanded_plot = None
            self.setup_default_layout()
        else:
            self.expanded_plot = clicked_plot
            self.setup_expanded_layout(clicked_plot)

    def update_header(self, max_v, rms_v, freq_v, max_i, rms_i, active_p, avg_t, avg_l, queue_size):
        text = (
                f"<span style='color: yellow;'>V (max): {max_v:.0f} V | V (rms): {rms_v:.1f} V </span> | "
                f"<span style='color: #00FF00; font-weight:bold;'>CH0: {freq_v:.1f}Hz</span> | "
                f"<span style='color: cyan;'>I (max): {max_i:.0f} mA | I (RMS): {rms_i:.0f} mA</span> | "
            f"<span style='color: #FF5555;'>Potência ativa: {active_p:.1f} W</span> | "
            f"<span style='color: magenta;'>{avg_t:.1f} °C </span> | "
            f"<span style='color: white;'>{avg_l:.0f} lx</span> | "
            f"<span style='color: gray; font-size:10px;'>Buf: {queue_size}</span>"
        )
        self.header_label.setText(text)

    # ================= LOGIC =================
    
    def serial_reader(self):
        read_buffer = bytearray()
        pending_ch0_data = None
        while not self.stop_event.is_set():
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    read_buffer.extend(data)
                    while True:
                        start_idx = read_buffer.find(struct.pack('<H', START_WORD))
                        if start_idx == -1: break 
                        read_buffer = read_buffer[start_idx:]
                        if len(read_buffer) < HEADER_SIZE: break
                        header = read_buffer[:HEADER_SIZE]
                        _, ch_id, d_size = struct.unpack(HEADER_FORMAT, header)
                        total_len = HEADER_SIZE + d_size + END_WORD_SIZE
                        if len(read_buffer) < total_len: break
                        
                        end_bytes = read_buffer[HEADER_SIZE + d_size : total_len]
                        if struct.unpack('<H', end_bytes)[0] == END_WORD:
                            raw_bytes = read_buffer[HEADER_SIZE : HEADER_SIZE + d_size]
                            vals = np.frombuffer(raw_bytes, dtype=np.uint16).copy()
                            
                            if ch_id == 0:
                                pending_ch0_data = vals
                            elif ch_id == 1:
                                if pending_ch0_data is not None:
                                    n = min(len(pending_ch0_data), len(vals))
                                    self.data_queue.put((0, pending_ch0_data[:n]))
                                    self.data_queue.put((1, vals[:n]))
                                    pending_ch0_data = None
                            else:
                                self.data_queue.put((ch_id, vals))
                            read_buffer = read_buffer[total_len:]
                        else: read_buffer = read_buffer[1:]
            except Exception: time.sleep(0.01)

    def find_trigger(self, data_buf, trigger_level):
        limit = len(data_buf) - SAMPLES_TO_DISPLAY
        if limit <= TRIGGER_RANGE: return None
        arr = np.array(data_buf, dtype=np.uint16)
        candidates = np.where((arr[:limit-1] < trigger_level) & (arr[1:limit] >= trigger_level))[0]
        return candidates[0] + 1 if len(candidates) > 0 else None

    def calculate_frequency_and_bounds(self, waveform):
        if len(waveform) < 2: return 0.0, 0, len(waveform)
        crossings = np.where((waveform[:-1] < 0) & (waveform[1:] >= 0))[0]
        if len(crossings) < 2: return 0.0, 0, len(waveform)
        i1 = crossings[0]
        y1, y2 = waveform[i1], waveform[i1+1]
        t_start = i1 + (0 - y1) / (y2 - y1) if (y2 - y1) != 0 else i1
        i2 = crossings[-1]
        y3, y4 = waveform[i2], waveform[i2+1]
        t_end = i2 + (0 - y3) / (y4 - y3) if (y4 - y3) != 0 else i2
        total_samples = t_end - t_start
        num_cycles = len(crossings) - 1
        freq = (SAMPLE_RATE_HZ / (total_samples / num_cycles)) if total_samples > 0 else 0.0
        return freq, i1, i2

    def calculate_fft(self, signal, rate):
        window = np.hanning(len(signal))
        windowed_sig = signal * window
        fft_vals = np.fft.rfft(windowed_sig)
        fft_freqs = np.fft.rfftfreq(len(signal), 1.0/rate)
        fft_mag = np.abs(fft_vals) / len(signal) * 2
        return fft_freqs, fft_mag

    def update_loop(self):
        try:
            while not self.data_queue.empty():
                cid, vals = self.data_queue.get_nowait()
                if cid == 0: self.buf0.extend(vals)
                elif cid == 1: self.buf1.extend(vals)
                elif cid == 2: self.buf2.extend(vals)
                elif cid == 3: self.buf3.extend(vals)
        except queue.Empty: pass

        while len(self.buf0) > len(self.buf1): self.buf0.popleft()
        while len(self.buf1) > len(self.buf0): self.buf1.popleft()

        avg_t = avg_l = 0
        if len(self.buf2) > 0:
            d2 = np.array(self.buf2) * GAIN_CH2_C_PER_MV
            avg_t = np.mean(d2)
            self.curves['temp'].setData(d2)
        if len(self.buf3) > 0:
            d3 = np.array(self.buf3) * GAIN_CH3_LX_PER_MV
            avg_l = np.mean(d3)
            self.curves['lux'].setData(d3)

        if len(self.buf0) > SAMPLES_TO_DISPLAY + TRIGGER_RANGE:
            trig_v = self.find_trigger(self.buf0, TRIGGER_LEVEL_V_MV)
            if trig_v is not None:
                if len(self.buf1) >= trig_v + SAMPLES_TO_DISPLAY:
                    
                    raw_v = np.array([self.buf0[i+trig_v] for i in range(SAMPLES_TO_DISPLAY)], dtype=np.uint16)
                    raw_i_sync = np.array([self.buf1[i+trig_v] for i in range(SAMPLES_TO_DISPLAY)], dtype=np.uint16)
                    
                    trig_i = self.find_trigger(self.buf1, TRIGGER_LEVEL_I_MV)
                    if trig_i is not None and len(self.buf1) >= trig_i + SAMPLES_TO_DISPLAY:
                         raw_i_vis = np.array([self.buf1[i+trig_i] for i in range(SAMPLES_TO_DISPLAY)], dtype=np.uint16)
                    else:
                         raw_i_vis = raw_i_sync 
                    
                    v_phys = (raw_v.astype(np.float64) - DC_OFFSET_MV) * GAIN_CH0_V_PER_MV
                    i_phys_sync = (raw_i_sync.astype(np.float64) - DC_OFFSET_MV) * GAIN_CH1_MA_PER_MV
                    i_phys_vis = (raw_i_vis.astype(np.float64) - DC_OFFSET_MV) * GAIN_CH1_MA_PER_MV
                    
                    if INVERT_CURRENT:
                        i_phys_sync = -i_phys_sync
                        i_phys_vis = -i_phys_vis

                    p_inst = v_phys * (i_phys_sync / 1000.0)
                    freq_v, c_start, c_end = self.calculate_frequency_and_bounds(v_phys)
                    
                    # --- MODE SWITCHING LOGIC ---
                    is_fft = self.fft_cb.isChecked()
                    
                    # If mode changed, reset the view ONCE
                    if is_fft != self.fft_mode_active:
                        self.fft_mode_active = is_fft
                        if is_fft:
                            self.plots['volt'].getAxis('bottom').setLabel('Freq (Hz)')
                            self.plots['curr'].getAxis('bottom').setLabel('Freq (Hz)')
                            self.plots['power'].getAxis('bottom').setLabel('Freq (Hz)')
                            self.plots['volt'].setXRange(0, 1000)
                            self.plots['curr'].setXRange(0, 1000)
                            self.plots['power'].setXRange(0, 1000)
                        else:
                            self.plots['volt'].getAxis('bottom').setLabel('Time (ms)')
                            self.plots['curr'].getAxis('bottom').setLabel('Time (ms)')
                            self.plots['power'].getAxis('bottom').setLabel('Time (ms)')
                            self.plots['volt'].setXRange(0, DISPLAY_WINDOW_MS)
                            self.plots['curr'].setXRange(0, DISPLAY_WINDOW_MS)
                            self.plots['power'].setXRange(0, DISPLAY_WINDOW_MS)

                    # --- DATA UPDATES ---
                    if is_fft:
                        f_v, mag_v = self.calculate_fft(v_phys, SAMPLE_RATE_HZ)
                        f_i, mag_i = self.calculate_fft(i_phys_vis, SAMPLE_RATE_HZ)
                        f_p, mag_p = self.calculate_fft(p_inst, SAMPLE_RATE_HZ)
                        
                        self.curves['volt'].setData(f_v, mag_v)
                        self.curves['curr'].setData(f_i, mag_i)
                        self.curves['power'].setData(f_p, mag_p)
                        
                        if self.overlay_cb.isChecked():
                            self.curve_overlay.setData(f_i, mag_i)
                    else:
                        self.curves['volt'].setData(self.time_axis, v_phys)
                        self.curves['curr'].setData(self.time_axis, i_phys_vis)
                        self.curves['power'].setData(self.time_axis, p_inst)
                        
                        if self.overlay_cb.isChecked():
                            self.curve_overlay.setData(self.time_axis, i_phys_vis)
                    
                    # Stats (Always time domain)
                    if c_end > c_start:
                        active_p = np.mean(p_inst[c_start:c_end])
                        rms_v = np.sqrt(np.mean(v_phys[c_start:c_end]**2))
                        rms_i = np.sqrt(np.mean(i_phys_sync[c_start:c_end]**2))
                    else:
                        active_p = np.mean(p_inst)
                        rms_v = np.sqrt(np.mean(v_phys**2))
                        rms_i = np.sqrt(np.mean(i_phys_sync**2))
                    
                    max_v = np.max(np.abs(v_phys)) 
                    max_i = np.max(np.abs(i_phys_sync)) 
                    
                    self.hist_v.append(rms_v)
                    self.hist_i.append(rms_i / 1000.0)
                    self.hist_p.append(active_p)
                    self.curves['trend_v'].setData(np.array(self.hist_v))
                    self.curves['trend_i'].setData(np.array(self.hist_i))
                    self.curves['trend_p'].setData(np.array(self.hist_p))
                    
                    self.update_header(max_v, rms_v, freq_v, max_i, rms_i, active_p, avg_t, avg_l, self.data_queue.qsize())
                    
                    consume = trig_v + SAMPLES_TO_DISPLAY
                    for _ in range(consume):
                        self.buf0.popleft()
                        self.buf1.popleft()
            else:
                if len(self.buf0) > SAMPLES_TO_DISPLAY * 10:
                    self.buf0.popleft()
                    self.buf1.popleft()

    def closeEvent(self, event):
        self.stop_event.set()
        self.reader_thread.join(timeout=1)
        self.ser.close()
        event.accept()

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = PowerAnalyzer()
    window.show()
    sys.exit(app.exec())
