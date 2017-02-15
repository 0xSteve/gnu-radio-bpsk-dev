#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Bpsk Receiverlms
# Generated: Tue Jun  7 23:35:00 2016
##################################################

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

from PyQt4 import Qt
from gnuradio import analog
from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio import qtgui
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from gnuradio.qtgui import Range, RangeWidget
from optparse import OptionParser
import scipy
import sip
import sys
import threading
import time


class BPSK_ReceiverLMS(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Bpsk Receiverlms")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Bpsk Receiverlms")
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "BPSK_ReceiverLMS")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())

        ##################################################
        # Variables
        ##################################################
        self.sps = sps = 8
        self.probe_var_n = probe_var_n = 0
        self.probe_var = probe_var = 0
        self.nfilts = nfilts = 32
        self.eb = eb = 0.35
        self.SNR = SNR = 500
        self.transistion = transistion = 100
        self.timing_loop_bw = timing_loop_bw = 6.28/100.0
        self.sideband_rx = sideband_rx = 500
        self.sideband = sideband = 500
        self.samp_rate = samp_rate = 48000
        self.rrc_taps = rrc_taps = firdes.root_raised_cosine(nfilts/16, nfilts/16, 1.0/float(sps), 0.35, 11*sps*nfilts/16)
        self.qpsk = qpsk = digital.constellation_rect(([0.707+0.707j, -0.707+0.707j, -0.707-0.707j, 0.707-0.707j]), ([0, 1, 2, 3]), 4, 2, 2, 1, 1).base()
        self.preamble = preamble = [1,-1,1,-1,1,1,-1,-1,1,1,-1,1,1,1,-1,1,1,-1,1,-1,-1,1,-1,-1,1,1,1,-1,-1,-1,1,-1,1,1,1,1,-1,-1,1,-1,1,-1,-1,-1,1,1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,-1,-1]
        self.phase_bw = phase_bw = 6.28/100.0
        self.noise_amp = noise_amp = probe_var/(10**(SNR/20))
        self.matched_filter = matched_filter = firdes.root_raised_cosine(nfilts, nfilts, 1, eb, int(11*sps*nfilts))
        self.interpolation = interpolation = 2000
        self.eq_gain = eq_gain = 0.01
        self.delay = delay = 0
        self.decimation = decimation = 1
        self.constel = constel = digital.constellation_calcdist(([1,- 1]), ([0,1]), 2, 1).base()
        self.carrier = carrier = 10000
        self.arity = arity = 2
        self.Signal_rms = Signal_rms = probe_var
        self.Signal_Noise_amp = Signal_Noise_amp = probe_var_n

        ##################################################
        # Blocks
        ##################################################
        self._timing_loop_bw_range = Range(0.0, 0.2, 0.01, 6.28/100.0, 200)
        self._timing_loop_bw_win = RangeWidget(self._timing_loop_bw_range, self.set_timing_loop_bw, "Time: BW", "counter_slider", float)
        self.top_grid_layout.addWidget(self._timing_loop_bw_win, 0,0)
        self.probe_rms = blocks.probe_signal_f()
        self.probe_avg_n = blocks.probe_signal_f()
        self._phase_bw_range = Range(0.0, 1.0, 0.001, 6.28/100.0, 200)
        self._phase_bw_win = RangeWidget(self._phase_bw_range, self.set_phase_bw, "Phase: Bandwidth", "counter_slider", float)
        self.top_grid_layout.addWidget(self._phase_bw_win, 0,4)
        self._eq_gain_range = Range(0.0, 0.5, 0.001, 0.01, 200)
        self._eq_gain_win = RangeWidget(self._eq_gain_range, self.set_eq_gain, "Equalizer: rate", "slider", float)
        self.top_grid_layout.addWidget(self._eq_gain_win, 1,3)
        self.rational_resampler_xxx_0_0 = filter.rational_resampler_ccc(
                interpolation=decimation,
                decimation=interpolation,
                taps=(rrc_taps),
                fractional_bw=None,
        )
        self.qtgui_sink_x_0 = qtgui.sink_c(
        	1024, #fftsize
        	firdes.WIN_BLACKMAN_hARRIS, #wintype
        	0, #fc
        	samp_rate, #bw
        	"", #name
        	True, #plotfreq
        	True, #plotwaterfall
        	True, #plottime
        	True, #plotconst
        )
        self.qtgui_sink_x_0.set_update_time(1.0/10)
        self._qtgui_sink_x_0_win = sip.wrapinstance(self.qtgui_sink_x_0.pyqwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_sink_x_0_win)
        
        self.qtgui_sink_x_0.enable_rf_freq(False)
        
        
          
        def _probe_var_n_probe():
            while True:
                val = self.probe_avg_n.level()
                try:
                    self.set_probe_var_n(val)
                except AttributeError:
                    pass
                time.sleep(1.0 / (10))
        _probe_var_n_thread = threading.Thread(target=_probe_var_n_probe)
        _probe_var_n_thread.daemon = True
        _probe_var_n_thread.start()
        def _probe_var_probe():
            while True:
                val = self.probe_rms.level()
                try:
                    self.set_probe_var(val)
                except AttributeError:
                    pass
                time.sleep(1.0 / (10))
        _probe_var_thread = threading.Thread(target=_probe_var_probe)
        _probe_var_thread.daemon = True
        _probe_var_thread.start()
        self.freq_xlating_fir_filter_xxx_0_0 = filter.freq_xlating_fir_filter_ccc(1, (filter.firdes.low_pass(1, samp_rate*10, sideband_rx,1000)), carrier, samp_rate)
        self.digital_pfb_clock_sync_xxx_0 = digital.pfb_clock_sync_ccf(sps, timing_loop_bw, (rrc_taps), nfilts, nfilts/2, 1.5, 1)
        self.digital_lms_dd_equalizer_cc_1 = digital.lms_dd_equalizer_cc(8, eq_gain, 1, constel)
        self.digital_diff_decoder_bb_0 = digital.diff_decoder_bb(2)
        self.digital_costas_loop_cc_0 = digital.costas_loop_cc(phase_bw, arity, False)
        self.digital_constellation_decoder_cb_0 = digital.constellation_decoder_cb(constel)
        self.blocks_wavfile_source_0 = blocks.wavfile_source("/Users/ahmadtrabousli/Desktop/GnuRadioModems/Impulse Responses/5km_20pc/BPSK_Output/goff_random_20pc_5km_E_no3.wav", False)
        self.blocks_throttle_1_0_0 = blocks.throttle(gr.sizeof_gr_complex*1, samp_rate,True)
        self.blocks_rms_xx_1 = blocks.rms_cf(0.01)
        self.blocks_rms_xx_0 = blocks.rms_cf(0.01)
        self.blocks_pack_k_bits_bb_0 = blocks.pack_k_bits_bb(8)
        self.blocks_float_to_complex_0 = blocks.float_to_complex(1)
        self.blocks_file_sink_0_1 = blocks.file_sink(gr.sizeof_char*1, "/Users/ahmadtrabousli/Desktop/GnuRadioModems/PSK/outputText", False)
        self.blocks_file_sink_0_1.set_unbuffered(False)
        self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_char*1, "/Users/ahmadtrabousli/Desktop/GnuRadioModems/PSK/outputBinary", False)
        self.blocks_file_sink_0.set_unbuffered(False)
        self.blocks_delay_1 = blocks.delay(gr.sizeof_char*1, int(delay))
        self.blocks_add_xx_0 = blocks.add_vcc(1)
        self.analog_noise_source_x_0 = analog.noise_source_c(analog.GR_GAUSSIAN, noise_amp, 0)
        self._Signal_rms_tool_bar = Qt.QToolBar(self)
        
        if None:
          self._Signal_rms_formatter = None
        else:
          self._Signal_rms_formatter = lambda x: x
        
        self._Signal_rms_tool_bar.addWidget(Qt.QLabel("Signal_rms"+": "))
        self._Signal_rms_label = Qt.QLabel(str(self._Signal_rms_formatter(self.Signal_rms)))
        self._Signal_rms_tool_bar.addWidget(self._Signal_rms_label)
        self.top_layout.addWidget(self._Signal_rms_tool_bar)
          
        self._Signal_Noise_amp_tool_bar = Qt.QToolBar(self)
        
        if None:
          self._Signal_Noise_amp_formatter = None
        else:
          self._Signal_Noise_amp_formatter = lambda x: x
        
        self._Signal_Noise_amp_tool_bar.addWidget(Qt.QLabel("Signal_Noise_amp"+": "))
        self._Signal_Noise_amp_label = Qt.QLabel(str(self._Signal_Noise_amp_formatter(self.Signal_Noise_amp)))
        self._Signal_Noise_amp_tool_bar.addWidget(self._Signal_Noise_amp_label)
        self.top_layout.addWidget(self._Signal_Noise_amp_tool_bar)
          

        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_noise_source_x_0, 0), (self.blocks_add_xx_0, 1))    
        self.connect((self.analog_noise_source_x_0, 0), (self.blocks_rms_xx_1, 0))    
        self.connect((self.blocks_add_xx_0, 0), (self.digital_pfb_clock_sync_xxx_0, 0))    
        self.connect((self.blocks_delay_1, 0), (self.blocks_file_sink_0, 0))    
        self.connect((self.blocks_float_to_complex_0, 0), (self.freq_xlating_fir_filter_xxx_0_0, 0))    
        self.connect((self.blocks_pack_k_bits_bb_0, 0), (self.blocks_file_sink_0_1, 0))    
        self.connect((self.blocks_rms_xx_0, 0), (self.probe_rms, 0))    
        self.connect((self.blocks_rms_xx_1, 0), (self.probe_avg_n, 0))    
        self.connect((self.blocks_throttle_1_0_0, 0), (self.blocks_add_xx_0, 0))    
        self.connect((self.blocks_throttle_1_0_0, 0), (self.blocks_rms_xx_0, 0))    
        self.connect((self.blocks_wavfile_source_0, 0), (self.blocks_float_to_complex_0, 0))    
        self.connect((self.digital_constellation_decoder_cb_0, 0), (self.digital_diff_decoder_bb_0, 0))    
        self.connect((self.digital_costas_loop_cc_0, 0), (self.digital_constellation_decoder_cb_0, 0))    
        self.connect((self.digital_diff_decoder_bb_0, 0), (self.blocks_delay_1, 0))    
        self.connect((self.digital_diff_decoder_bb_0, 0), (self.blocks_pack_k_bits_bb_0, 0))    
        self.connect((self.digital_lms_dd_equalizer_cc_1, 0), (self.digital_costas_loop_cc_0, 0))    
        self.connect((self.digital_pfb_clock_sync_xxx_0, 0), (self.digital_lms_dd_equalizer_cc_1, 0))    
        self.connect((self.digital_pfb_clock_sync_xxx_0, 0), (self.qtgui_sink_x_0, 0))    
        self.connect((self.freq_xlating_fir_filter_xxx_0_0, 0), (self.rational_resampler_xxx_0_0, 0))    
        self.connect((self.rational_resampler_xxx_0_0, 0), (self.blocks_throttle_1_0_0, 0))    

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "BPSK_ReceiverLMS")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()


    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.set_matched_filter(firdes.root_raised_cosine(self.nfilts, self.nfilts, 1, self.eb, int(11*self.sps*self.nfilts)))
        self.set_rrc_taps(firdes.root_raised_cosine(self.nfilts/16, self.nfilts/16, 1.0/float(self.sps), 0.35, 11*self.sps*self.nfilts/16))

    def get_probe_var_n(self):
        return self.probe_var_n

    def set_probe_var_n(self, probe_var_n):
        self.probe_var_n = probe_var_n
        self.set_Signal_Noise_amp(self._Signal_Noise_amp_formatter(self.probe_var_n))

    def get_probe_var(self):
        return self.probe_var

    def set_probe_var(self, probe_var):
        self.probe_var = probe_var
        self.set_Signal_rms(self._Signal_rms_formatter(self.probe_var))
        self.set_noise_amp(self.probe_var/(10**(self.SNR/20)))

    def get_nfilts(self):
        return self.nfilts

    def set_nfilts(self, nfilts):
        self.nfilts = nfilts
        self.set_matched_filter(firdes.root_raised_cosine(self.nfilts, self.nfilts, 1, self.eb, int(11*self.sps*self.nfilts)))
        self.set_rrc_taps(firdes.root_raised_cosine(self.nfilts/16, self.nfilts/16, 1.0/float(self.sps), 0.35, 11*self.sps*self.nfilts/16))

    def get_eb(self):
        return self.eb

    def set_eb(self, eb):
        self.eb = eb
        self.set_matched_filter(firdes.root_raised_cosine(self.nfilts, self.nfilts, 1, self.eb, int(11*self.sps*self.nfilts)))

    def get_SNR(self):
        return self.SNR

    def set_SNR(self, SNR):
        self.SNR = SNR
        self.set_noise_amp(self.probe_var/(10**(self.SNR/20)))

    def get_transistion(self):
        return self.transistion

    def set_transistion(self, transistion):
        self.transistion = transistion

    def get_timing_loop_bw(self):
        return self.timing_loop_bw

    def set_timing_loop_bw(self, timing_loop_bw):
        self.timing_loop_bw = timing_loop_bw
        self.digital_pfb_clock_sync_xxx_0.set_loop_bandwidth(self.timing_loop_bw)

    def get_sideband_rx(self):
        return self.sideband_rx

    def set_sideband_rx(self, sideband_rx):
        self.sideband_rx = sideband_rx
        self.freq_xlating_fir_filter_xxx_0_0.set_taps((filter.firdes.low_pass(1, self.samp_rate*10, self.sideband_rx,1000)))

    def get_sideband(self):
        return self.sideband

    def set_sideband(self, sideband):
        self.sideband = sideband

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.blocks_throttle_1_0_0.set_sample_rate(self.samp_rate)
        self.freq_xlating_fir_filter_xxx_0_0.set_taps((filter.firdes.low_pass(1, self.samp_rate*10, self.sideband_rx,1000)))
        self.qtgui_sink_x_0.set_frequency_range(0, self.samp_rate)

    def get_rrc_taps(self):
        return self.rrc_taps

    def set_rrc_taps(self, rrc_taps):
        self.rrc_taps = rrc_taps
        self.digital_pfb_clock_sync_xxx_0.set_taps((self.rrc_taps))
        self.rational_resampler_xxx_0_0.set_taps((self.rrc_taps))

    def get_qpsk(self):
        return self.qpsk

    def set_qpsk(self, qpsk):
        self.qpsk = qpsk

    def get_preamble(self):
        return self.preamble

    def set_preamble(self, preamble):
        self.preamble = preamble

    def get_phase_bw(self):
        return self.phase_bw

    def set_phase_bw(self, phase_bw):
        self.phase_bw = phase_bw
        self.digital_costas_loop_cc_0.set_loop_bandwidth(self.phase_bw)

    def get_noise_amp(self):
        return self.noise_amp

    def set_noise_amp(self, noise_amp):
        self.noise_amp = noise_amp
        self.analog_noise_source_x_0.set_amplitude(self.noise_amp)

    def get_matched_filter(self):
        return self.matched_filter

    def set_matched_filter(self, matched_filter):
        self.matched_filter = matched_filter

    def get_interpolation(self):
        return self.interpolation

    def set_interpolation(self, interpolation):
        self.interpolation = interpolation

    def get_eq_gain(self):
        return self.eq_gain

    def set_eq_gain(self, eq_gain):
        self.eq_gain = eq_gain
        self.digital_lms_dd_equalizer_cc_1.set_gain(self.eq_gain)

    def get_delay(self):
        return self.delay

    def set_delay(self, delay):
        self.delay = delay
        self.blocks_delay_1.set_dly(int(self.delay))

    def get_decimation(self):
        return self.decimation

    def set_decimation(self, decimation):
        self.decimation = decimation

    def get_constel(self):
        return self.constel

    def set_constel(self, constel):
        self.constel = constel

    def get_carrier(self):
        return self.carrier

    def set_carrier(self, carrier):
        self.carrier = carrier
        self.freq_xlating_fir_filter_xxx_0_0.set_center_freq(self.carrier)

    def get_arity(self):
        return self.arity

    def set_arity(self, arity):
        self.arity = arity

    def get_Signal_rms(self):
        return self.Signal_rms

    def set_Signal_rms(self, Signal_rms):
        self.Signal_rms = Signal_rms
        Qt.QMetaObject.invokeMethod(self._Signal_rms_label, "setText", Qt.Q_ARG("QString", eng_notation.num_to_str(self.Signal_rms)))

    def get_Signal_Noise_amp(self):
        return self.Signal_Noise_amp

    def set_Signal_Noise_amp(self, Signal_Noise_amp):
        self.Signal_Noise_amp = Signal_Noise_amp
        Qt.QMetaObject.invokeMethod(self._Signal_Noise_amp_label, "setText", Qt.Q_ARG("QString", eng_notation.num_to_str(self.Signal_Noise_amp)))


def main(top_block_cls=BPSK_ReceiverLMS, options=None):

    from distutils.version import StrictVersion
    if StrictVersion(Qt.qVersion()) >= StrictVersion("4.5.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()
    tb.start()
    tb.show()

    def quitting():
        tb.stop()
        tb.wait()
    qapp.connect(qapp, Qt.SIGNAL("aboutToQuit()"), quitting)
    qapp.exec_()


if __name__ == '__main__':
    main()
