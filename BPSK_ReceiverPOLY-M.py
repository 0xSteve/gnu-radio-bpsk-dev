#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Bpsk Receiverpoly
# Generated: Tue Jun  7 15:44:32 2016
##################################################
from sys import argv
from gnuradio import analog
from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import scipy
import threading
import time


class BPSK_ReceiverPOLY(gr.top_block):
    
    def __init__(self):
        gr.top_block.__init__(self, "Bpsk Receiverpoly")
        
        ##################################################
        # Variables
        ##################################################
        self.sps = sps = 8
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
        self.probe_var_n = probe_var_n = 0
        self.preamble = preamble = [1,-1,1,-1,1,1,-1,-1,1,1,-1,1,1,1,-1,1,1,-1,1,-1,-1,1,-1,-1,1,1,1,-1,-1,-1,1,-1,1,1,1,1,-1,-1,1,-1,1,-1,-1,-1,1,1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,-1,-1]
        self.phase_bw = phase_bw = 6.28/100.0
        self.noise_amp = noise_amp = probe_var/(10**(SNR/20))
        self.matched_filter = matched_filter = firdes.root_raised_cosine(nfilts, nfilts, 1, eb, int(11*sps*nfilts))
        self.interpolation = interpolation = 60000
        self.eq_gain = eq_gain = 0.01
        self.delay = delay = 0
        self.decimation = decimation = 1
        self.constel = constel = digital.constellation_calcdist(([1,- 1]), ([0,1]), 2, 1).base()
        self.carrier = carrier = 10000
        self.arity = arity = 2
        
        ##################################################
        # Blocks
        ##################################################
        self.probe_rms = blocks.probe_signal_f()
        self.probe_avg_n = blocks.probe_signal_f()
        self.rational_resampler_xxx_0_0 = filter.rational_resampler_ccc(
                                                                        interpolation=decimation,
                                                                        decimation=interpolation,
                                                                        taps=(rrc_taps),
                                                                        fractional_bw=None,
                                                                        )
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
        self.digital_pfb_clock_sync_xxx_0 = digital.pfb_clock_sync_ccf(sps, timing_loop_bw, (rrc_taps), nfilts*2, nfilts/2, 1.5, 1)
        self.digital_diff_decoder_bb_0 = digital.diff_decoder_bb(2)
        self.digital_constellation_decoder_cb_0 = digital.constellation_decoder_cb(constel)
        
        script, SNRinput, inputwav, outputBinary, delay= argv
        
        self.blocks_wavfile_source_0 = blocks.wavfile_source(inputwav, False)
        self.blocks_throttle_1_0_0 = blocks.throttle(gr.sizeof_gr_complex*1, samp_rate,True)
        self.blocks_rms_xx_1 = blocks.rms_cf(0.01)
        self.blocks_rms_xx_0 = blocks.rms_cf(0.01)
        self.blocks_float_to_complex_0 = blocks.float_to_complex(1)
        self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_char*1, outputBinary, False)
        self.blocks_file_sink_0.set_unbuffered(False)
        self.blocks_delay_1 = blocks.delay(gr.sizeof_char*1, int(delay))
        self.blocks_add_xx_0 = blocks.add_vcc(1)
        self.analog_noise_source_x_0 = analog.noise_source_c(analog.GR_GAUSSIAN, noise_amp, 0)
        
        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_noise_source_x_0, 0), (self.blocks_add_xx_0, 1))
        self.connect((self.analog_noise_source_x_0, 0), (self.blocks_rms_xx_1, 0))
        self.connect((self.blocks_add_xx_0, 0), (self.digital_pfb_clock_sync_xxx_0, 0))
        self.connect((self.blocks_delay_1, 0), (self.blocks_file_sink_0, 0))
        self.connect((self.blocks_float_to_complex_0, 0), (self.freq_xlating_fir_filter_xxx_0_0, 0))
        self.connect((self.blocks_rms_xx_0, 0), (self.probe_rms, 0))
        self.connect((self.blocks_rms_xx_1, 0), (self.probe_avg_n, 0))
        self.connect((self.blocks_throttle_1_0_0, 0), (self.blocks_add_xx_0, 0))
        self.connect((self.blocks_throttle_1_0_0, 0), (self.blocks_rms_xx_0, 0))
        self.connect((self.blocks_wavfile_source_0, 0), (self.blocks_float_to_complex_0, 0))
        self.connect((self.digital_constellation_decoder_cb_0, 0), (self.digital_diff_decoder_bb_0, 0))
        self.connect((self.digital_diff_decoder_bb_0, 0), (self.blocks_delay_1, 0))
        self.connect((self.digital_pfb_clock_sync_xxx_0, 0), (self.digital_constellation_decoder_cb_0, 0))
        self.connect((self.freq_xlating_fir_filter_xxx_0_0, 0), (self.rational_resampler_xxx_0_0, 0))
        self.connect((self.rational_resampler_xxx_0_0, 0), (self.blocks_throttle_1_0_0, 0))
    
    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.set_matched_filter(firdes.root_raised_cosine(self.nfilts, self.nfilts, 1, self.eb, int(11*self.sps*self.nfilts)))
        self.set_rrc_taps(firdes.root_raised_cosine(self.nfilts/8, self.nfilts/8, 1.0/float(self.sps), 0.35, 11*self.sps*self.nfilts/8))
    
    def get_probe_var(self):
        return self.probe_var

    def set_probe_var(self, probe_var):
        self.probe_var = probe_var
        self.set_noise_amp(self.probe_var/(10**(self.SNR/20)))
    
    def get_nfilts(self):
        return self.nfilts

    def set_nfilts(self, nfilts):
        self.nfilts = nfilts
        self.set_matched_filter(firdes.root_raised_cosine(self.nfilts, self.nfilts, 1, self.eb, int(11*self.sps*self.nfilts)))
        self.set_rrc_taps(firdes.root_raised_cosine(self.nfilts/8, self.nfilts/8, 1.0/float(self.sps), 0.35, 11*self.sps*self.nfilts/8))
    
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
    
    def get_rrc_taps(self):
        return self.rrc_taps
    
    def set_rrc_taps(self, rrc_taps):
        self.rrc_taps = rrc_taps
        self.rational_resampler_xxx_0_0.set_taps((self.rrc_taps))
        self.digital_pfb_clock_sync_xxx_0.set_taps((self.rrc_taps))
    
    def get_qpsk(self):
        return self.qpsk
    
    def set_qpsk(self, qpsk):
        self.qpsk = qpsk
    
    def get_probe_var_n(self):
        return self.probe_var_n
    
    def set_probe_var_n(self, probe_var_n):
        self.probe_var_n = probe_var_n
    
    def get_preamble(self):
        return self.preamble
    
    def set_preamble(self, preamble):
        self.preamble = preamble
    
    def get_phase_bw(self):
        return self.phase_bw
    
    def set_phase_bw(self, phase_bw):
        self.phase_bw = phase_bw
    
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


def main(top_block_cls=BPSK_ReceiverPOLY, options=None):
    tb = top_block_cls()
    script, SNRinput, inputwav, outputBinary, sdelay= argv
    tb.set_SNR(float(SNRinput))
    tb.set_delay(int(sdelay))
    tb.start()
    tb.set_SNR(float(SNRinput))
    tb.set_delay(int(sdelay))
    tb.wait()
if __name__ == '__main__':
    main()
