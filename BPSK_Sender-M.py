#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Bpsk Sender1
# Generated: Mon Jun  6 11:08:59 2016
##################################################
from sys import argv
from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import math


class BPSK_Sender1(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "Bpsk Sender1")

        ##################################################
        # Variables
        ##################################################
        self.sps = sps = 8
        self.nfilts = nfilts = 32
        self.transistion = transistion = 100
        self.sideband_rx = sideband_rx = 500
        self.sideband = sideband = 500
        self.samp_rate = samp_rate = 48000
        self.rrc_taps = rrc_taps = firdes.root_raised_cosine(nfilts, nfilts, 1.0/float(sps), 0.35, 11*sps*nfilts)
        self.qpsk = qpsk = digital.constellation_rect(([0.707+0.707j, -0.707+0.707j, -0.707-0.707j, 0.707-0.707j]), ([0, 1, 2, 3]), 4, 2, 2, 1, 1).base()
        self.preamble = preamble = [1,-1,1,-1,1,1,-1,-1,1,1,-1,1,1,1,-1,1,1,-1,1,-1,-1,1,-1,-1,1,1,1,-1,-1,-1,1,-1,1,1,1,1,-1,-1,1,-1,1,-1,-1,-1,1,1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,-1,-1]
        self.payload_size = payload_size = 10
        self.interpolation = interpolation = 18000
        self.gap = gap = 0
        self.eb = eb = 0.35
        self.delay = delay = 0
        self.decimation = decimation = 1
        self.constel = constel = digital.constellation_calcdist(([1,- 1]), ([0,1]), 2, 1).base()
        self.const_type = const_type = 1
        self.const = const = (digital.constellation_bpsk(), digital.constellation_qpsk(), digital.constellation_8psk())
        self.carrier = carrier = 10000
        self.arity = arity = 2

        ##################################################
        # Blocks
        ##################################################
        self.rational_resampler_xxx_0 = filter.rational_resampler_ccc(
                interpolation=interpolation,
                decimation=decimation,
                taps=None,
                fractional_bw=None,
        )
        self.freq_xlating_fir_filter_xxx_0 = filter.freq_xlating_fir_filter_ccc(1, (firdes.band_pass (0.5,samp_rate,10000-sideband,10000+sideband,transistion)), -carrier, samp_rate)
        self.digital_constellation_modulator_0 = digital.generic_mod(
          constellation=constel,
          differential=True,
          samples_per_symbol=sps,
          pre_diff_code=True,
          excess_bw=0.35,
          verbose=False,
          log=False,
          )
        self.blocks_wavfile_sink_1 = blocks.wavfile_sink("BPSK_output.wav", 1, 48000, 16)
        self.blocks_unpack_k_bits_bb_0_0 = blocks.unpack_k_bits_bb(8)
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_vff((0.1, ))
        self.blocks_file_source_0 = blocks.file_source(gr.sizeof_char*1, "TestData2", False)
        self.blocks_file_sink_0_0_0 = blocks.file_sink(gr.sizeof_char*1, "inputBinary", False)
        self.blocks_file_sink_0_0_0.set_unbuffered(False)
        self.blocks_file_sink_0_0 = blocks.file_sink(gr.sizeof_char*1, "inputBinary2", False)
        self.blocks_file_sink_0_0.set_unbuffered(False)
        
        

        
        self.blocks_delay_0_0 = blocks.delay(gr.sizeof_char*1, int(delay))
        self.blocks_complex_to_real_0 = blocks.complex_to_real(1)

        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_complex_to_real_0, 0), (self.blocks_multiply_const_vxx_0, 0))    
        self.connect((self.blocks_delay_0_0, 0), (self.blocks_file_sink_0_0, 0))    
        self.connect((self.blocks_delay_0_0, 0), (self.blocks_file_sink_0_0_0, 0))    
        self.connect((self.blocks_file_source_0, 0), (self.blocks_unpack_k_bits_bb_0_0, 0))    
        self.connect((self.blocks_file_source_0, 0), (self.digital_constellation_modulator_0, 0))    
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.blocks_wavfile_sink_1, 0))    
        self.connect((self.blocks_unpack_k_bits_bb_0_0, 0), (self.blocks_delay_0_0, 0))    
        self.connect((self.digital_constellation_modulator_0, 0), (self.rational_resampler_xxx_0, 0))    
        self.connect((self.freq_xlating_fir_filter_xxx_0, 0), (self.blocks_complex_to_real_0, 0))    
        self.connect((self.rational_resampler_xxx_0, 0), (self.freq_xlating_fir_filter_xxx_0, 0))    

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.set_rrc_taps(firdes.root_raised_cosine(self.nfilts, self.nfilts, 1.0/float(self.sps), 0.35, 11*self.sps*self.nfilts))

    def get_nfilts(self):
        return self.nfilts

    def set_nfilts(self, nfilts):
        self.nfilts = nfilts
        self.set_rrc_taps(firdes.root_raised_cosine(self.nfilts, self.nfilts, 1.0/float(self.sps), 0.35, 11*self.sps*self.nfilts))

    def get_transistion(self):
        return self.transistion

    def set_transistion(self, transistion):
        self.transistion = transistion
        self.freq_xlating_fir_filter_xxx_0.set_taps((firdes.band_pass (0.5,self.samp_rate,10000-self.sideband,10000+self.sideband,self.transistion)))

    def get_sideband_rx(self):
        return self.sideband_rx

    def set_sideband_rx(self, sideband_rx):
        self.sideband_rx = sideband_rx

    def get_sideband(self):
        return self.sideband

    def set_sideband(self, sideband):
        self.sideband = sideband
        self.freq_xlating_fir_filter_xxx_0.set_taps((firdes.band_pass (0.5,self.samp_rate,10000-self.sideband,10000+self.sideband,self.transistion)))

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.freq_xlating_fir_filter_xxx_0.set_taps((firdes.band_pass (0.5,self.samp_rate,10000-self.sideband,10000+self.sideband,self.transistion)))

    def get_rrc_taps(self):
        return self.rrc_taps

    def set_rrc_taps(self, rrc_taps):
        self.rrc_taps = rrc_taps

    def get_qpsk(self):
        return self.qpsk

    def set_qpsk(self, qpsk):
        self.qpsk = qpsk

    def get_preamble(self):
        return self.preamble

    def set_preamble(self, preamble):
        self.preamble = preamble

    def get_payload_size(self):
        return self.payload_size

    def set_payload_size(self, payload_size):
        self.payload_size = payload_size

    def get_interpolation(self):
        return self.interpolation

    def set_interpolation(self, interpolation):
        self.interpolation = interpolation

    def get_gap(self):
        return self.gap

    def set_gap(self, gap):
        self.gap = gap

    def get_eb(self):
        return self.eb

    def set_eb(self, eb):
        self.eb = eb

    def get_delay(self):
        return self.delay

    def set_delay(self, delay):
        self.delay = delay
        self.blocks_delay_0_0.set_dly(int(self.delay))

    def get_decimation(self):
        return self.decimation

    def set_decimation(self, decimation):
        self.decimation = decimation

    def get_constel(self):
        return self.constel

    def set_constel(self, constel):
        self.constel = constel

    def get_const_type(self):
        return self.const_type

    def set_const_type(self, const_type):
        self.const_type = const_type

    def get_const(self):
        return self.const

    def set_const(self, const):
        self.const = const

    def get_carrier(self):
        return self.carrier

    def set_carrier(self, carrier):
        self.carrier = carrier
        self.freq_xlating_fir_filter_xxx_0.set_center_freq(-self.carrier)

    def get_arity(self):
        return self.arity

    def set_arity(self, arity):
        self.arity = arity


def main(top_block_cls=BPSK_Sender1, options=None):
    tb = top_block_cls()
    tb.start()
    tb.wait()


if __name__ == '__main__':
    main()
