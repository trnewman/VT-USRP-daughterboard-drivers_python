#!/usr/bin/env python
#
# Copyright 2007 Free Software Foundation, Inc.
# 
# This file is part of GNU Radio
# 
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

from gnuradio import gr

class channel_model(gr.hier_block2):
    def __init__(self, noise_voltage=0.0, frequency_offset=0.0, epsilon=1.0, taps=[1.0,0.0], noise_seed=3021):
        ''' Creates a channel model that includes:
          - AWGN noise power in terms of noise voltage
          - A frequency offest in the channel in ratio
          - A timing offset ratio to model clock difference (epsilon)
          - Multipath taps
          '''
	gr.hier_block2.__init__(self, "channel_model",
				gr.io_signature(1, 1, gr.sizeof_gr_complex), # Input signature
				gr.io_signature(1, 1, gr.sizeof_gr_complex)) # Output signature

        #print epsilon
        self.timing_offset = gr.fractional_interpolator_cc(0, epsilon)
        
        self.multipath = gr.fir_filter_ccc(1, taps)
        
        self.noise_adder = gr.add_cc()
        self.noise = gr.noise_source_c(gr.GR_GAUSSIAN, noise_voltage, noise_seed)
        self.freq_offset = gr.sig_source_c(1, gr.GR_SIN_WAVE, frequency_offset, 1.0, 0.0)
        self.mixer_offset = gr.multiply_cc()

        self.connect(self, self.timing_offset, self.multipath)
        self.connect(self.multipath, (self.mixer_offset,0))
        self.connect(self.freq_offset,(self.mixer_offset,1))
        self.connect(self.mixer_offset, (self.noise_adder,1))
        self.connect(self.noise, (self.noise_adder,0))
        self.connect(self.noise_adder, self)
	
        
    def set_noise_voltage(self, noise_voltage):
        self.noise.set_amplitude(noise_voltage)
        
    def set_frequency_offset(self, frequency_offset):
        self.freq_offset.set_frequency(frequency_offset)
     
    def set_taps(self, taps):
        self.multipath.set_taps(taps)

    def set_timing_offset(self, epsilon):
        self.timing_offset.set_interp_ratio(epsilon)
