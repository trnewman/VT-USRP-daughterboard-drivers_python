#
# Copyright 2005 Free Software Foundation, Inc.
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

import math
from gnuradio import gr, optfir
from gnuradio.blksimpl.fm_emph import fm_preemph

#from gnuradio import ctcss

class nbfm_tx(gr.hier_block):
    def __init__(self, fg, audio_rate, quad_rate, tau=75e-6, max_dev=5e3):
        """
        Narrow Band FM Transmitter.

        Takes a single float input stream of audio samples in the range [-1,+1]
        and produces a single FM modulated complex baseband output.

        @param fg: flow graph
        @param audio_rate: sample rate of audio stream, >= 16k
        @type audio_rate: integer
        @param quad_rate: sample rate of output stream
        @type quad_rate: integer
        @param tau: preemphasis time constant (default 75e-6)
        @type tau: float
        @param max_dev: maximum deviation in Hz (default 5e3)
        @type max_dev: float

        quad_rate must be an integer multiple of audio_rate.
        """
        
        # FIXME audio_rate and quad_rate ought to be exact rationals
        audio_rate = int(audio_rate)
        quad_rate = int(quad_rate)

        if quad_rate % audio_rate != 0:
            raise ValueError, "quad_rate is not an integer multiple of audio_rate"

        
        do_interp = audio_rate != quad_rate
        
        if do_interp:
            interp_factor = quad_rate / audio_rate
            interp_taps = optfir.low_pass (interp_factor,   # gain
                                           quad_rate,       # Fs
                                           4500,            # passband cutoff
                                           7000,            # stopband cutoff
                                           0.1,  	    # passband ripple dB
                                           40)              # stopband atten dB

            #print "len(interp_taps) =", len(interp_taps)
            self.interpolator = gr.interp_fir_filter_fff (interp_factor, interp_taps)

        self.preemph = fm_preemph (fg, quad_rate, tau=tau)
        
        k = 2 * math.pi * max_dev / quad_rate
        self.modulator = gr.frequency_modulator_fc (k)

        if do_interp:
            fg.connect (self.interpolator, self.preemph, self.modulator)
            gr.hier_block.__init__(self, fg, self.interpolator, self.modulator)
        else:
            fg.connect(self.preemph, self.modulator)
            gr.hier_block.__init__(self, fg, self.preemph, self.modulator)
        
       
#class ctcss_gen_f(gr.sig_source_f):
#    def __init__(self, sample_rate, tone_freq):
#        gr.sig_source_f.__init__(self, sample_rate, gr.SIN_WAVE, tone_freq, 0.1, 0.0)
#
#    def set_tone (self, tone):
#        gr.sig_source_f.set_frequency(self,tone)

class ctcss_gen_f(gr.hier_block):
    def __init__(self, fg, sample_rate, tone_freq):
        self.plgen = gr.sig_source_f(sample_rate, gr.GR_SIN_WAVE, tone_freq, 0.1, 0.0)
        
        gr.hier_block.__init__(self, fg, self.plgen, self.plgen)
