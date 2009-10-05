#
# Copyright 2008 Free Software Foundation, Inc.
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

from gnuradio import gr, window
from stream_to_vector_decimator import stream_to_vector_decimator
import math

class _logpwrfft_base(gr.hier_block2):
    """!
    Create a log10(abs(fft)) stream chain, with real or complex input.
    """
	
    def __init__(self, sample_rate, fft_size, ref_scale, frame_rate, avg_alpha, average):
    	"""!
	Create an log10(abs(fft)) stream chain.
	Provide access to the setting the filter and sample rate.
	@param sample_rate	Incoming stream sample rate
	@param fft_size		Number of FFT bins
	@param ref_scale	Sets 0 dB value input amplitude
	@param frame_rate	Output frame rate
	@param avg_alpha	FFT averaging (over time) constant [0.0-1.0]
	@param average		Whether to average [True, False]
	"""
	gr.hier_block2.__init__(self, self._name, 
				gr.io_signature(1, 1, self._item_size),          # Input signature
				gr.io_signature(1, 1, gr.sizeof_float*fft_size)) # Output signature

	self._sd = stream_to_vector_decimator(item_size=self._item_size,
					      sample_rate=sample_rate,
					      vec_rate=frame_rate,
					      vec_len=fft_size)
		
	fft_window = window.blackmanharris(fft_size)
	fft = self._fft_block[0](fft_size, True, fft_window)
	window_power = sum(map(lambda x: x*x, fft_window))

	c2mag = gr.complex_to_mag(fft_size)
	self._avg = gr.single_pole_iir_filter_ff(1.0, fft_size)
	self._log = gr.nlog10_ff(20, fft_size,
			         -10*math.log10(fft_size)              # Adjust for number of bins
				 -10*math.log10(window_power/fft_size) # Adjust for windowing loss
			         -20*math.log10(ref_scale/2))          # Adjust for reference scale
	self.connect(self, self._sd, fft, c2mag, self._avg, self._log, self)
	self.set_average(average)
	self.set_avg_alpha(avg_alpha)

    def set_sample_rate(self, sample_rate):
    	"""!
    	Set the new sampling rate
	@param sample_rate the new rate
	"""
	self._sd.set_sample_rate(sample_rate)

    def set_average(self, average):
        """!
        Set the averaging filter on/off.
        @param average true to set averaging on
        """
        self._average = average
        if self._average: 
    	    self._avg.set_taps(self._avg_alpha)
	else: 
	    self._avg.set_taps(1.0)

    def set_avg_alpha(self, avg_alpha):
	"""!
	Set the average alpha and set the taps if average was on.
	@param avg_alpha the new iir filter tap
	"""
	self._avg_alpha = avg_alpha
	self.set_average(self._average)

    def average(self):
	"""!
	Return whether or not averaging is being performed.
	"""
	return self._average
	
    def avg_alpha(self):
	"""!
	Return averaging filter constant.
	"""
	return self._avg_alpha


class logpwrfft_f(_logpwrfft_base):
	"""!
	Create an fft block chain, with real input.
	"""
	_name = "logpwrfft_f"
	_item_size = gr.sizeof_float
	_fft_block = (gr.fft_vfc, )

class logpwrfft_c(_logpwrfft_base):
	"""!
	Create an fft block chain, with complex input.
	"""
	_name = "logpwrfft_c"
	_item_size = gr.sizeof_gr_complex
	_fft_block = (gr.fft_vcc, )

