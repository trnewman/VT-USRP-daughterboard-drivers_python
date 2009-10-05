#
# GMSK modulation and demodulation.  
#
#
# Copyright 2005,2006 Free Software Foundation, Inc.
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

# See gnuradio-examples/python/digital for examples

from gnuradio import gr
from gnuradio import modulation_utils
from math import pi
import numpy
from pprint import pprint
import inspect

# default values (used in __init__ and add_options)
_def_samples_per_symbol = 2
_def_bt = 0.35
_def_verbose = False
_def_log = False

_def_gain_mu = None
_def_mu = 0.5
_def_freq_error = 0.0
_def_omega_relative_limit = 0.005


# /////////////////////////////////////////////////////////////////////////////
#                              GMSK modulator
# /////////////////////////////////////////////////////////////////////////////

class gmsk_mod(gr.hier_block):

    def __init__(self, fg,
                 samples_per_symbol=_def_samples_per_symbol,
                 bt=_def_bt,
                 verbose=_def_verbose,
                 log=_def_log):
        """
	Hierarchical block for Gaussian Minimum Shift Key (GMSK)
	modulation.

	The input is a byte stream (unsigned char) and the
	output is the complex modulated signal at baseband.

	@param fg: flow graph
	@type fg: flow graph
	@param samples_per_symbol: samples per baud >= 2
	@type samples_per_symbol: integer
	@param bt: Gaussian filter bandwidth * symbol time
	@type bt: float
        @param verbose: Print information about modulator?
        @type verbose: bool
        @param debug: Print modualtion data to files?
        @type debug: bool       
	"""

        self._fg = fg
        self._samples_per_symbol = samples_per_symbol
        self._bt = bt

        if not isinstance(samples_per_symbol, int) or samples_per_symbol < 2:
            raise TypeError, ("samples_per_symbol must be an integer >= 2, is %r" % (samples_per_symbol,))

	ntaps = 4 * samples_per_symbol			# up to 3 bits in filter at once
	sensitivity = (pi / 2) / samples_per_symbol	# phase change per bit = pi / 2

	# Turn it into NRZ data.
	self.nrz = gr.bytes_to_syms()

	# Form Gaussian filter
        # Generate Gaussian response (Needs to be convolved with window below).
	self.gaussian_taps = gr.firdes.gaussian(
		1,		       # gain
		samples_per_symbol,    # symbol_rate
		bt,		       # bandwidth * symbol time
		ntaps	               # number of taps
		)

	self.sqwave = (1,) * samples_per_symbol       # rectangular window
	self.taps = numpy.convolve(numpy.array(self.gaussian_taps),numpy.array(self.sqwave))
	self.gaussian_filter = gr.interp_fir_filter_fff(samples_per_symbol, self.taps)

	# FM modulation
	self.fmmod = gr.frequency_modulator_fc(sensitivity)
		
        if verbose:
            self._print_verbage()
         
        if log:
            self._setup_logging()

	# Connect & Initialize base class
	self._fg.connect(self.nrz, self.gaussian_filter, self.fmmod)
	gr.hier_block.__init__(self, self._fg, self.nrz, self.fmmod)

    def samples_per_symbol(self):
        return self._samples_per_symbol

    def bits_per_symbol(self=None):     # staticmethod that's also callable on an instance
        return 1
    bits_per_symbol = staticmethod(bits_per_symbol)      # make it a static method.


    def _print_verbage(self):
        print "bits per symbol = %d" % self.bits_per_symbol()
        print "Gaussian filter bt = %.2f" % self._bt


    def _setup_logging(self):
        print "Modulation logging turned on."
        self._fg.connect(self.nrz,
                         gr.file_sink(gr.sizeof_float, "nrz.dat"))
        self._fg.connect(self.gaussian_filter,
                         gr.file_sink(gr.sizeof_float, "gaussian_filter.dat"))
        self._fg.connect(self.fmmod,
                         gr.file_sink(gr.sizeof_gr_complex, "fmmod.dat"))


    def add_options(parser):
        """
        Adds GMSK modulation-specific options to the standard parser
        """
        parser.add_option("", "--bt", type="float", default=_def_bt,
                          help="set bandwidth-time product [default=%default] (GMSK)")
    add_options=staticmethod(add_options)


    def extract_kwargs_from_options(options):
        """
        Given command line options, create dictionary suitable for passing to __init__
        """
        return modulation_utils.extract_kwargs_from_options(gmsk_mod.__init__,
                                                            ('self', 'fg'), options)
    extract_kwargs_from_options=staticmethod(extract_kwargs_from_options)



# /////////////////////////////////////////////////////////////////////////////
#                            GMSK demodulator
# /////////////////////////////////////////////////////////////////////////////

class gmsk_demod(gr.hier_block):

    def __init__(self, fg,
                 samples_per_symbol=_def_samples_per_symbol,
                 gain_mu=_def_gain_mu,
                 mu=_def_mu,
                 omega_relative_limit=_def_omega_relative_limit,
                 freq_error=_def_freq_error,
                 verbose=_def_verbose,
                 log=_def_log):
        """
	Hierarchical block for Gaussian Minimum Shift Key (GMSK)
	demodulation.

	The input is the complex modulated signal at baseband.
	The output is a stream of bits packed 1 bit per byte (the LSB)

	@param fg: flow graph
	@type fg: flow graph
	@param samples_per_symbol: samples per baud
	@type samples_per_symbol: integer
        @param verbose: Print information about modulator?
        @type verbose: bool
        @param log: Print modualtion data to files?
        @type log: bool 

        Clock recovery parameters.  These all have reasonble defaults.
        
        @param gain_mu: controls rate of mu adjustment
        @type gain_mu: float
        @param mu: fractional delay [0.0, 1.0]
        @type mu: float
        @param omega_relative_limit: sets max variation in omega
        @type omega_relative_limit: float, typically 0.000200 (200 ppm)
        @param freq_error: bit rate error as a fraction
        @param float
	"""

        self._fg = fg
        self._samples_per_symbol = samples_per_symbol
        self._gain_mu = gain_mu
        self._mu = mu
        self._omega_relative_limit = omega_relative_limit
        self._freq_error = freq_error
        
        if samples_per_symbol < 2:
            raise TypeError, "samples_per_symbol >= 2, is %f" % samples_per_symbol

        self._omega = samples_per_symbol*(1+self._freq_error)

        if not self._gain_mu:
            self._gain_mu = 0.175
            
	self._gain_omega = .25 * self._gain_mu * self._gain_mu        # critically damped

	# Demodulate FM
	sensitivity = (pi / 2) / samples_per_symbol
	self.fmdemod = gr.quadrature_demod_cf(1.0 / sensitivity)

	# the clock recovery block tracks the symbol clock and resamples as needed.
	# the output of the block is a stream of soft symbols (float)
	self.clock_recovery = gr.clock_recovery_mm_ff(self._omega, self._gain_omega,
                                                      self._mu, self._gain_mu,
                                                      self._omega_relative_limit)

        # slice the floats at 0, outputting 1 bit (the LSB of the output byte) per sample
        self.slicer = gr.binary_slicer_fb()

        if verbose:
            self._print_verbage()
         
        if log:
            self._setup_logging()

	# Connect & Initialize base class
	self._fg.connect(self.fmdemod, self.clock_recovery, self.slicer)
	gr.hier_block.__init__(self, self._fg, self.fmdemod, self.slicer)

    def samples_per_symbol(self):
        return self._samples_per_symbol

    def bits_per_symbol(self=None):   # staticmethod that's also callable on an instance
        return 1
    bits_per_symbol = staticmethod(bits_per_symbol)      # make it a static method.


    def _print_verbage(self):
        print "bits per symbol = %d" % self.bits_per_symbol()
        print "M&M clock recovery omega = %f" % self._omega
        print "M&M clock recovery gain mu = %f" % self._gain_mu
        print "M&M clock recovery mu = %f" % self._mu
        print "M&M clock recovery omega rel. limit = %f" % self._omega_relative_limit
        print "frequency error = %f" % self._freq_error


    def _setup_logging(self):
        print "Demodulation logging turned on."
        self._fg.connect(self.fmdemod,
                        gr.file_sink(gr.sizeof_float, "fmdemod.dat"))
        self._fg.connect(self.clock_recovery,
                        gr.file_sink(gr.sizeof_float, "clock_recovery.dat"))
        self._fg.connect(self.slicer,
                        gr.file_sink(gr.sizeof_char, "slicer.dat"))

    def add_options(parser):
        """
        Adds GMSK demodulation-specific options to the standard parser
        """
        parser.add_option("", "--gain-mu", type="float", default=_def_gain_mu,
                          help="M&M clock recovery gain mu [default=%default] (GMSK/PSK)")
        parser.add_option("", "--mu", type="float", default=_def_mu,
                          help="M&M clock recovery mu [default=%default] (GMSK/PSK)")
        parser.add_option("", "--omega-relative-limit", type="float", default=_def_omega_relative_limit,
                          help="M&M clock recovery omega relative limit [default=%default] (GMSK/PSK)")
        parser.add_option("", "--freq-error", type="float", default=_def_freq_error,
                          help="M&M clock recovery frequency error [default=%default] (GMSK)")
    add_options=staticmethod(add_options)

    def extract_kwargs_from_options(options):
        """
        Given command line options, create dictionary suitable for passing to __init__
        """
        return modulation_utils.extract_kwargs_from_options(gmsk_demod.__init__,
                                                            ('self', 'fg'), options)
    extract_kwargs_from_options=staticmethod(extract_kwargs_from_options)


#
# Add these to the mod/demod registry
#
modulation_utils.add_type_1_mod('gmsk', gmsk_mod)
modulation_utils.add_type_1_demod('gmsk', gmsk_demod)
