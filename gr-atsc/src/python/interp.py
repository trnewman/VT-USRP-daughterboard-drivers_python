#!/usr/bin/env /usr/bin/python
#
# Copyright 2004 Free Software Foundation, Inc.
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
# the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
# 
# This module starts the atsc processing chain taking the captured
# off-air signal created with:
#
#  usrp_rx_cfile.py -R <side with tuner, a or b>
#                   -d 10     set decimation to get signal at 6.4e6 rate
#                   -f <center of tv signal channel freq>
#                   -g <appropriate gain for best signal / noise>
# 
# All this module does is multiply the sample rate by 3, from 6.4e6 to
# 19.2e6 complex samples / sec, then lowpass filter with a cutoff of 3.2MHz
# and a transition band width of .5MHz.  Center of the tv channels is
# then at 0 with edges at -3.2MHz and 3.2MHz.

from gnuradio import gr
import sys

def graph (args):

    nargs = len (args)
    if nargs == 1:
	infile = args[0]
    else:
	sys.stderr.write('usage: interp.py input_file\n')
	sys.exit (1)

    sampling_freq = 6400000

    fg = gr.flow_graph ()

    src0 = gr.file_source (gr.sizeof_gr_complex,infile)
    src1 = gr.sig_source_c (sampling_freq, gr.GR_CONST_WAVE, 1, 0)
    src2 = gr.sig_source_c (sampling_freq, gr.GR_CONST_WAVE, 1, 0)

    interlv = gr.interleave(gr.sizeof_gr_complex)

    lp_coeffs = gr.firdes.low_pass ( 3, 19.2e6, 3.2e6, .5e6, gr.firdes.WIN_HAMMING )
    lp = gr.fir_filter_ccf ( 1, lp_coeffs )

    file = gr.file_sink(gr.sizeof_gr_complex,"/tmp/atsc_pipe_1")

    fg.connect( src0, (interlv, 0) )
    fg.connect( src1, (interlv, 1) )
    fg.connect( src2, (interlv, 2) )
    fg.connect( interlv, lp, file )

    fg.start()
    raw_input ('Head End: Press Enter to stop')
    fg.stop()

if __name__ == '__main__':
    graph (sys.argv[1:])


