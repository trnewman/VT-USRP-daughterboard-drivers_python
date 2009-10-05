#!/usr/bin/env python
#
# Copyright 2006,2007 Free Software Foundation, Inc.
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
from gnuradio.eng_option import eng_option
from optparse import OptionParser

class dial_tone_source(gr.top_block):
    def __init__(self, src, dst, port, pkt_size, sample_rate):
        gr.top_block.__init__(self, "dial_tone_source")

        amplitude = 0.3
        src0 = gr.sig_source_f (sample_rate, gr.GR_SIN_WAVE, 350, amplitude)
        src1 = gr.sig_source_f (sample_rate, gr.GR_SIN_WAVE, 440, amplitude)
        add = gr.add_ff()

        # Throttle needed here to account for the other side's audio card sampling rate
	thr = gr.throttle(gr.sizeof_float, sample_rate)
	sink = gr.udp_sink(gr.sizeof_float, src, 0, dst, port, pkt_size)
	self.connect(src0, (add, 0))
	self.connect(src1, (add, 1))
	self.connect(add, thr, sink)

if __name__ == '__main__':
    parser = OptionParser(option_class=eng_option)
    parser.add_option("", "--src-name", type="string", default="localhost",
                      help="local host name (domain name or IP address)")
    parser.add_option("", "--dst-name", type="string", default="localhost",
                      help="Remote host name (domain name or IP address")
    parser.add_option("", "--dst-port", type="int", default=65500,
                      help="port value to connect to")
    parser.add_option("", "--packet-size", type="int", default=1472,
                      help="packet size.")
    parser.add_option("-r", "--sample-rate", type="int", default=8000,
                      help="audio signal sample rate [default=%default]")
    (options, args) = parser.parse_args()
    if len(args) != 0:
        parser.print_help()
        raise SystemExit, 1

    # Create an instance of a hierarchical block
    top_block = dial_tone_source(options.src_name, options.dst_name, options.dst_port,
                                 options.packet_size, options.sample_rate)
    
    try:    
        # Run forever
        top_block.run()
    except KeyboardInterrupt:
        # Ctrl-C exits
        pass
    
