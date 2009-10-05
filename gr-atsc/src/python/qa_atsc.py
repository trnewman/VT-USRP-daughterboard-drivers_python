#!/usr/bin/env python
#
# Copyright 2004,2006 Free Software Foundation, Inc.
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

from gnuradio import gr, gr_unittest
import atsc                    # qa code needs to run without being installed
#from gnuradio import atsc
from atsc_utils import *
import sys


class memoize(object):
    def __init__(self, thunk):
        self.thunk = thunk
        self.cached = False
        self.value = None

    def __call__(self):
        if self.cached:
            return self.value
        self.value = self.thunk()
        self.cached = True
        return self.value


"""
Make a fake transport stream that's big enough for our purposes.
We generate 8 full fields.  This is relatively expensive.  It
takes about 2 seconds to execute.  
"""
make_transport_stream = \
    memoize(lambda : tuple(make_fake_transport_stream_packet(8 * atsc.ATSC_DSEGS_PER_FIELD)))


def pad_transport_stream(src):
    """
    An MPEG transport stream packet is 188 bytes long.  Internally we use a packet
    that is 256 bytes long to help with buffer alignment.  This function adds the
    appropriate trailing padding to convert each packet from 188 to 256 bytes.
    """
    return pad_stream(src, atsc.sizeof_atsc_mpeg_packet, atsc.sizeof_atsc_mpeg_packet_pad)


def depad_transport_stream(src):
    """
    An MPEG transport stream packet is 188 bytes long.  Internally we use a packet
    that is 256 bytes long to help with buffer alignment.  This function removes the
    trailing padding to convert each packet from 256 back to 188 bytes.
    """
    return depad_stream(src, atsc.sizeof_atsc_mpeg_packet, atsc.sizeof_atsc_mpeg_packet_pad)


class vector_source_ts(gr.hier_block):
    """
    MPEG Transport stream source for testing.
    """
    def __init__(self, fg, ts):
        """
        Pad tranport stream packets to 256 bytes and reformat appropriately.
        
        @param fg: flow graph
        @param ts: MPEG transport stream.
        @type  ts: sequence of ints in [0,255]; len(ts) % 188 == 0
        """
        src = gr.vector_source_b(pad_transport_stream(ts))
        s2v = gr.stream_to_vector(gr.sizeof_char, atsc.sizeof_atsc_mpeg_packet)
        fg.connect(src, s2v)
        gr.hier_block.__init__(self, fg, None, s2v)


class vector_sink_ts(gr.hier_block):
    """
    MPEG Transport stream sink for testing.
    """
    def __init__(self, fg):
        """
        @param fg: flow graph
        """
        v2s = gr.vector_to_stream(gr.sizeof_char, atsc.sizeof_atsc_mpeg_packet)
        self.sink = gr.vector_sink_b()
        fg.connect(v2s, self.sink)
        gr.hier_block.__init__(self, fg, v2s, None)

    def data(self):
        """
        Extracts tranport stream from sink and returns it to python.

        Depads tranport stream packets from 256 back to 188 bytes.
        @rtype: tuple of ints in [0,255]; len(result) % 188 == 0
        """
        return tuple(depad_transport_stream(self.sink.data()))



class qa_atsc(gr_unittest.TestCase):

    def setUp(self):
        self.fg = gr.flow_graph()

    def tearDown(self):
        self.fg = None


    # The tests are run in alphabetical order

    def test_loopback_000(self):
        """
        Loopback randomizer to derandomizer
        """
        src_data = make_transport_stream()
        expected_result = src_data

        src = vector_source_ts(self.fg, src_data)
        rand = atsc.randomizer()
        derand = atsc.derandomizer()
        dst = vector_sink_ts(self.fg)
        self.fg.connect(src, rand, derand, dst)
        self.fg.run ()
        result_data = dst.data ()
        self.assertEqual (expected_result, result_data)

    def test_loopback_001(self):
        """
        Loopback randomizer/rs_encoder to rs_decoder/derandomizer
        """
        src_data = make_transport_stream()
        expected_result = src_data

        src = vector_source_ts(self.fg, src_data)
        rand = atsc.randomizer()
        rs_enc = atsc.rs_encoder()
        rs_dec = atsc.rs_decoder()
        derand = atsc.derandomizer()
        dst = vector_sink_ts(self.fg)
        self.fg.connect(src, rand, rs_enc, rs_dec, derand, dst)
        self.fg.run ()
        result_data = dst.data ()
        self.assertEqual (expected_result, result_data)

    def test_loopback_002(self):
        """
        Loopback randomizer/rs_encoder/interleaver to
	deinterleaver/rs_decoder/derandomizer 
        """
        src_data = make_transport_stream()
	interleaver_delay = 52
        expected_result = src_data[0:len(src_data)-(interleaver_delay*atsc.ATSC_MPEG_PKT_LENGTH)]

        src = vector_source_ts(self.fg, src_data)
        rand = atsc.randomizer()
        rs_enc = atsc.rs_encoder()
	inter = atsc.interleaver()
	deinter = atsc.deinterleaver()
        rs_dec = atsc.rs_decoder()
        derand = atsc.derandomizer()
        dst = vector_sink_ts(self.fg)
        self.fg.connect(src, rand, rs_enc, inter, deinter, rs_dec, derand, dst)
        self.fg.run ()
        result_data = dst.data ()
	result_data = result_data[(interleaver_delay*atsc.ATSC_MPEG_PKT_LENGTH):len(result_data)]
        self.assertEqual (expected_result, result_data)


    def test_loopback_003(self):
        """
        Loopback randomizer/rs_encoder/interleaver/trellis_encoder
	via ds_to_softds to
	viterbi_decoder/deinterleaver/rs_decoder/derandomizer 
        """
        src_data = make_transport_stream()
	interleaver_delay = 52
	viterbi_delay = 12
        expected_result = src_data[0:len(src_data)-((interleaver_delay+viterbi_delay)*atsc.ATSC_MPEG_PKT_LENGTH)]

        src = vector_source_ts(self.fg, src_data)
        rand = atsc.randomizer()
        rs_enc = atsc.rs_encoder()
        inter = atsc.interleaver()
	trellis = atsc.trellis_encoder()
	softds = atsc.ds_to_softds()
	viterbi = atsc.viterbi_decoder()
        deinter = atsc.deinterleaver()
        rs_dec = atsc.rs_decoder()
        derand = atsc.derandomizer()
        dst = vector_sink_ts(self.fg)
	self.fg.connect(src, rand, rs_enc, inter, trellis, softds, viterbi, deinter, rs_dec, derand, dst)
        self.fg.run ()
        result_data = dst.data ()[((interleaver_delay+viterbi_delay)*atsc.ATSC_MPEG_PKT_LENGTH):len(dst.data())]
        self.assertEqual (expected_result, result_data)

        
if __name__ == '__main__':
    gr_unittest.main()






