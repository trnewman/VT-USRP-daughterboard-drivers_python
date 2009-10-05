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

import sys
from gnuradio import gr, gru

def _generate_synthesis_taps(mpoints):
    return []   # FIXME


def _split_taps(taps, mpoints):
    assert (len(taps) % mpoints) == 0
    result = [list() for x in range(mpoints)]
    for i in xrange(len(taps)):
        (result[i % mpoints]).append(taps[i])
    return [tuple(x) for x in result]


class synthesis_filterbank(gr.hier_block):
    """
    Uniformly modulated polyphase DFT filter bank: synthesis

    See http://cnx.rice.edu/content/m10424/latest
    """
    def __init__(self, fg, mpoints, taps=None):
        """
        Takes M complex streams in, produces single complex stream out
        that runs at M times the input sample rate

        @param fg:      flow_graph
        @param mpoints: number of freq bins/interpolation factor/subbands
        @param taps:    filter taps for subband filter

        The channel spacing is equal to the input sample rate.
        The total bandwidth and output sample rate are equal the input
        sample rate * nchannels.

        Output stream to frequency mapping:
        
          channel zero is at zero frequency.

          if mpoints is odd:
            
            Channels with increasing positive frequencies come from
            channels 1 through (N-1)/2.

            Channel (N+1)/2 is the maximum negative frequency, and
            frequency increases through N-1 which is one channel lower
            than the zero frequency.

          if mpoints is even:

            Channels with increasing positive frequencies come from
            channels 1 through (N/2)-1.

            Channel (N/2) is evenly split between the max positive and
            negative bins.

            Channel (N/2)+1 is the maximum negative frequency, and
            frequency increases through N-1 which is one channel lower
            than the zero frequency.

            Channels near the frequency extremes end up getting cut
            off by subsequent filters and therefore have diminished
            utility.
        """
        item_size = gr.sizeof_gr_complex

        if taps is None:
            taps = _generate_synthesis_taps(mpoints)

        # pad taps to multiple of mpoints
        r = len(taps) % mpoints
        if r != 0:
            taps = taps + (mpoints - r) * (0,)

        # split in mpoints separate set of taps
        sub_taps = _split_taps(taps, mpoints)

        self.ss2v = gr.streams_to_vector(item_size, mpoints)
        self.ifft = gr.fft_vcc(mpoints, False, [])
        self.v2ss = gr.vector_to_streams(item_size, mpoints)
        # mpoints filters go in here...
        self.ss2s = gr.streams_to_stream(item_size, mpoints)

        fg.connect(self.ss2v, self.ifft, self.v2ss)

        # build mpoints fir filters...
        for i in range(mpoints):
            f = gr.fft_filter_ccc(1, sub_taps[i])
            fg.connect((self.v2ss, i), f)
            fg.connect(f, (self.ss2s, i))

        gr.hier_block.__init__(self, fg, self.ss2v, self.ss2s)


class analysis_filterbank(gr.hier_block):
    """
    Uniformly modulated polyphase DFT filter bank: analysis

    See http://cnx.rice.edu/content/m10424/latest
    """
    def __init__(self, fg, mpoints, taps=None):
        """
        Takes 1 complex stream in, produces M complex streams out
        that runs at 1/M times the input sample rate

        @param fg:      flow_graph
        @param mpoints: number of freq bins/interpolation factor/subbands
        @param taps:    filter taps for subband filter

        Same channel to frequency mapping as described above.
        """
        item_size = gr.sizeof_gr_complex

        if taps is None:
            taps = _generate_synthesis_taps(mpoints)

        # pad taps to multiple of mpoints
        r = len(taps) % mpoints
        if r != 0:
            taps = taps + (mpoints - r) * (0,)
        
        # split in mpoints separate set of taps
        sub_taps = _split_taps(taps, mpoints)

        # print >> sys.stderr, "mpoints =", mpoints, "len(sub_taps) =", len(sub_taps) 
        
        self.s2ss = gr.stream_to_streams(item_size, mpoints)
        # filters here
        self.ss2v = gr.streams_to_vector(item_size, mpoints)
        self.fft = gr.fft_vcc(mpoints, True, [])
        self.v2ss = gr.vector_to_streams(item_size, mpoints)

        # build mpoints fir filters...
        for i in range(mpoints):
            f = gr.fft_filter_ccc(1, sub_taps[mpoints-i-1])
            fg.connect((self.s2ss, i), f)
            fg.connect(f, (self.ss2v, i))

        fg.connect(self.ss2v, self.fft, self.v2ss)
        gr.hier_block.__init__(self, fg, self.s2ss, self.v2ss)
