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

from gnuradio import gr, gr_unittest
import math


class test_sig_source (gr_unittest.TestCase):

    def setUp (self):
        self.fg = gr.flow_graph ()


    def tearDown (self):
        self.fg = None


    def test_001(self):
        fg = self.fg

        src1_data = (0,0.2,-0.3,0,12,0)
        src2_data = (0,0.0,3.0,0,10,0)
        src3_data = (0,0.0,3.0,0,1,0)

        src1 = gr.vector_source_f (src1_data)
        s2v1 = gr.stream_to_vector(gr.sizeof_float, len(src1_data))
        fg.connect( src1, s2v1 )

        src2 = gr.vector_source_f (src2_data)
        s2v2 = gr.stream_to_vector(gr.sizeof_float, len(src1_data))
        fg.connect( src2, s2v2 )

        src3 = gr.vector_source_f (src3_data)
        s2v3 = gr.stream_to_vector(gr.sizeof_float, len(src1_data))
        fg.connect( src3, s2v3 )

        dst1 = gr.vector_sink_s ()
        dst2 = gr.vector_sink_s ()
        argmax = gr.argmax_fs (len(src1_data))

        fg.connect (s2v1, (argmax, 0))
        fg.connect (s2v2, (argmax, 1))
        fg.connect (s2v3, (argmax, 2))

        fg.connect ((argmax,0), dst1)
        fg.connect ((argmax,1), dst2)

        fg.run ()
        index = dst1.data ()
        source = dst2.data ()
        self.assertEqual ( index, (4,))
        self.assertEqual ( source, (0,))



if __name__ == '__main__':
    gr_unittest.main ()

