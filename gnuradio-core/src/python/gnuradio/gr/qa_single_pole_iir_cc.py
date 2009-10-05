#!/usr/bin/env python
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

from gnuradio import gr, gr_unittest

class test_single_pole_iir_cc(gr_unittest.TestCase):

    def setUp (self):
        self.fg = gr.flow_graph ()

    def tearDown (self):
        self.fg = None

    def test_001(self):
        src_data = (0+0j, 1000+1000j, 2000+2000j, 3000+3000j, 4000+4000j, 5000+5000j)
        expected_result = src_data
        src = gr.vector_source_c(src_data)
        op = gr.single_pole_iir_filter_cc (1.0)
        dst = gr.vector_sink_c()
        self.fg.connect (src, op, dst)
        self.fg.run()
        result_data = dst.data()
        self.assertComplexTuplesAlmostEqual (expected_result, result_data)

    def test_002(self):
        src_data = (complex(0,0), complex(1000,-1000), complex(2000,-2000), complex(3000,-3000), complex(4000,-4000), complex(5000,-5000))
        expected_result = (complex(0,0), complex(125,-125), complex(359.375,-359.375), complex(689.453125,-689.453125), complex(1103.271484,-1103.271484), complex(1590.36255,-1590.36255))
        src = gr.vector_source_c(src_data)
        op = gr.single_pole_iir_filter_cc (0.125)
        dst = gr.vector_sink_c()
        self.fg.connect (src, op, dst)
        self.fg.run()
        result_data = dst.data()
        self.assertComplexTuplesAlmostEqual (expected_result, result_data, 3)

    def test_003(self):
        block_size = 2
        src_data = (complex(0,0), complex(1000,-1000), complex(2000,-2000), complex(3000,-3000), complex(4000,-4000), complex(5000,-5000))
        expected_result = (complex(0,0), complex(125,-125), complex(250,-250), complex(484.375,-484.375), complex(718.75,-718.75), complex(1048.828125,-1048.828125))
        src = gr.vector_source_c(src_data)
        s2p = gr.serial_to_parallel(gr.sizeof_gr_complex, block_size)
        op = gr.single_pole_iir_filter_cc (0.125, block_size)
        p2s = gr.parallel_to_serial(gr.sizeof_gr_complex, block_size)
        dst = gr.vector_sink_c()
        self.fg.connect (src, s2p, op, p2s, dst)
        self.fg.run()
        result_data = dst.data()
        self.assertComplexTuplesAlmostEqual (expected_result, result_data, 3)


if __name__ == '__main__':
    gr_unittest.main ()
        
