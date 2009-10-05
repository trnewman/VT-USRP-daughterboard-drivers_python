#!/usr/bin/env python
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
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

from gnuradio import gr, gr_unittest
import math

def sincos(x):
    return  math.cos(x) + math.sin(x) * 1j

class test_bytes_to_syms (gr_unittest.TestCase):

    def setUp (self):
        self.fg = gr.flow_graph ()

    def tearDown (self):
        self.fg = None

    def test_bytes_to_syms_001 (self):
        src_data = (0x01, 0x80, 0x03)
        expected_result = (-1, -1, -1, -1, -1, -1, -1, +1,
                           +1, -1, -1, -1, -1, -1, -1, -1,
                           -1, -1, -1, -1, -1, -1, +1, +1)
        src = gr.vector_source_b (src_data)
        op = gr.bytes_to_syms ()
        dst = gr.vector_sink_f ()
        self.fg.connect (src, op)
        self.fg.connect (op, dst)
        self.fg.run ()
        result_data = dst.data ()
        self.assertEqual (expected_result, result_data)

    def test_simple_framer (self):
        src_data = (0x00, 0x11, 0x22, 0x33, 
                    0x44, 0x55, 0x66, 0x77,
                    0x88, 0x99, 0xaa, 0xbb, 
                    0xcc, 0xdd, 0xee, 0xff)

        expected_result = (
            0xac, 0xdd, 0xa4, 0xe2, 0xf2, 0x8c, 0x20, 0xfc, 0x00, 0x00, 0x11, 0x22, 0x33, 0x55,
            0xac, 0xdd, 0xa4, 0xe2, 0xf2, 0x8c, 0x20, 0xfc, 0x01, 0x44, 0x55, 0x66, 0x77, 0x55,
            0xac, 0xdd, 0xa4, 0xe2, 0xf2, 0x8c, 0x20, 0xfc, 0x02, 0x88, 0x99, 0xaa, 0xbb, 0x55,
            0xac, 0xdd, 0xa4, 0xe2, 0xf2, 0x8c, 0x20, 0xfc, 0x03, 0xcc, 0xdd, 0xee, 0xff, 0x55)

        src = gr.vector_source_b (src_data)
        op = gr.simple_framer (4)
        dst = gr.vector_sink_b ()
        self.fg.connect (src, op)
        self.fg.connect (op, dst)
        self.fg.run ()
        result_data = dst.data ()
        self.assertEqual (expected_result, result_data)
        

if __name__ == '__main__':
    gr_unittest.main ()
        
