#!/usr/bin/env python
#
# Copyright 2006 Free Software Foundation, Inc.
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
import random

def make_random_int_tuple(L, min, max):
    result = []
    for x in range(L):
        result.append(random.randint(min, max))
    return tuple(result)

    
class test_encoder (gr_unittest.TestCase):

    def setUp (self):
        self.fg = gr.flow_graph ()

    def tearDown (self):
        self.fg = None

    def test_diff_encdec_000(self):
        random.seed(0)
        modulus = 2
        src_data = make_random_int_tuple(1000, 0, modulus-1)
        expected_result = src_data
        src = gr.vector_source_b(src_data)
        enc = gr.diff_encoder_bb(modulus)
        dec = gr.diff_decoder_bb(modulus)
        dst = gr.vector_sink_b()
        self.fg.connect(src, enc, dec, dst)
        self.fg.run()               # run the graph and wait for it to finish
        actual_result = dst.data()  # fetch the contents of the sink
        self.assertEqual(expected_result, actual_result)

    def test_diff_encdec_001(self):
        random.seed(0)
        modulus = 4
        src_data = make_random_int_tuple(1000, 0, modulus-1)
        expected_result = src_data
        src = gr.vector_source_b(src_data)
        enc = gr.diff_encoder_bb(modulus)
        dec = gr.diff_decoder_bb(modulus)
        dst = gr.vector_sink_b()
        self.fg.connect(src, enc, dec, dst)
        self.fg.run()               # run the graph and wait for it to finish
        actual_result = dst.data()  # fetch the contents of the sink
        self.assertEqual(expected_result, actual_result)

    def test_diff_encdec_002(self):
        random.seed(0)
        modulus = 8
        src_data = make_random_int_tuple(40000, 0, modulus-1)
        expected_result = src_data
        src = gr.vector_source_b(src_data)
        enc = gr.diff_encoder_bb(modulus)
        dec = gr.diff_decoder_bb(modulus)
        dst = gr.vector_sink_b()
        self.fg.connect(src, enc, dec, dst)
        self.fg.run()               # run the graph and wait for it to finish
        actual_result = dst.data()  # fetch the contents of the sink
        self.assertEqual(expected_result, actual_result)

if __name__ == '__main__':
    gr_unittest.main ()

