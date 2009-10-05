#!/usr/bin/env python
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

from gnuradio import gr, gr_unittest
import random

class test_packing(gr_unittest.TestCase):

    def setUp(self):
        self.fg = gr.flow_graph ()

    def tearDown(self):
        self.fg = None

    def test_001(self):
        """
        Test stream_to_streams.
        """
        src_data = (0x80,)
        expected_results = (1,0,0,0,0,0,0,0)
        src = gr.vector_source_b(src_data,False)
        op = gr.packed_to_unpacked_bb(1, gr.GR_MSB_FIRST)
        self.fg.connect(src, op)
        
        dst = gr.vector_sink_b()
        self.fg.connect(op, dst)

        self.fg.run()

        self.assertEqual(expected_results, dst.data())

    def test_002(self):
        """
        Test stream_to_streams.
        """
        src_data = (0x80,)
        expected_results = (0,0,0,0,0,0,0, 1)
        src = gr.vector_source_b(src_data,False)
        op = gr.packed_to_unpacked_bb(1, gr.GR_LSB_FIRST)
        self.fg.connect(src, op)
        
        dst = gr.vector_sink_b()
        self.fg.connect(op, dst)

        self.fg.run()

        self.assertEqual(expected_results, dst.data())

    def test_003(self):
        """
        Test stream_to_streams.
        """
        src_data = (0x11,)
        expected_results = (4, 2)
        src = gr.vector_source_b(src_data,False)
        op = gr.packed_to_unpacked_bb(3, gr.GR_LSB_FIRST)
        self.fg.connect(src, op)
        
        dst = gr.vector_sink_b()
        self.fg.connect(op, dst)

        self.fg.run()

        self.assertEqual(expected_results, dst.data())

    def test_004(self):
        """
        Test stream_to_streams.
        """
        src_data = (0x11,)
        expected_results = (0, 4)
        src = gr.vector_source_b(src_data,False)
        op = gr.packed_to_unpacked_bb(3, gr.GR_MSB_FIRST)
        self.fg.connect(src, op)
        
        dst = gr.vector_sink_b()
        self.fg.connect(op, dst)

        self.fg.run()

        self.assertEqual(expected_results, dst.data())

    def test_005(self):
        """
        Test stream_to_streams.
        """
        src_data = (1,0,0,0,0,0,1,0,0,1,0,1,1,0,1,0)
        expected_results =  (0x82,0x5a)
        src = gr.vector_source_b(src_data,False)
        op = gr.unpacked_to_packed_bb(1, gr.GR_MSB_FIRST)
        self.fg.connect(src, op)
        
        dst = gr.vector_sink_b()
        self.fg.connect(op, dst)

        self.fg.run()

        self.assertEqual(expected_results, dst.data())

    def test_006(self):
        """
        Test stream_to_streams.
        """
        src_data = (0,1,0,0,0,0,0,1,0,1,0,1,1,0,1,0)
        expected_results =  (0x82,0x5a)
        src = gr.vector_source_b(src_data,False)
        op = gr.unpacked_to_packed_bb(1, gr.GR_LSB_FIRST)
        self.fg.connect(src, op)
        
        dst = gr.vector_sink_b()
        self.fg.connect(op, dst)

        self.fg.run()

        self.assertEqual(expected_results, dst.data())


    def test_007(self):
        """
        Test stream_to_streams.
        """
        src_data = (4, 2, 0,0,0)
        expected_results = (0x11,)
        src = gr.vector_source_b(src_data,False)
        op = gr.unpacked_to_packed_bb(3, gr.GR_LSB_FIRST)
        self.fg.connect(src, op)
        
        dst = gr.vector_sink_b()
        self.fg.connect(op, dst)

        self.fg.run()

        self.assertEqual(expected_results, dst.data())

    def test_008(self):
        """
        Test stream_to_streams.
        """
        src_data = (0, 4, 2,0,0)
        expected_results = (0x11,)
        src = gr.vector_source_b(src_data,False)
        op = gr.unpacked_to_packed_bb(3, gr.GR_MSB_FIRST)
        self.fg.connect(src, op)
        
        dst = gr.vector_sink_b()
        self.fg.connect(op, dst)

        self.fg.run()

        self.assertEqual(expected_results, dst.data())

    def test_009(self):
        """
        Test stream_to_streams.
        """

        random.seed(0)
        src_data = []
        for i in xrange(202):
            src_data.append((random.randint(0,255)))
        src_data = tuple(src_data)
        expected_results = src_data

        src = gr.vector_source_b(tuple(src_data),False)
        op1 = gr.packed_to_unpacked_bb(3, gr.GR_MSB_FIRST)
        op2 = gr.unpacked_to_packed_bb(3, gr.GR_MSB_FIRST)
        self.fg.connect(src, op1, op2)
        
        dst = gr.vector_sink_b()
        self.fg.connect(op2, dst)

        self.fg.run()
        
        self.assertEqual(expected_results[0:201], dst.data())

    def test_010(self):
        """
        Test stream_to_streams.
        """

        random.seed(0)
        src_data = []
        for i in xrange(56):
            src_data.append((random.randint(0,255)))
        src_data = tuple(src_data)
        expected_results = src_data
        src = gr.vector_source_b(tuple(src_data),False)
        op1 = gr.packed_to_unpacked_bb(7, gr.GR_MSB_FIRST)
        op2 = gr.unpacked_to_packed_bb(7, gr.GR_MSB_FIRST)
        self.fg.connect(src, op1, op2)
        dst = gr.vector_sink_b()
        self.fg.connect(op2, dst)

        self.fg.run()
        self.assertEqual(expected_results[0:201], dst.data())

    def test_011(self):
        """
        Test stream_to_streams.
        """

        random.seed(0)
        src_data = []
        for i in xrange(56):
            src_data.append((random.randint(0,255)))
        src_data = tuple(src_data)
        expected_results = src_data
        src = gr.vector_source_b(tuple(src_data),False)
        op1 = gr.packed_to_unpacked_bb(7, gr.GR_LSB_FIRST)
        op2 = gr.unpacked_to_packed_bb(7, gr.GR_LSB_FIRST)
        self.fg.connect(src, op1, op2)
        dst = gr.vector_sink_b()
        self.fg.connect(op2, dst)

        self.fg.run()
        self.assertEqual(expected_results[0:201], dst.data())
        

    # tests on shorts
    
    def test_100a(self):
        """
        test short version
        """
        random.seed(0)
        src_data = []
        for i in xrange(100):
            src_data.append((random.randint(-2**15,2**15-1)))
        src_data = tuple(src_data)
        expected_results = src_data
        src = gr.vector_source_s(tuple(src_data),False)
        op1 = gr.packed_to_unpacked_ss(1, gr.GR_MSB_FIRST)
        op2 = gr.unpacked_to_packed_ss(1, gr.GR_MSB_FIRST)
        self.fg.connect(src, op1, op2)
        dst = gr.vector_sink_s()
        self.fg.connect(op2, dst)

        self.fg.run()
        self.assertEqual(expected_results, dst.data())

    def test_100b(self):
        """
        test short version
        """
        random.seed(0)
        src_data = []
        for i in xrange(100):
            src_data.append((random.randint(-2**15,2**15-1)))
        src_data = tuple(src_data)
        expected_results = src_data
        src = gr.vector_source_s(tuple(src_data),False)
        op1 = gr.packed_to_unpacked_ss(1, gr.GR_LSB_FIRST)
        op2 = gr.unpacked_to_packed_ss(1, gr.GR_LSB_FIRST)
        self.fg.connect(src, op1, op2)
        dst = gr.vector_sink_s()
        self.fg.connect(op2, dst)

        self.fg.run()
        self.assertEqual(expected_results, dst.data())

    def test_101a(self):
        """
        test short version
        """
        random.seed(0)
        src_data = []
        for i in xrange(100):
            src_data.append((random.randint(-2**15,2**15-1)))
        src_data = tuple(src_data)
        expected_results = src_data
        src = gr.vector_source_s(tuple(src_data),False)
        op1 = gr.packed_to_unpacked_ss(8, gr.GR_MSB_FIRST)
        op2 = gr.unpacked_to_packed_ss(8, gr.GR_MSB_FIRST)
        self.fg.connect(src, op1, op2)
        dst = gr.vector_sink_s()
        self.fg.connect(op2, dst)

        self.fg.run()
        self.assertEqual(expected_results, dst.data())

    def test_101b(self):
        """
        test short version
        """
        random.seed(0)
        src_data = []
        for i in xrange(100):
            src_data.append((random.randint(-2**15,2**15-1)))
        src_data = tuple(src_data)
        expected_results = src_data
        src = gr.vector_source_s(tuple(src_data),False)
        op1 = gr.packed_to_unpacked_ss(8, gr.GR_LSB_FIRST)
        op2 = gr.unpacked_to_packed_ss(8, gr.GR_LSB_FIRST)
        self.fg.connect(src, op1, op2)
        dst = gr.vector_sink_s()
        self.fg.connect(op2, dst)

        self.fg.run()
        self.assertEqual(expected_results, dst.data())

    # tests on ints
    
    def test_200a(self):
        """
        test int version
        """
        random.seed(0)
        src_data = []
        for i in xrange(100):
            src_data.append((random.randint(-2**31,2**31-1)))
        src_data = tuple(src_data)
        expected_results = src_data
        src = gr.vector_source_i(tuple(src_data),False)
        op1 = gr.packed_to_unpacked_ii(1, gr.GR_MSB_FIRST)
        op2 = gr.unpacked_to_packed_ii(1, gr.GR_MSB_FIRST)
        self.fg.connect(src, op1, op2)
        dst = gr.vector_sink_i()
        self.fg.connect(op2, dst)

        self.fg.run()
        self.assertEqual(expected_results, dst.data())

    def test_200b(self):
        """
        test int version
        """
        random.seed(0)
        src_data = []
        for i in xrange(100):
            src_data.append((random.randint(-2**31,2**31-1)))
        src_data = tuple(src_data)
        expected_results = src_data
        src = gr.vector_source_i(tuple(src_data),False)
        op1 = gr.packed_to_unpacked_ii(1, gr.GR_LSB_FIRST)
        op2 = gr.unpacked_to_packed_ii(1, gr.GR_LSB_FIRST)
        self.fg.connect(src, op1, op2)
        dst = gr.vector_sink_i()
        self.fg.connect(op2, dst)

        self.fg.run()
        self.assertEqual(expected_results, dst.data())

    def test_201a(self):
        """
        test int version
        """
        random.seed(0)
        src_data = []
        for i in xrange(100):
            src_data.append((random.randint(-2**31,2**31-1)))
        src_data = tuple(src_data)
        expected_results = src_data
        src = gr.vector_source_i(tuple(src_data),False)
        op1 = gr.packed_to_unpacked_ii(8, gr.GR_MSB_FIRST)
        op2 = gr.unpacked_to_packed_ii(8, gr.GR_MSB_FIRST)
        self.fg.connect(src, op1, op2)
        dst = gr.vector_sink_i()
        self.fg.connect(op2, dst)

        self.fg.run()
        self.assertEqual(expected_results, dst.data())

    def test_201b(self):
        """
        test int version
        """
        random.seed(0)
        src_data = []
        for i in xrange(100):
            src_data.append((random.randint(-2**31,2**31-1)))
        src_data = tuple(src_data)
        expected_results = src_data
        src = gr.vector_source_i(tuple(src_data),False)
        op1 = gr.packed_to_unpacked_ii(8, gr.GR_LSB_FIRST)
        op2 = gr.unpacked_to_packed_ii(8, gr.GR_LSB_FIRST)
        self.fg.connect(src, op1, op2)
        dst = gr.vector_sink_i()
        self.fg.connect(op2, dst)

        self.fg.run()
        self.assertEqual(expected_results, dst.data())


if __name__ == '__main__':
   gr_unittest.main ()
        
