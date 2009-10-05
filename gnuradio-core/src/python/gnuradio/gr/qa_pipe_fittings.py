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

if 0:
    import os
    print "pid =", os.getpid()
    raw_input("Attach, then press Enter to continue")


def calc_expected_result(src_data, n):
    assert (len(src_data) % n) == 0
    result = [list() for x in range(n)]
    #print "len(result) =", len(result)
    for i in xrange(len(src_data)):
        (result[i % n]).append(src_data[i])
    return [tuple(x) for x in result]


class test_pipe_fittings(gr_unittest.TestCase):

    def setUp(self):
        self.fg = gr.flow_graph ()

    def tearDown(self):
        self.fg = None

    def test_001(self):
        """
        Test stream_to_streams.
        """
        n = 8
        src_len = n * 8
        src_data = range(src_len)

        expected_results = calc_expected_result(src_data, n)
        #print "expected results: ", expected_results
        src = gr.vector_source_i(src_data)
        op = gr.stream_to_streams(gr.sizeof_int, n)
        self.fg.connect(src, op)
        
        dsts = []
        for i in range(n):
            dst = gr.vector_sink_i()
            self.fg.connect((op, i), (dst, 0))
            dsts.append(dst)

        self.fg.run()

        for d in range(n):
            self.assertEqual(expected_results[d], dsts[d].data())

    def test_002(self):
        """
        Test streams_to_stream (using stream_to_streams).
        """
        n = 8
        src_len = n * 8
        src_data = tuple(range(src_len))
        expected_results = src_data

        src = gr.vector_source_i(src_data)
        op1 = gr.stream_to_streams(gr.sizeof_int, n)
        op2 = gr.streams_to_stream(gr.sizeof_int, n)
        dst = gr.vector_sink_i()
        
        self.fg.connect(src, op1)
        for i in range(n):
            self.fg.connect((op1, i), (op2, i))
        self.fg.connect(op2, dst)
        
        self.fg.run()
        self.assertEqual(expected_results, dst.data())
        
    def test_003(self):
        """
        Test streams_to_vector (using stream_to_streams & vector_to_stream).
        """
        n = 8
        src_len = n * 8
        src_data = tuple(range(src_len))
        expected_results = src_data

        src = gr.vector_source_i(src_data)
        op1 = gr.stream_to_streams(gr.sizeof_int, n)
        op2 = gr.streams_to_vector(gr.sizeof_int, n)
        op3 = gr.vector_to_stream(gr.sizeof_int, n)
        dst = gr.vector_sink_i()
        
        self.fg.connect(src, op1)
        for i in range(n):
            self.fg.connect((op1, i), (op2, i))
        self.fg.connect(op2, op3, dst)
        
        self.fg.run()
        self.assertEqual(expected_results, dst.data())
        
    def test_004(self):
        """
        Test vector_to_streams.
        """
        n = 8
        src_len = n * 8
        src_data = tuple(range(src_len))
        expected_results = src_data

        src = gr.vector_source_i(src_data)
        op1 = gr.stream_to_vector(gr.sizeof_int, n)
        op2 = gr.vector_to_streams(gr.sizeof_int, n)
        op3 = gr.streams_to_stream(gr.sizeof_int, n)
        dst = gr.vector_sink_i()
        
        self.fg.connect(src, op1, op2)
        for i in range(n):
            self.fg.connect((op2, i), (op3, i))
        self.fg.connect(op3, dst)
        
        self.fg.run()
        self.assertEqual(expected_results, dst.data())

if __name__ == '__main__':
    gr_unittest.main ()
        
