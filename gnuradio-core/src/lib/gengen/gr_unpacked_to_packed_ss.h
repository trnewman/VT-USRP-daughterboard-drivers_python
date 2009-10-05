/* -*- c++ -*- */
/*
 * Copyright 2006 Free Software Foundation, Inc.
 * 
 * This file is part of GNU Radio
 * 
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

// WARNING: this file is machine generated.  Edits will be over written

#ifndef INCLUDED_GR_UNPACKED_TO_PACKED_SS_H
#define INCLUDED_GR_UNPACKED_TO_PACKED_SS_H

#include <gr_block.h>
#include <gr_endianness.h>

class gr_unpacked_to_packed_ss;
typedef boost::shared_ptr<gr_unpacked_to_packed_ss> gr_unpacked_to_packed_ss_sptr;

gr_unpacked_to_packed_ss_sptr 
gr_make_unpacked_to_packed_ss (unsigned int bits_per_chunk, gr_endianness_t endianness);

/*!
 * \brief Convert a stream of unpacked bytes or shorts into a stream of packed bytes or shorts.
 * \ingroup converter
 *
 * input: stream of short; output: stream of short
 *
 * This is the inverse of gr_packed_to_unpacked_XX.
 *
 * The low \p bits_per_chunk bits are extracted from each input byte or short.
 * These bits are then packed densely into the output bytes or shorts, such that
 * all 8 or 16 bits of the output bytes or shorts are filled with valid input bits.
 * The right thing is done if bits_per_chunk is not a power of two.
 *
 * The combination of gr_packed_to_unpacked_XX followed by
 * gr_chunks_to_symbols_Xf or gr_chunks_to_symbols_Xc handles the
 * general case of mapping from a stream of bytes or shorts into arbitrary float
 * or complex symbols.
 *
 * \sa gr_packed_to_unpacked_bb, gr_unpacked_to_packed_bb,
 * \sa gr_packed_to_unpacked_ss, gr_unpacked_to_packed_ss,
 * \sa gr_chunks_to_symbols_bf, gr_chunks_to_symbols_bc.
 * \sa gr_chunks_to_symbols_sf, gr_chunks_to_symbols_sc.
 */
class gr_unpacked_to_packed_ss : public gr_block
{
  friend gr_unpacked_to_packed_ss_sptr
  gr_make_unpacked_to_packed_ss (unsigned int bits_per_chunk, gr_endianness_t endianness);

  gr_unpacked_to_packed_ss (unsigned int bits_per_chunk, gr_endianness_t endianness);

  unsigned int    d_bits_per_chunk;
  gr_endianness_t d_endianness;
  unsigned int    d_index;

 public:
  void forecast(int noutput_items, gr_vector_int &ninput_items_required);
  int general_work (int noutput_items,
		    gr_vector_int &ninput_items,
		    gr_vector_const_void_star &input_items,
		    gr_vector_void_star &output_items);

  bool check_topology(int ninputs, int noutputs) { return ninputs == noutputs; }
};

#endif
