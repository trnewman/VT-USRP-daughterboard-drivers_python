/* -*- c++ -*- */
/*
 * Copyright 2004 Free Software Foundation, Inc.
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gr_chunks_to_symbols_ic.h>
#include <gr_io_signature.h>
#include <assert.h>
#include <iostream>
#include <string.h>

gr_chunks_to_symbols_ic_sptr
gr_make_chunks_to_symbols_ic (const std::vector<gr_complex> &symbol_table, const int D)
{
  return gr_chunks_to_symbols_ic_sptr (new gr_chunks_to_symbols_ic (symbol_table,D));
}

gr_chunks_to_symbols_ic::gr_chunks_to_symbols_ic (const std::vector<gr_complex> &symbol_table, const int D)
  : gr_sync_interpolator ("chunks_to_symbols_ic",
			  gr_make_io_signature (1, -1, sizeof (int)),
			  gr_make_io_signature (1, -1, sizeof (gr_complex)),
			  D),
  d_D (D),
  d_symbol_table (symbol_table)
{
}

int
gr_chunks_to_symbols_ic::work (int noutput_items,
			gr_vector_const_void_star &input_items,
			gr_vector_void_star &output_items)
{
  assert (noutput_items % d_D == 0);
  assert (input_items.size() == output_items.size());
  int nstreams = input_items.size();

  for (int m=0;m<nstreams;m++) {
    const int *in = (int *) input_items[m];
    gr_complex *out = (gr_complex *) output_items[m];

    // per stream processing
    for (int i = 0; i < noutput_items / d_D; i++){
      assert (((unsigned int)in[i]*d_D+d_D) <= d_symbol_table.size());
      memcpy(out, &d_symbol_table[(unsigned int)in[i]*d_D], d_D*sizeof(gr_complex));
      out+=d_D;
    }
    // end of per stream processing

  }
  return noutput_items;
}
