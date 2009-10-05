/* -*- c++ -*- */
/*
 * Copyright 2004,2005 Free Software Foundation, Inc.
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gr_streams_to_vector.h>
#include <gr_io_signature.h>
#include <string.h>

gr_streams_to_vector_sptr
gr_make_streams_to_vector (size_t item_size, size_t nstreams)
{
  return gr_streams_to_vector_sptr (new gr_streams_to_vector (item_size, nstreams));
}

gr_streams_to_vector::gr_streams_to_vector (size_t item_size, size_t nstreams)
  : gr_sync_block ("streams_to_vector",
		   gr_make_io_signature (nstreams, nstreams, item_size),
		   gr_make_io_signature (1, 1, nstreams * item_size))
{
}

int
gr_streams_to_vector::work (int noutput_items,
			    gr_vector_const_void_star &input_items,
			    gr_vector_void_star &output_items)
{
  size_t item_size = input_signature()->sizeof_stream_item(0);
  int nstreams = input_items.size();

  const char **inv = (const char **) &input_items[0];
  char *out = (char *) output_items[0];

  for (int i = 0; i < noutput_items; i++){
    for (int j = 0; j < nstreams; j++){
      memcpy(out, inv[j], item_size);
      inv[j] += item_size;
      out += item_size;
    }
  }

  return noutput_items;
}
