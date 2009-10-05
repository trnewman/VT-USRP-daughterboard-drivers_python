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
#include <config.h>
#endif
#include <gr_vector_source_s.h>
#include <algorithm>
#include <gr_io_signature.h>


gr_vector_source_s::gr_vector_source_s (const std::vector<short> &data, bool repeat)
  : gr_sync_block ("vector_source_s",
	       gr_make_io_signature (0, 0, 0),
	       gr_make_io_signature (1, 1, sizeof (short))),
    d_data (data),
    d_repeat (repeat),
    d_offset (0)
{
}

int
gr_vector_source_s::work (int noutput_items,
		    gr_vector_const_void_star &input_items,
		    gr_vector_void_star &output_items)
{
  short *optr = (short *) output_items[0];

  if (d_repeat){
    unsigned int size = d_data.size ();
    unsigned int offset = d_offset;
    
    if (size == 0)
      return -1;
    
    for (int i = 0; i < noutput_items; i++){
      optr[i] = d_data[offset++];
      if (offset >= size)
	offset = 0;
    }
    d_offset = offset;
    return noutput_items;
  }

  else {
    if (d_offset >= d_data.size ())
      return -1;			// Done!

    unsigned n = std::min ((unsigned) d_data.size () - d_offset,
			   (unsigned) noutput_items);
    for (unsigned i = 0; i < n; i++)
      optr[i] = d_data[d_offset + i];

    d_offset += n;
    return n;
  }
}

gr_vector_source_s_sptr
gr_make_vector_source_s (const std::vector<short> &data, bool repeat)
{
  return gr_vector_source_s_sptr (new gr_vector_source_s (data, repeat));
}

