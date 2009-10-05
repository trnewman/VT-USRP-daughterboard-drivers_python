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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gr_keep_one_in_n.h>
#include <gr_io_signature.h>
#include <string.h>

gr_keep_one_in_n_sptr
gr_make_keep_one_in_n (size_t item_size, int n)
{
  return gr_keep_one_in_n_sptr (new gr_keep_one_in_n (item_size, n));
}

gr_keep_one_in_n::gr_keep_one_in_n (size_t item_size, int n)
  : gr_block ("keep_one_in_n",
	      gr_make_io_signature (1, 1, item_size),
	      gr_make_io_signature (1, 1, item_size)),
    d_n (n), d_count(n)
{
}

void
gr_keep_one_in_n::set_n(int n)
{
  if (n < 1)
    n = 1;

  d_n = n;
  d_count = n;
}

int
gr_keep_one_in_n::general_work (int noutput_items,
				gr_vector_int &ninput_items,
				gr_vector_const_void_star &input_items,
				gr_vector_void_star &output_items)
{
  const char *in = (const char *) input_items[0];
  char *out = (char *) output_items[0];
  
  size_t item_size = input_signature ()->sizeof_stream_item (0);
  int	 ni = 0;
  int	 no = 0;

  while (ni < ninput_items[0] && no < noutput_items){
    d_count--;
    if (d_count <= 0){
      memcpy (out, in, item_size);		// copy 1 item
      out += item_size;
      no++;
      d_count = d_n;
    }
    in += item_size;
    ni++;
  }

  consume_each (ni);
  return no;
}
