/* -*- c++ -*- */
/*
 * Copyright 2008 Free Software Foundation, Inc.
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

#include <gr_moving_average_cc.h>
#include <gr_io_signature.h>

gr_moving_average_cc_sptr 
gr_make_moving_average_cc (int length, gr_complex scale, int max_iter)
{
  return gr_moving_average_cc_sptr (new gr_moving_average_cc (length, scale, max_iter));
}

gr_moving_average_cc::gr_moving_average_cc (int length, gr_complex scale, int max_iter)
  : gr_sync_block ("moving_average_cc",
		   gr_make_io_signature (1, 1, sizeof (gr_complex)),
		   gr_make_io_signature (1, 1, sizeof (gr_complex))),
    d_length(length),
    d_scale(scale),
    d_max_iter(max_iter)
{
  set_history(length);
}

gr_moving_average_cc::~gr_moving_average_cc ()
{
}

int 
gr_moving_average_cc::work (int noutput_items,
	      gr_vector_const_void_star &input_items,
	      gr_vector_void_star &output_items)
{
  const gr_complex *in = (const gr_complex *) input_items[0];
  gr_complex *out = (gr_complex *) output_items[0];

  gr_complex sum = 0;
  int num_iter = (noutput_items>d_max_iter) ? d_max_iter : noutput_items;
  for (int i = 0; i < d_length-1 ; i++) {
    sum += in[i];
  }

  for (int i = 0; i < num_iter; i++) {
    sum += in[i+d_length-1];
    out[i] = sum * d_scale;
    sum -= in[i];
  }

  return num_iter;
}
