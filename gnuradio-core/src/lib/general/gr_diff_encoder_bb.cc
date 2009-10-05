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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gr_diff_encoder_bb.h>
#include <gr_io_signature.h>

gr_diff_encoder_bb_sptr
gr_make_diff_encoder_bb (unsigned int modulus)
{
  return gr_diff_encoder_bb_sptr (new gr_diff_encoder_bb(modulus));
}

gr_diff_encoder_bb::gr_diff_encoder_bb (unsigned int modulus)
  : gr_sync_block ("diff_encoder_bb",
		   gr_make_io_signature (1, 1, sizeof (unsigned char)),
		   gr_make_io_signature (1, 1, sizeof (unsigned char))),
    d_last_out(0), d_modulus(modulus)
{
}

int
gr_diff_encoder_bb::work (int noutput_items,
			 gr_vector_const_void_star &input_items,
			 gr_vector_void_star &output_items)
{
  const unsigned char *in = (const unsigned char *) input_items[0];
  unsigned char *out = (unsigned char *) output_items[0];

  unsigned last_out = d_last_out;
  unsigned modulus = d_modulus;

  for (int i = 0; i < noutput_items; i++){
    out[i] = (in[i] + last_out) % modulus;
    last_out = out[i];
  }

  d_last_out = last_out;
  return noutput_items;
}
