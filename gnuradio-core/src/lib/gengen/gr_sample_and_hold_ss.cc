/* -*- c++ -*- */
/*
 * Copyright 2007 Free Software Foundation, Inc.
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

#include <gr_sample_and_hold_ss.h>
#include <gr_io_signature.h>

gr_sample_and_hold_ss_sptr
gr_make_sample_and_hold_ss ()
{
  return gr_sample_and_hold_ss_sptr (new gr_sample_and_hold_ss ());
}

gr_sample_and_hold_ss::gr_sample_and_hold_ss ()
  : gr_sync_block ("sample_and_hold_ss",
		   gr_make_io_signature2 (2, 2, sizeof (short), sizeof(char)),
		   gr_make_io_signature (1, 1, sizeof (short))),
    d_data(0)
{
}

int
gr_sample_and_hold_ss::work (int noutput_items,
		   gr_vector_const_void_star &input_items,
		   gr_vector_void_star &output_items)
{
  short *iptr = (short *) input_items[0];
  const char *ctrl = (const char *) input_items[1];
  short *optr = (short *) output_items[0];

  for (int i = 0; i < noutput_items; i++){
    if(ctrl[i]) {
      d_data = iptr[i];
    }
    optr[i] = d_data;
  }
  return noutput_items;
}
