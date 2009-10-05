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

#include <gr_add_vii.h>
#include <gr_io_signature.h>

gr_add_vii_sptr
gr_make_add_vii (size_t nitems_per_block)
{
  return gr_add_vii_sptr (new gr_add_vii (nitems_per_block));
}

gr_add_vii::gr_add_vii (size_t nitems_per_block)
  : gr_sync_block ("add_vii",
		   gr_make_io_signature (1, -1, sizeof (int)*nitems_per_block),
		   gr_make_io_signature (1,  1, sizeof (int)*nitems_per_block))
{
}

int
gr_add_vii::work (int noutput_items,
		   gr_vector_const_void_star &input_items,
		   gr_vector_void_star &output_items)
{
  int *optr = (int *) output_items[0];

  int ninputs = input_items.size ();
  int nitems_per_block = output_signature()->sizeof_stream_item(0)/sizeof(int);

  for (int i = 0; i < noutput_items; i++){
    for (int j = 0; j < nitems_per_block; j++){
      int acc = ((int *) input_items[0])[i*nitems_per_block+j];
      for (int k = 1; k < ninputs; k++)
        acc += ((int *) input_items[k])[i*nitems_per_block+j];

      *optr++ = (int) acc;
    }
  }
  return noutput_items;
}
