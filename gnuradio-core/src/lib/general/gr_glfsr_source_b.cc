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


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <gr_glfsr_source_b.h>
#include <gri_glfsr.h>
#include <gr_io_signature.h>
#include <stdexcept>

gr_glfsr_source_b_sptr 
gr_make_glfsr_source_b(int degree, bool repeat, int mask, int seed)
{
  return gr_glfsr_source_b_sptr(new gr_glfsr_source_b(degree, repeat, mask, seed));
}

gr_glfsr_source_b::gr_glfsr_source_b(int degree, bool repeat, int mask, int seed)
  : gr_sync_block ("glfsr_source_b",
		   gr_make_io_signature (0, 0, 0),
		   gr_make_io_signature (1, 1, sizeof(unsigned char))),
    d_repeat(repeat),
    d_index(0)
{
  if (degree < 1 || degree > 32)
    throw std::runtime_error("gr_glfsr_source_b: degree must be between 1 and 32 inclusive");
  d_length = (unsigned int)((1ULL << degree)-1);

  if (mask == 0)
    mask = gri_glfsr::glfsr_mask(degree);
  d_glfsr = new gri_glfsr(mask, seed);
}

gr_glfsr_source_b::~gr_glfsr_source_b()
{
  delete d_glfsr;
}

int
gr_glfsr_source_b::work(int noutput_items,
			gr_vector_const_void_star &input_items,
			gr_vector_void_star &output_items)
{
  char *out = (char *) output_items[0];
  if ((d_index > d_length) && d_repeat == false)
    return -1; /* once through the sequence */

  int i;
  for (i = 0; i < noutput_items; i++) {
    out[i] = d_glfsr->next_bit();
    d_index++;
    if (d_index > d_length && d_repeat == false)
      break;
  }

  return i;
}

int
gr_glfsr_source_b::mask() const
{
  return d_glfsr->mask();
}
