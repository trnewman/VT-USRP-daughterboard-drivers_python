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

#ifndef INCLUDED_GR_MOVING_AVERAGE_II_H
#define INCLUDED_GR_MOVING_AVERAGE_II_H

#include <gr_sync_block.h>

class gr_moving_average_ii;

typedef boost::shared_ptr<gr_moving_average_ii> gr_moving_average_ii_sptr;

gr_moving_average_ii_sptr gr_make_moving_average_ii (int length, int scale, int max_iter = 4096);

/*!
 * \brief output is the moving sum of the last N samples, scaled by the scale factor
 * \ingroup filter.  max_iter limits how long we go without flushing the accumulator
 * This is necessary to avoid numerical instability for float and complex.
 *
 */
class gr_moving_average_ii : public gr_sync_block
{
private:
  friend gr_moving_average_ii_sptr gr_make_moving_average_ii(int length, int scale, int max_iter);

  gr_moving_average_ii (int length, int scale, int max_iter = 4096);

  int d_length;
  int d_scale;
  int d_max_iter;

public:
  ~gr_moving_average_ii ();	

  int work (int noutput_items,
	    gr_vector_const_void_star &input_items,
	    gr_vector_void_star &output_items);
};

#endif /* INCLUDED_GR_MOVING_AVERAGE_II_H */
