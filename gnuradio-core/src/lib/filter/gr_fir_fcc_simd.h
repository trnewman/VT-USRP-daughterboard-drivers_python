/* -*- c++ -*- */
/*
 * Copyright 2002 Free Software Foundation, Inc.
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
#ifndef INCLUDED_GR_FIR_FCC_SIMD_H
#define INCLUDED_GR_FIR_FCC_SIMD_H

#include <gr_fir_fcc_generic.h>


/*!
 * \brief common base class for SIMD versions of gr_fir_fcc
 * \ingroup filter
 *
 * This base class handles alignment issues common to SSE and 3DNOW
 * subclasses.
 */

class gr_fir_fcc_simd : public gr_fir_fcc_generic
{
protected:
  typedef void (*fcomplex_dotprod_t)(const float *input,
				     const float *taps,
				     unsigned n_2_complex_blocks,
				     float *result);

  /*!
   * \p aligned_taps holds 4 copies of the coefficients preshifted
   * by 0, 1, 2, or 3 float pairs to meet all possible input data alignments.
   * This allows us to always fetch data and taps that are 128-bit aligned.
   */
  float 		*d_aligned_taps[4];

  fcomplex_dotprod_t	d_fcomplex_dotprod; 	// fast dot product primitive

public:

  // CREATORS
  gr_fir_fcc_simd ();
  gr_fir_fcc_simd (const std::vector<gr_complex> &taps);
  ~gr_fir_fcc_simd ();

  // MANIPULATORS
  virtual void set_taps (const std::vector<gr_complex> &taps);
  virtual gr_complex filter (const float input[]);
};

#endif
