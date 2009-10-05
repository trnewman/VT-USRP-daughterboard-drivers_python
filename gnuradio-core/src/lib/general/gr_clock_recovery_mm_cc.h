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

#ifndef INCLUDED_GR_CLOCK_RECOVERY_MM_CC_H
#define	INCLUDED_GR_CLOCK_RECOVERY_MM_CC_H

#include <gr_block.h>
#include <gr_complex.h>

class gri_mmse_fir_interpolator_cc;

class gr_clock_recovery_mm_cc;
typedef boost::shared_ptr<gr_clock_recovery_mm_cc> gr_clock_recovery_mm_cc_sptr;

// public constructor
gr_clock_recovery_mm_cc_sptr 
gr_make_clock_recovery_mm_cc (float omega, float gain_omega, float mu, float gain_mu,
			      float omega_relative_limit=0.001);

/*!
 * \brief Mueller and Müller (M&M) based clock recovery block with complex input, complex output.
 * \ingroup clock
 *
 * This implements the Mueller and Müller (M&M) discrete-time error-tracking synchronizer.
 * The complex version here is based on:
 * Modified Mueller and Muller clock recovery circuit
 * Based:
 *    G. R. Danesfahani, T.G. Jeans, "Optimisation of modified Mueller and Muller 
 *    algorithm,"  Electronics Letters, Vol. 31, no. 13,  22 June 1995, pp. 1032 - 1033.
 */
class gr_clock_recovery_mm_cc : public gr_block
{
 public:
  ~gr_clock_recovery_mm_cc ();
  void forecast(int noutput_items, gr_vector_int &ninput_items_required);
  int general_work (int noutput_items,
		    gr_vector_int &ninput_items,
		    gr_vector_const_void_star &input_items,
		    gr_vector_void_star &output_items);
  float mu() const { return d_mu;}
  float omega() const { return d_omega;}
  float gain_mu() const { return d_gain_mu;}
  float gain_omega() const { return d_gain_omega;}
  void set_verbose (bool verbose) { d_verbose = verbose; }

  void set_gain_mu (float gain_mu) { d_gain_mu = gain_mu; }
  void set_gain_omega (float gain_omega) { d_gain_omega = gain_omega; }
  void set_mu (float mu) { d_mu = mu; }
  void set_omega (float omega) { 
    d_omega = omega;
    d_min_omega = omega*(1.0 - d_omega_relative_limit);
    d_max_omega = omega*(1.0 + d_omega_relative_limit);
  }

protected:
  gr_clock_recovery_mm_cc (float omega, float gain_omega, float mu, float gain_mu,
			   float omega_relative_limi);

 private:
  float 			d_mu;
  float 			d_omega;
  float                         d_gain_omega;
  float				d_min_omega;	        // minimum allowed omega
  float				d_max_omega;	        // maximum allowed omeg
  float				d_omega_relative_limit;	// used to compute min and max omega
  float                         d_gain_mu;
  gr_complex                    d_last_sample;
  gri_mmse_fir_interpolator_cc 	*d_interp;
  bool			        d_verbose;

  gr_complex                    d_p_2T;
  gr_complex                    d_p_1T;
  gr_complex                    d_p_0T;

  gr_complex                    d_c_2T;
  gr_complex                    d_c_1T;
  gr_complex                    d_c_0T;

  gr_complex slicer_0deg (gr_complex sample);
  gr_complex slicer_45deg (gr_complex sample);

  friend gr_clock_recovery_mm_cc_sptr
  gr_make_clock_recovery_mm_cc (float omega, float gain_omega, float mu, float gain_mu, 
				float omega_relative_limit);
};

#endif
