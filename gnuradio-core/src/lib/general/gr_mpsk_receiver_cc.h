/* -*- c++ -*- */
/*
 * Copyright 2004,2007 Free Software Foundation, Inc.
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

#ifndef INCLUDED_GR_MPSK_RECEIVER_CC_H
#define	INCLUDED_GR_MPSK_RECEIVER_CC_H

#include <gr_block.h>
#include <gr_complex.h>
#include <fstream>

class gri_mmse_fir_interpolator_cc;

class gr_mpsk_receiver_cc;
typedef boost::shared_ptr<gr_mpsk_receiver_cc> gr_mpsk_receiver_cc_sptr;

// public constructor
gr_mpsk_receiver_cc_sptr 
gr_make_mpsk_receiver_cc (unsigned int M, float theta, 
			  float alpha, float beta,
			  float fmin, float fmax,
			  float mu, float gain_mu, 
			  float omega, float gain_omega, float omega_rel);

/*!
 * \brief This block takes care of receiving M-PSK modulated signals through phase, frequency, and symbol
 * synchronization. 
 * \ingroup synch
 *
 * This block takes care of receiving M-PSK modulated signals through phase, frequency, and symbol
 * synchronization. It performs carrier frequency and phase locking as well as symbol timing recovery. 
 * It works with (D)BPSK, (D)QPSK, and (D)8PSK as tested currently. It should also work for OQPSK and 
 * PI/4 DQPSK.
 *
 * The phase and frequency synchronization are based on a Costas loop that finds the error of the incoming
 * signal point compared to its nearest constellation point. The frequency and phase of the NCO are 
 * updated according to this error. There are optimized phase error detectors for BPSK and QPSK, but 8PSK
 * is done using a brute-force computation of the constellation points to find the minimum.
 *
 * The symbol synchronization is done using a modified Mueller and Muller circuit from the paper:
 * 
 *    G. R. Danesfahani, T.G. Jeans, "Optimisation of modified Mueller and Muller 
 *    algorithm,"  Electronics Letters, Vol. 31, no. 13,  22 June 1995, pp. 1032 - 1033.
 *
 * This circuit interpolates the downconverted sample (using the NCO developed by the Costas loop)
 * every mu samples, then it finds the sampling error based on this and the past symbols and the decision
 * made on the samples. Like the phase error detector, there are optimized decision algorithms for BPSK
 * and QPKS, but 8PSK uses another brute force computation against all possible symbols. The modifications
 * to the M&M used here reduce self-noise.
 *
 */

class gr_mpsk_receiver_cc : public gr_block
{
 public:
  ~gr_mpsk_receiver_cc ();
  void forecast(int noutput_items, gr_vector_int &ninput_items_required);
  int general_work (int noutput_items,
		    gr_vector_int &ninput_items,
		    gr_vector_const_void_star &input_items,
		    gr_vector_void_star &output_items);


  // Member functions related to the symbol tracking portion of the receiver
  //! (M&M) Returns current value of mu
  float mu() const { return d_mu;}

  //! (M&M) Returns current value of omega
  float omega() const { return d_omega;}

  //! (M&M) Returns mu gain factor
  float gain_mu() const { return d_gain_mu;}

  //! (M&M) Returns omega gain factor
  float gain_omega() const { return d_gain_omega;}

  //! (M&M) Sets value of mu
  void set_mu (float mu) { d_mu = mu; }
  
  //! (M&M) Sets value of omega and its min and max values 
  void set_omega (float omega) { 
    d_omega = omega;
    d_min_omega = omega*(1.0 - d_omega_rel);
    d_max_omega = omega*(1.0 + d_omega_rel);
  }

  //! (M&M) Sets value for mu gain factor
  void set_gain_mu (float gain_mu) { d_gain_mu = gain_mu; }

  //! (M&M) Sets value for omega gain factor
  void set_gain_omega (float gain_omega) { d_gain_omega = gain_omega; }



  // Member function related to the phase/frequency tracking portion of the receiver
  //! (CL) Returns the value for alpha (the phase gain term)
  float alpha() const { return d_alpha; }
  
  //! (CL) Returns the value of beta (the frequency gain term)
  float beta() const { return d_beta; }

  //! (CL) Returns the current value of the frequency of the NCO in the Costas loop
  float freq() const { return d_freq; }

  //! (CL) Returns the current value of the phase of the NCO in the Costal loop
  float phase() const { return d_phase; }

  //! (CL) Sets the value for alpha (the phase gain term)
  void set_alpha(float alpha) { d_alpha = alpha; }
  
  //! (CL) Setss the value of beta (the frequency gain term)
  void set_beta(float beta) { d_beta = beta; }

  //! (CL) Sets the current value of the frequency of the NCO in the Costas loop
  void set_freq(float freq) { d_freq = freq; }

  //! (CL) Setss the current value of the phase of the NCO in the Costal loop
  void set_phase(float phase) { d_phase = phase; }


protected:

 /*!
   * \brief Constructor to synchronize incoming M-PSK symbols
   *
   * \param M	        modulation order of the M-PSK modulation
   * \param theta	any constant phase rotation from the real axis of the constellation
   * \param alpha	gain parameter to adjust the phase in the Costas loop (~0.01)
   * \param beta        gain parameter to adjust the frequency in the Costas loop (~alpha^2/4)	
   * \param fmin        minimum normalized frequency value the loop can achieve
   * \param fmax        maximum normalized frequency value the loop can achieve
   * \param mu          initial parameter for the interpolator [0,1]
   * \param gain_mu     gain parameter of the M&M error signal to adjust mu (~0.05)
   * \param omega       initial value for the number of symbols between samples (~number of samples/symbol)
   * \param gain_omega  gain parameter to adjust omega based on the error (~omega^2/4)
   * \param omega_rel   sets the maximum (omega*(1+omega_rel)) and minimum (omega*(1+omega_rel)) omega (~0.005)
   *
   * The constructor also chooses which phase detector and decision maker to use in the work loop based on the
   * value of M.
   */
  gr_mpsk_receiver_cc (unsigned int M, float theta, 
		       float alpha, float beta,
		       float fmin, float fmax,
		       float mu, float gain_mu, 
		       float omega, float gain_omega, float omega_rel);

  void make_constellation();
  void mm_sampler(const gr_complex symbol);
  void mm_error_tracking(gr_complex sample);
  void phase_error_tracking(gr_complex sample);


/*!
   * \brief Phase error detector for MPSK modulations.
   *
   * \param sample   the I&Q sample from which to determine the phase error
   *
   * This function determines the phase error for any MPSK signal by creating a set of PSK constellation points
   * and doing a brute-force search to see which point minimizes the Euclidean distance. This point is then used
   * to derotate the sample to the real-axis and a atan (using the fast approximation function) to determine the
   * phase difference between the incoming sample and the real constellation point
   *
   * This should be cleaned up and made more efficient.
   *
   * \returns the approximated phase error.
 */
  float phase_error_detector_generic(gr_complex sample) const; // generic for M but more costly

 /*!
   * \brief Phase error detector for BPSK modulation.
   *
   * \param sample   the I&Q sample from which to determine the phase error
   *
   * This function determines the phase error using a simple BPSK phase error detector by multiplying the real
   * and imaginary (the error signal) components together. As the imaginary part goes to 0, so does this error.
   *
   * \returns the approximated phase error.
 */
  float phase_error_detector_bpsk(gr_complex sample) const;    // optimized for BPSK

 /*!
   * \brief Phase error detector for QPSK modulation.
   *
   * \param sample   the I&Q sample from which to determine the phase error
   *
   * This function determines the phase error using the limiter approach in a standard 4th order Costas loop
   *
   * \returns the approximated phase error.
 */
  float phase_error_detector_qpsk(gr_complex sample) const;



 /*!
   * \brief Decision maker for a generic MPSK constellation.
   *
   * \param sample   the baseband I&Q sample from which to make the decision
   *
   * This decision maker is a generic implementation that does a brute-force search 
   * for the constellation point that minimizes the error between it and the incoming signal.
   *
   * \returns the index to d_constellation that minimizes the error/
 */
  unsigned int decision_generic(gr_complex sample) const;


 /*!
   * \brief Decision maker for BPSK constellation.
   *
   * \param sample   the baseband I&Q sample from which to make the decision
   *
   * This decision maker is a simple slicer function that makes a decision on the symbol based on its
   * placement on the real axis of greater than 0 or less than 0; the quadrature component is always 0.
   *
   * \returns the index to d_constellation that minimizes the error/
 */
  unsigned int decision_bpsk(gr_complex sample) const;
  

 /*!
   * \brief Decision maker for QPSK constellation.
   *
   * \param sample   the baseband I&Q sample from which to make the decision
   *
   * This decision maker is a simple slicer function that makes a decision on the symbol based on its
   * placement versus both axes and returns which quadrant the symbol is in.
   *
   * \returns the index to d_constellation that minimizes the error/
 */
  unsigned int decision_qpsk(gr_complex sample) const;

  private:
  unsigned int d_M;
  float        d_theta;

  // Members related to carrier and phase tracking
  float d_alpha;
  float d_beta;
  float d_freq, d_max_freq, d_min_freq;
  float d_phase;

/*!
   * \brief Decision maker function pointer 
   *
   * \param sample   the baseband I&Q sample from which to make the decision
   *
   * This is a function pointer that is set in the constructor to point to the proper decision function
   * for the specified constellation order.
   *
   * \return index into d_constellation point that is the closest to the recieved sample
 */
  unsigned int (gr_mpsk_receiver_cc::*d_decision)(gr_complex sample) const; // pointer to decision function


  std::vector<gr_complex> d_constellation;
  unsigned int d_current_const_point;

  // Members related to symbol timing
  float d_mu, d_gain_mu;
  float d_omega, d_gain_omega, d_omega_rel, d_max_omega, d_min_omega;
  gr_complex d_p_2T, d_p_1T, d_p_0T;
  gr_complex d_c_2T, d_c_1T, d_c_0T;

 /*!
   * \brief Phase error detector function pointer 
   *
   * \param sample   the I&Q sample from which to determine the phase error
   *
   * This is a function pointer that is set in the constructor to point to the proper phase error detector
   * function for the specified constellation order.
 */
  float (gr_mpsk_receiver_cc::*d_phase_error_detector)(gr_complex sample) const;


  //! get interpolated value
  gri_mmse_fir_interpolator_cc 	*d_interp;
  
  //! delay line length.
  static const unsigned int DLLEN = 8;
  
  //! delay line plus some length for overflow protection
  gr_complex d_dl[2*DLLEN] __attribute__ ((aligned(8)));
  
  //! index to delay line
  unsigned int d_dl_idx;

  friend gr_mpsk_receiver_cc_sptr
  gr_make_mpsk_receiver_cc (unsigned int M, float theta,
			    float alpha, float beta,
			    float fmin, float fmax,
			    float mu, float gain_mu, 
			    float omega, float gain_omega, float omega_rel);
};

#endif
