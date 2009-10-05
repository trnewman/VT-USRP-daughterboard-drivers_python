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
#ifndef _QA_FLOAT_DOTPROD_X86_H_
#define _QA_FLOAT_DOTPROD_X86_H_

#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestCase.h>

class qa_float_dotprod_x86 : public CppUnit::TestCase {
 public:
  void setUp            ();
  void tearDown         ();

  CPPUNIT_TEST_SUITE (qa_float_dotprod_x86);
  CPPUNIT_TEST (t1_3dnow);
  CPPUNIT_TEST (t2_3dnow);
  CPPUNIT_TEST (t3_3dnow);
  CPPUNIT_TEST (t1_sse);
  CPPUNIT_TEST (t2_sse);
  CPPUNIT_TEST (t3_sse);
  CPPUNIT_TEST_SUITE_END ();

 private:

  void t1_3dnow ();
  void t2_3dnow ();
  void t3_3dnow ();
  void t1_sse ();
  void t2_sse ();
  void t3_sse ();


  typedef float (*float_dotprod_t)(const float *input,
				   const float *taps,
				   unsigned n_4_float_blocks);

  void t1_base (float_dotprod_t);
  void t2_base (float_dotprod_t);
  void t3_base (float_dotprod_t);

  
  void zb ();

  float	*taps;		// 16-byte aligned
  float *input;		// 16-byte aligned

};


#endif /* _QA_FLOAT_DOTPROD_X86_H_ */
