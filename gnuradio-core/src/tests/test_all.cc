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

#include <cppunit/TextTestRunner.h>

#include <qa_runtime.h>
#include <qa_general.h>
#include <qa_filter.h>
// #include <qa_atsc.h>

// FIXME add atsc back in.

int 
main (int argc, char **argv)
{
  
  CppUnit::TextTestRunner	runner;

  runner.addTest (qa_runtime::suite ());
  runner.addTest (qa_general::suite ());
  runner.addTest (qa_filter::suite ());
  // runner.addTest (qa_atsc::suite ());
  
  bool was_successful = runner.run ("", false);

  return was_successful ? 0 : 1;
}
