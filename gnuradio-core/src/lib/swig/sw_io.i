/* -*- c++ -*- */
/*
 * Copyright 2006 Free Software Foundation, Inc.
 * 
 * This file is part of GNU Radio
 * 
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Io Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Io Public License for more details.
 * 
 * You should have received a copy of the GNU Io Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef SWIGIMPORTED
%module(directors="1") gnuradio_swig_py_io
#endif


%feature("autodoc", "1");		// generate python docstrings

%import "gnuradio.i"				// the common stuff

%{
#include "gnuradio_swig_bug_workaround.h"	// mandatory bug fix
%}

%include "io.i"
