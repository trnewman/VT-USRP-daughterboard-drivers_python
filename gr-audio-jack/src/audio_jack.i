/* -*- c++ -*- */
/*
 * Copyright 2005 Free Software Foundation, Inc.
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

%feature("autodoc","1");

%include "exception.i"
%import "gnuradio.i"				// the common stuff

%{
#include "gnuradio_swig_bug_workaround.h"	// mandatory bug fix
#include "audio_jack_sink.h"
#include "audio_jack_source.h"
#include <stdexcept>
%}

// ----------------------------------------------------------------

GR_SWIG_BLOCK_MAGIC(audio_jack,source)

audio_jack_source_sptr
audio_jack_make_source (int sampling_rate,
			const std::string dev = "",
			bool ok_to_block = true
			) throw (std::runtime_error);

class audio_jack_source : public gr_sync_block {

 protected:
  audio_jack_source (int sampling_rate,
		     const std::string device_name,
		     bool ok_to_block
		     ) throw (std::runtime_error);

 public:
  ~audio_jack_source ();

  bool start();
  bool stop();
};

// ----------------------------------------------------------------

GR_SWIG_BLOCK_MAGIC(audio_jack,sink)

audio_jack_sink_sptr
audio_jack_make_sink (int sampling_rate,
		      const std::string dev = "",
		      bool ok_to_block = true
		      ) throw (std::runtime_error);

class audio_jack_sink : public gr_sync_block {

 protected:
  audio_jack_sink (int sampling_rate,
		   const std::string device_name,
		   bool ok_to_block
		   ) throw (std::runtime_error);

 public:
  ~audio_jack_sink ();
};
