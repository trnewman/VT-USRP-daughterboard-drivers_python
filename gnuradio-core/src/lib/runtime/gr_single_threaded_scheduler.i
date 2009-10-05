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

#include <gr_runtime.h>

class gr_single_threaded_scheduler;
typedef boost::shared_ptr<gr_single_threaded_scheduler>	gr_single_threaded_scheduler_sptr;
%template(gr_single_threaded_scheduler_sptr) boost::shared_ptr<gr_single_threaded_scheduler>;
%rename(single_threaded_scheduler) gr_make_single_threaded_scheduler;
%ignore gr_single_threaded_scheduler;

gr_single_threaded_scheduler_sptr 
gr_make_single_threaded_scheduler (const std::vector<gr_block_sptr> &modules);

class gr_single_threaded_scheduler {
 public:
  ~gr_single_threaded_scheduler ();

  // void run ();
  void stop ();

 private:
  gr_single_threaded_scheduler (const std::vector<gr_block_sptr> &modules);
};

%inline {
  void sts_pyrun (gr_single_threaded_scheduler_sptr s) {
    Py_BEGIN_ALLOW_THREADS;		// release global interpreter lock
    s->run ();
    Py_END_ALLOW_THREADS;		// acquire global interpreter lock
  }
}

