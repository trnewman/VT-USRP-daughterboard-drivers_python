/* -*- c++ -*- */
/*
 * Copyright 2003,2005 Free Software Foundation, Inc.
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

#ifndef _GR_VMCIRCBUF_CREATEFILEMAPPING_H_
#define _GR_VMCIRCBUF_CREATEFILEMAPPING_H_

#include <gr_vmcircbuf.h>

#ifdef HAVE_CREATEFILEMAPPING
#include <windows.h>
#endif
/*!
 * \brief concrete class to implement circular buffers with mmap and shm_open
 * \ingroup base
 */
class gr_vmcircbuf_createfilemapping : public gr_vmcircbuf
{
 public:
   // CREATORS
   gr_vmcircbuf_createfilemapping (int size);
   virtual ~gr_vmcircbuf_createfilemapping ();
#ifdef HAVE_CREATEFILEMAPPING
 private:
   HANDLE d_handle;
   LPVOID d_first_copy;
   LPVOID d_second_copy;
#endif
};

/*!
 * \brief concrete factory for circular buffers built using mmap and shm_open
 */
class gr_vmcircbuf_createfilemapping_factory : public gr_vmcircbuf_factory
{
 private:
  static gr_vmcircbuf_factory *s_the_factory;

 public:
  static gr_vmcircbuf_factory *singleton ();

  virtual const char *name () const { return "gr_vmcircbuf_createfilemapping_factory"; }

  /*!
   * \brief return granularity of mapping, typically equal to page size
   */
  virtual int granularity ();

  /*!
   * \brief return a gr_vmcircbuf, or 0 if unable.
   *
   * Call this to create a doubly mapped circular buffer.
   */
  virtual gr_vmcircbuf *make (int size);
};

#endif /* _GR_VMCIRCBUF_CREATEFILEMAPPING_H_ */
