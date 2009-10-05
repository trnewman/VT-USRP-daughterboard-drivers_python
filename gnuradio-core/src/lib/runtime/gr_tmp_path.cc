/* -*- c++ -*- */
/*
 * Copyright 2003 Free Software Foundation, Inc.
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

#include <gr_tmp_path.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const char *
gr_tmp_path ()
{
  static char *pp = 0;

  if (pp)
    return pp;

  char *s = getenv ("TMP");
  if (s){
    pp = strdup (s);
    return pp;
  }

#ifdef P_tmpdir
  if (P_tmpdir){
    pp = strdup (P_tmpdir);
    return pp;
  }
#endif

  pp = strdup ("/tmp");
  return pp;
}

