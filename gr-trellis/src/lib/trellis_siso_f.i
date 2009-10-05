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

GR_SWIG_BLOCK_MAGIC(trellis,siso_f);

trellis_siso_f_sptr trellis_make_siso_f (
    const fsm &FSM,
    int K,
    int S0,
    int SK,
    bool POSTI,
    bool POSTO,
    trellis_siso_type_t SISO_TYPE);


class trellis_siso_f : public gr_block
{
private:
  trellis_siso_f (
    const fsm &FSM,
    int K,
    int S0,
    int SK,
    bool POSTI,
    bool POSTO,
    trellis_siso_type_t SISO_TYPE);

public:
    fsm FSM () const { return d_FSM; }
    int K () const { return d_K; }
    int S0 () const { return d_S0; }
    int SK () const { return d_SK; }
    bool POSTI () const { return d_POSTI; }
    bool POSTO () const { return d_POSTO; }
    trellis_siso_type_t SISO_TYPE () const { return d_SISO_TYPE; }
};
