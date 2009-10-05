/* -*- c++ -*- */
/*
 * Copyright 2007 Free Software Foundation, Inc.
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gr_top_block.h>
#include <gr_top_block_impl.h>
#include <gr_flat_flowgraph.h>
#include <gr_scheduler_thread.h>
#include <gr_local_sighandler.h>

#include <stdexcept>
#include <iostream>
#include <string.h>
#include <unistd.h>

#define GR_TOP_BLOCK_IMPL_DEBUG 0

static gr_top_block_impl *s_impl = 0;

/*!
 * Make a vector of gr_block from a vector of gr_basic_block
 *
 * Pass-by-value to avoid problem with possible asynchronous modification
 */
static gr_block_vector_t
make_gr_block_vector(gr_basic_block_vector_t blocks)
{
  gr_block_vector_t result;
  for (gr_basic_block_viter_t p = blocks.begin(); p != blocks.end(); p++) {
    result.push_back(make_gr_block_sptr(*p));
  }

  return result;
}

// FIXME: This prevents using more than one gr_top_block instance

static void 
runtime_sigint_handler(int signum)
{
  if (GR_TOP_BLOCK_IMPL_DEBUG){
    char *msg = "SIGINT received, calling stop()\n";
    ::write(1, msg, strlen(msg));	// write is OK to call from signal handler
  }

  if (s_impl)
    s_impl->stop();
}

// ----------------------------------------------------------------

gr_top_block_impl::gr_top_block_impl(gr_top_block *owner) 
  : d_running(false),
    d_ffg(),
    d_owner(owner),
    d_lock_count(0)
{
  if (s_impl)
    throw std::logic_error("gr_top_block_impl: multiple simultaneous gr_top_blocks not allowed");

  s_impl = this;
}

gr_top_block_impl::~gr_top_block_impl()
{
  s_impl = 0; // don't call delete we don't own these
  d_owner = 0;
}

void
gr_top_block_impl::start()
{
  if (GR_TOP_BLOCK_IMPL_DEBUG)
    std::cout << "start: entered" << std::endl;

  if (d_running)
    throw std::runtime_error("top_block::start: top block already running or wait() not called after previous stop()");

  if (d_lock_count > 0)
    throw std::runtime_error("top_block::start: can't call start with flow graph locked");

  // Create new flat flow graph by flattening hierarchy
  d_ffg = d_owner->flatten();

  // Validate new simple flow graph and wire it up
  d_ffg->validate();
  d_ffg->setup_connections();

  // Execute scheduler threads
  start_threads();
  d_running = true;
}

void
gr_top_block_impl::start_threads()
{
  if (GR_TOP_BLOCK_IMPL_DEBUG)
    std::cout << "start_threads: entered" << std::endl;

  d_graphs = d_ffg->partition();
  for (std::vector<gr_basic_block_vector_t>::iterator p = d_graphs.begin();
       p != d_graphs.end(); p++) {
    gr_scheduler_thread *thread = new gr_scheduler_thread(make_gr_block_vector(*p));
    d_threads.push_back(thread);
    if (GR_TOP_BLOCK_IMPL_DEBUG)
      std::cout << "start_threads: starting " << thread << std::endl;
    thread->start();
  }

  d_running = true;
}

/*
 * N.B. as currently implemented, it is possible that this may be
 * invoked by the SIGINT handler which is fragile as hell...
 */
void
gr_top_block_impl::stop()
{
  if (GR_TOP_BLOCK_IMPL_DEBUG){
    char *msg = "stop: entered\n";
    ::write(1, msg, strlen(msg));
  }

  for (gr_scheduler_thread_viter_t p = d_threads.begin(); p != d_threads.end(); p++) {
    if (*p)
      (*p)->stop();
  }
  d_running = false;
}

void
gr_top_block_impl::wait()
{
  if (GR_TOP_BLOCK_IMPL_DEBUG)
    std::cout << "wait: entered" << std::endl;

  void *dummy_status; // don't ever dereference this
  gr_local_sighandler sigint(SIGINT, runtime_sigint_handler);

  for (gr_scheduler_thread_viter_t p = d_threads.begin(); p != d_threads.end(); p++) {
    if (GR_TOP_BLOCK_IMPL_DEBUG)
      std::cout << "wait: joining thread " << (*p) << std::endl;
    (*p)->join(&dummy_status); // pthreads will self-delete, so pointer is now dead
    (*p) = 0; // FIXME: switch to stl::list and actually remove from container
    if (GR_TOP_BLOCK_IMPL_DEBUG)
      std::cout << "wait: join returned" << std::endl;
  }

  d_threads.clear();
}

// N.B. lock() and unlock() cannot be called from a flow graph thread or
// deadlock will occur when reconfiguration happens
void
gr_top_block_impl::lock()
{
  omni_mutex_lock lock(d_reconf);
  d_lock_count++;
  if (GR_TOP_BLOCK_IMPL_DEBUG)
    std::cout << "runtime: locked, count = " << d_lock_count <<  std::endl;
}

void
gr_top_block_impl::unlock()
{
  omni_mutex_lock lock(d_reconf);
  if (d_lock_count <= 0){
    d_lock_count = 0;		// fix it, then complain
    throw std::runtime_error("unpaired unlock() call");
  }

  d_lock_count--;
  if (GR_TOP_BLOCK_IMPL_DEBUG)
    std::cout << "unlock: unlocked, count = " << d_lock_count << std::endl;

  if (d_lock_count == 0) {
    if (GR_TOP_BLOCK_IMPL_DEBUG)
      std::cout << "unlock: restarting flowgraph" << std::endl;
    restart();
  }
}

void
gr_top_block_impl::restart()
{
  if (GR_TOP_BLOCK_IMPL_DEBUG)
    std::cout << "restart: entered" << std::endl;

  if (!d_running)
    return;		// nothing to do

  // Stop scheduler threads and wait for completion
  stop();
  wait();
  if (GR_TOP_BLOCK_IMPL_DEBUG)
    std::cout << "restart: threads stopped" << std::endl;

  // Create new simple flow graph
  gr_flat_flowgraph_sptr new_ffg = d_owner->flatten();        
  new_ffg->validate();		       // check consistency, sanity, etc

  if (GR_TOP_BLOCK_IMPL_DEBUG) {
      std::cout << std::endl << "*** Existing flat flowgraph @" << d_ffg << ":" << std::endl;
      d_ffg->dump();
  }
  new_ffg->merge_connections(d_ffg);   // reuse buffers, etc

  if (GR_TOP_BLOCK_IMPL_DEBUG) {
    std::cout << std::endl << "*** New flat flowgraph after merge @" << new_ffg << ":" << std::endl;
    new_ffg->dump();
  }
  
  d_ffg = new_ffg;

  start_threads();
  d_running = true;
}

void
gr_top_block_impl::dump()
{
  if (d_ffg)
    d_ffg->dump();
}
