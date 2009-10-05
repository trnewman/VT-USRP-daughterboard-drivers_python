/* -*- c++ -*- */
/*
 * Copyright 2006 Free Software Foundation, Inc.
 * 
 * This file is part of GNU Radio.
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

// tell mld_threads to NOT use omni_threads,
// but rather Darwin's pthreads
#define _USE_OMNI_THREADS_
#define DO_DEBUG 0

#include <usb.h>
#include "fusb.h"
#include "fusb_darwin.h"
#include "darwin_libusb.h"

static const int USB_TIMEOUT = 100;	// in milliseconds
static const UInt8 NUM_QUEUE_ITEMS = 20;

fusb_devhandle_darwin::fusb_devhandle_darwin (usb_dev_handle* udh)
  : fusb_devhandle (udh)
{
  // that's it
}

fusb_devhandle_darwin::~fusb_devhandle_darwin ()
{
  // nop
}

fusb_ephandle*
fusb_devhandle_darwin::make_ephandle (int endpoint, bool input_p,
				      int block_size, int nblocks)
{
  return new fusb_ephandle_darwin (this, endpoint, input_p,
				   block_size, nblocks);
}

// ----------------------------------------------------------------

fusb_ephandle_darwin::fusb_ephandle_darwin (fusb_devhandle_darwin* dh,
					    int endpoint, bool input_p,
					    int block_size, int nblocks)
  : fusb_ephandle (endpoint, input_p, block_size, nblocks),
    d_devhandle (dh), d_pipeRef (0), d_transferType (0),
    d_interfaceRef (0),  d_interface (0), d_queue (0),
    d_buffer (0), d_bufLenBytes (0)
{
  d_bufLenBytes = fusb_sysconfig::max_block_size();

// create circular buffer
  d_buffer = new circular_buffer<char> (NUM_QUEUE_ITEMS * d_bufLenBytes,
					!d_input_p, d_input_p);

// create the queue
  d_queue = new circular_linked_list <s_buffer_ptr> (NUM_QUEUE_ITEMS);
  d_queue->iterate_start ();
  s_node_ptr l_node = d_queue->iterate_next ();
  while (l_node) {
    l_node->both (new s_both<s_buffer_ptr> (l_node, this));
    s_buffer_ptr l_buf = new s_buffer (d_bufLenBytes);
    l_node->object (l_buf);
    l_node = d_queue->iterate_next ();
    l_buf = NULL;
  }

  d_readRunning = new mld_mutex ();
  d_runThreadRunning = new mld_mutex ();
  d_runBlock = new mld_condition ();
  d_readBlock = new mld_condition ();
}

fusb_ephandle_darwin::~fusb_ephandle_darwin ()
{
  stop ();

  d_queue->iterate_start ();
  s_node_ptr l_node = d_queue->iterate_next ();
  while (l_node) {
    s_both_ptr l_both = l_node->both ();
    delete l_both;
    l_both = NULL;
    l_node->both (NULL);
    s_buffer_ptr l_buf = l_node->object ();
    delete l_buf;
    l_buf = NULL;
    l_node->object (NULL);
    l_node = d_queue->iterate_next ();
  }
  delete d_queue;
  d_queue = NULL;
  delete d_buffer;
  d_buffer = NULL;
  delete d_readRunning;
  d_readRunning = NULL;
  delete d_runThreadRunning;
  d_runThreadRunning = NULL;
  delete d_runBlock;
  d_runBlock = NULL;
  delete d_readBlock;
  d_readBlock = NULL;
}

bool
fusb_ephandle_darwin::start ()
{
  UInt8  direction, number, interval;
  UInt16 maxPacketSize;

// reset circular buffer
  d_buffer->reset ();

// reset the queue
  d_queue->num_used (0);
  d_queue->iterate_start ();
  s_node_ptr l_node = d_queue->iterate_next ();
  while (l_node) {
    l_node->both()->set (l_node, this);
    l_node->object()->reset ();
    l_node->set_available ();
    l_node = d_queue->iterate_next ();
  }

  d_pipeRef = d_transferType = 0;

  usb_dev_handle* dev = d_devhandle->get_usb_dev_handle ();
  if (! dev)
    USB_ERROR_STR (false, -ENXIO, "fusb_ephandle_darwin::start: "
		   "null device");

  darwin_dev_handle* device = (darwin_dev_handle*) dev->impl_info;
  if (! device)
    USB_ERROR_STR (false, -ENOENT, "fusb_ephandle_darwin::start: "
		   "device not initialized");

  if (usb_debug)
    fprintf (stderr, "fusb_ephandle_darwin::start: "
	     "dev = %p, device = %p\n", dev, device);

  d_interfaceRef = device->interface;
  if (! d_interfaceRef)
    USB_ERROR_STR (false, -EACCES, "fusb_ephandle_darwin::start: "
		   "interface used without being claimed");
  d_interface = *d_interfaceRef;

// get read or write pipe info (depends on "d_input_p")

  if (usb_debug > 3)
    fprintf (stderr, "fusb_ephandle_darwin::start "
	     "d_endpoint = %d, d_input_p = %s\n",
	     d_endpoint, d_input_p ? "TRUE" : "FALSE");

  int l_endpoint = (d_input_p ? USB_ENDPOINT_IN : USB_ENDPOINT_OUT);
  int pipeRef = ep_to_pipeRef (device, d_endpoint | l_endpoint);
  if (pipeRef < 0)
    USB_ERROR_STR (false, -EINVAL, "fusb_ephandle_darwin::start "
		   " invalid pipeRef.\n");

  d_pipeRef = pipeRef;
  d_interface->GetPipeProperties (d_interfaceRef,
				  d_pipeRef,
				  &direction,
				  &number,
				  &d_transferType,
				  &maxPacketSize,
				  &interval);
  if (usb_debug == 3)
    fprintf (stderr, "fusb_ephandle_darwin::start: %s: ep = 0x%02x, "
	     "pipeRef = %d, d_i = %p, d_iR = %p, if_dir = %d, if_# = %d, "
	     "if_int = %d, if_maxPS = %d\n", d_input_p ? "read" : "write",
	     d_endpoint, d_pipeRef, d_interface, d_interfaceRef, direction,
	     number, interval, maxPacketSize);

  // set global start boolean
  d_started = true;

  // lock the runBlock mutex, before creating the run thread.
  // this guarantees that we can control execution between these 2 threads
  d_runBlock->mutex ()->lock ();

  // create the run thread, which allows OSX to process I/O separately
  d_runThread = new mld_thread (run_thread, this);

  // wait until the run thread (and possibky read thread) are -really-
  // going; this will unlock the mutex before waiting for a signal ()
  d_runBlock->wait ();

  if (usb_debug)
    fprintf (stderr, "fusb_ephandle_darwin::start: %s started.\n",
	     d_input_p ? "read" : "write");

  return (true);
}

void
fusb_ephandle_darwin::run_thread (void* arg)
{
  fusb_ephandle_darwin* This = static_cast<fusb_ephandle_darwin*>(arg);

  // lock the run thread running mutex; if ::stop() is called, it will
  // first abort() the pipe then wait for the run thread to finish,
  // via a lock() on this mutex
  mld_mutex_ptr l_runThreadRunning = This->d_runThreadRunning;
  l_runThreadRunning->lock ();

  mld_mutex_ptr l_readRunning = This->d_readRunning;
  mld_condition_ptr l_readBlock = This->d_readBlock;
  mld_mutex_ptr l_readBlock_mutex = l_readBlock->mutex ();

  bool l_input_p = This->d_input_p;

  if (usb_debug)
    fprintf (stderr, "fusb_ephandle_darwin::run_thread: "
	     "starting for %s.\n",
	     l_input_p ? "read" : "write");

  usb_interface_t** l_interfaceRef = This->d_interfaceRef;
  usb_interface_t* l_interface = This->d_interface;
  CFRunLoopSourceRef l_cfSource;

// create async run loop
  l_interface->CreateInterfaceAsyncEventSource (l_interfaceRef, &l_cfSource);
  CFRunLoopAddSource (CFRunLoopGetCurrent (), l_cfSource,
		      kCFRunLoopDefaultMode);
// get run loop reference, to allow other threads to stop
  This->d_CFRunLoopRef = CFRunLoopGetCurrent ();

  mld_thread_ptr l_rwThread = NULL;

  if (l_input_p) {
    // lock the readBlock mutex, before creating the read thread.
    // this guarantees that we can control execution between these 2 threads
    l_readBlock_mutex->lock ();
    // create the read thread, which just issues all of the starting
    // async read commands, then returns
    l_rwThread = new mld_thread (read_thread, arg);
    // wait until the the read thread is -really- going; this will
    // unlock the read block mutex before waiting for a signal ()
    l_readBlock->wait ();
  }

  // now signal the run condition to release and finish ::start().

  // lock the runBlock mutex first; this will force waiting until the
  // ->wait() command is issued in ::start()
  mld_mutex_ptr l_run_block_mutex = This->d_runBlock->mutex ();
  l_run_block_mutex->lock ();

  // now that the lock is in place, signal the parent thread that
  // things are running
  This->d_runBlock->signal ();

  // release the run_block mutex, just in case
  l_run_block_mutex->unlock ();

  // run the loop
  CFRunLoopRun ();

  if (l_input_p) {
    // wait for read_thread () to finish, if needed
    l_readRunning->lock ();
    l_readRunning->unlock ();
  }

  // remove run loop stuff
  CFRunLoopRemoveSource (CFRunLoopGetCurrent (),
			 l_cfSource, kCFRunLoopDefaultMode);

  if (usb_debug)
    fprintf (stderr, "fusb_ephandle_darwin::run_thread: finished for %s.\n",
	     l_input_p ? "read" : "write");

  // release the run thread running mutex
  l_runThreadRunning->unlock ();
}

void
fusb_ephandle_darwin::read_thread (void* arg)
{
  if (usb_debug)
    fprintf (stderr, "fusb_ephandle_darwin::read_thread: starting.\n");

  fusb_ephandle_darwin* This = static_cast<fusb_ephandle_darwin*>(arg);

  // before doing anything else, lock the read running mutex.  this
  // mutex does flow control between this thread and the run_thread
  mld_mutex_ptr l_readRunning = This->d_readRunning;
  l_readRunning->lock ();

  // signal the read condition from run_thread() to continue

  // lock the readBlock mutex first; this will force waiting until the
  // ->wait() command is issued in ::run_thread()
  mld_condition_ptr l_readBlock = This->d_readBlock;
  mld_mutex_ptr l_read_block_mutex = l_readBlock->mutex ();
  l_read_block_mutex->lock ();

  // now that the lock is in place, signal the parent thread that
  // things are running here
  l_readBlock->signal ();

  // release the run_block mutex, just in case
  l_read_block_mutex->unlock ();

  // queue up all of the available read requests
  s_queue_ptr l_queue = This->d_queue;
  l_queue->iterate_start ();
  s_node_ptr l_node = l_queue->iterate_next ();
  while (l_node) {
    This->read_issue (l_node->both ());
    l_node = l_queue->iterate_next ();
  }

  if (usb_debug)
    fprintf (stderr, "fusb_ephandle_darwin::read_thread: finished.\n");

  // release the read running mutex, to let the parent thread knows
  // that this thread is finished
  l_readRunning->unlock ();
}

void
fusb_ephandle_darwin::read_issue (s_both_ptr l_both)
{
  if ((! l_both) || (! d_started)) {
    if (usb_debug > 4)
      fprintf (stderr, "fusb_ephandle_darwin::read_issue: Doing nothing; "
	       "l_both is %X; started is %s\n", (unsigned int) l_both,
	       d_started ? "TRUE" : "FALSE");
    return;
  }

// set the node and buffer from the input "both"
  s_node_ptr l_node = l_both->node ();
  s_buffer_ptr l_buf = l_node->object ();
  void* v_buffer = (void*) l_buf->buffer ();

// read up to d_bufLenBytes
  UInt32 bufLen = d_bufLenBytes;
  l_buf->n_used (bufLen);

// setup system call result
  io_return_t result = kIOReturnSuccess;

  if (d_transferType == kUSBInterrupt)
/* This is an interrupt pipe. We can't specify a timeout. */
    result = d_interface->ReadPipeAsync
      (d_interfaceRef, d_pipeRef, v_buffer, bufLen,
       (IOAsyncCallback1) read_completed, (void*) l_both);
  else
    result = d_interface->ReadPipeAsyncTO
      (d_interfaceRef, d_pipeRef, v_buffer, bufLen, 0, USB_TIMEOUT,
       (IOAsyncCallback1) read_completed, (void*) l_both);

  if (result != kIOReturnSuccess)
    USB_ERROR_STR_NO_RET (- darwin_to_errno (result),
			  "fusb_ephandle_darwin::read_issue "
			  "(ReadPipeAsync%s): %s",
			  d_transferType == kUSBInterrupt ? "" : "TO",
			  darwin_error_str (result));
  else if (usb_debug > 4)
    fprintf (stderr, "fusb_ephandle_darwin::read_issue: "
	     "Queued %X (%ld Bytes)\n", (unsigned int) l_both, bufLen);
}

void
fusb_ephandle_darwin::read_completed (void* refCon,
				      io_return_t result,
				      void* io_size)
{
  UInt32 l_size = (UInt32) io_size;
  s_both_ptr l_both = static_cast<s_both_ptr>(refCon);
  fusb_ephandle_darwin* This = static_cast<fusb_ephandle_darwin*>(l_both->This ());
  s_node_ptr l_node = l_both->node ();
  circular_buffer<char>* l_buffer = This->d_buffer;
  s_buffer_ptr l_buf = l_node->object ();
  UInt32 l_i_size = l_buf->n_used ();

  if (This->d_started && (l_i_size != l_size))
    fprintf (stderr, "fusb_ephandle_darwin::read_completed: "
	     "Expected %ld bytes; read %ld.\n",
	     l_i_size, l_size);
  else if (usb_debug > 4)
    fprintf (stderr, "fusb_ephandle_darwin::read_completed: "
	     "Read %X (%ld bytes)\n",
	     (unsigned int) l_both, l_size);

// add this read to the transfer buffer
  if (l_buffer->enqueue (l_buf->buffer (), l_size) == -1) {
    fputs ("iU", stderr);
    fflush (stderr);
  }

// set buffer's # data to 0
  l_buf->n_used (0);

// issue another read for this "both"
  This->read_issue (l_both);
}

int
fusb_ephandle_darwin::read (void* buffer, int nbytes)
{
  UInt32 l_nbytes = (UInt32) nbytes;
  d_buffer->dequeue ((char*) buffer, &l_nbytes);

  if (usb_debug > 4)
    fprintf (stderr, "fusb_ephandle_darwin::read: request for %d bytes, %ld bytes retrieved.\n", nbytes, l_nbytes);

  return ((int) l_nbytes);
}

int
fusb_ephandle_darwin::write (const void* buffer, int nbytes)
{
  UInt32 l_nbytes = (UInt32) nbytes;

  if (! d_started) {
    if (usb_debug)
      fprintf (stderr, "fusb_ephandle_darwin::write: Not yet started.\n");

    return (0);
  }

  while (l_nbytes != 0) {
// find out how much data to copy; limited to "d_bufLenBytes" per node
    UInt32 t_nbytes = (l_nbytes > d_bufLenBytes) ? d_bufLenBytes : l_nbytes;

// get next available node to write into;
// blocks internally if none available
    s_node_ptr l_node = d_queue->find_next_available_node ();

// copy the input into the node's buffer
    s_buffer_ptr l_buf = l_node->object ();
    l_buf->buffer ((char*) buffer, t_nbytes);
    void* v_buffer = (void*) l_buf->buffer ();

// setup callback parameter & system call return
    s_both_ptr l_both = l_node->both ();
    io_return_t result = kIOReturnSuccess;

    if (d_transferType == kUSBInterrupt)
/* This is an interrupt pipe ... can't specify a timeout. */
      result = d_interface->WritePipeAsync
	(d_interfaceRef, d_pipeRef, v_buffer, t_nbytes,
	 (IOAsyncCallback1) write_completed, (void*) l_both);
    else
      result = d_interface->WritePipeAsyncTO
	(d_interfaceRef, d_pipeRef, v_buffer, t_nbytes, 0, USB_TIMEOUT,
	 (IOAsyncCallback1) write_completed, (void*) l_both);

    if (result != kIOReturnSuccess)
      USB_ERROR_STR (-1, - darwin_to_errno (result),
		     "fusb_ephandle_darwin::write_thread "
		     "(WritePipeAsync%s): %s",
		     d_transferType == kUSBInterrupt ? "" : "TO",
		     darwin_error_str (result));
    else if (usb_debug > 4) {
      fprintf (stderr, "fusb_ephandle_darwin::write_thread: "
	       "Queued %X (%ld Bytes)\n", (unsigned int) l_both, t_nbytes);
    }
    l_nbytes -= t_nbytes;
  }

  return (nbytes);
}

void
fusb_ephandle_darwin::write_completed (void* refCon,
				       io_return_t result,
				       void* io_size)
{
  s_both_ptr l_both = static_cast<s_both_ptr>(refCon);
  fusb_ephandle_darwin* This = static_cast<fusb_ephandle_darwin*>(l_both->This ());
  UInt32 l_size = (UInt32) io_size;
  s_node_ptr l_node = l_both->node ();
  s_queue_ptr l_queue = This->d_queue;
  s_buffer_ptr l_buf = l_node->object ();
  UInt32 l_i_size = l_buf->n_used ();

  if (This->d_started && (l_i_size != l_size))
    fprintf (stderr, "fusb_ephandle_darwin::write_completed: "
	     "Expected %ld bytes written; wrote %ld.\n",
	     l_i_size, l_size);
  else if (usb_debug > 4)
    fprintf (stderr, "fusb_ephandle_darwin::write_completed: "
	     "Wrote %X (%ld Bytes)\n", (unsigned int) l_both, l_size);

// set buffer's # data to 0
  l_buf->n_used (0);
// make the node available for reuse
  l_queue->make_node_available (l_node);
}

void
fusb_ephandle_darwin::abort ()
{
  if (usb_debug)
    fprintf (stderr, "fusb_ephandle_darwin::abort: starting.\n");

  io_return_t result = d_interface->AbortPipe (d_interfaceRef, d_pipeRef);

  if (result != kIOReturnSuccess)
    USB_ERROR_STR_NO_RET (- darwin_to_errno (result),
			  "fusb_ephandle_darwin::abort "
			  "(AbortPipe): %s", darwin_error_str (result));
  if (usb_debug)
    fprintf (stderr, "fusb_ephandle_darwin::abort: finished.\n");
}

bool
fusb_ephandle_darwin::stop ()
{
  if (! d_started)
    return (true);

  if (usb_debug)
    fprintf (stderr, "fusb_ephandle_darwin::stop: stopping %s.\n",
	     d_input_p ? "read" : "write");

  d_started = false;

// abort any pending IO transfers
  abort ();

// wait for write transfer to finish
  wait_for_completion ();

// tell IO buffer to abort any waiting conditions
  d_buffer->abort ();

// stop the run loop
  CFRunLoopStop (d_CFRunLoopRef);

// wait for the runThread to stop
  d_runThreadRunning->lock ();
  d_runThreadRunning->unlock ();

  if (usb_debug)
    fprintf (stderr, "fusb_ephandle_darwin::stop: %s stopped.\n",
	     d_input_p ? "read" : "write");

  return (true);
}

void
fusb_ephandle_darwin::wait_for_completion ()
{
  if (d_queue)
    while (d_queue->in_use ())
      usleep (1000);
}
