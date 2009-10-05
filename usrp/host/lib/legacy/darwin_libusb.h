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

/*
 * The following code was taken from LIBUSB verion 0.1.10a,
 * and makes the fusb_darwin codes do-able in the current GR
 * programming framework.  Parts and pieces were taken from
 * usbi.h, darwin.c, and error.h .
 *
 * LIBUSB version 0.1.10a is covered by the LGPL, version 2;
 * These codes are used with permission from:
 *   (c) 2000-2003 Johannes Erdfelt <johannes@erdfelt.com>
 *   (c) 2002-2005 Nathan Hjelm <hjelmn@users.sourceforge.net>
 * All rights reserved.
 */

#ifndef __DARWIN_LIBUSB_H__
#define __DARWIN_LIBUSB_H__

#include <IOKit/IOCFBundle.h>
#include <IOKit/IOCFPlugIn.h>
#include <IOKit/usb/IOUSBLib.h>
#include <IOKit/IOKitLib.h>

extern "C" {
static char *
darwin_error_str (int result)
{
  switch (result) {
  case kIOReturnSuccess:
    return "no error";
  case kIOReturnNotOpen:
    return "device not opened for exclusive access";
  case kIOReturnNoDevice:
    return "no connection to an IOService";
  case kIOUSBNoAsyncPortErr:
    return "no asyc port has been opened for interface";
  case kIOReturnExclusiveAccess:
    return "another process has device opened for exclusive access";
  case kIOUSBPipeStalled:
    return "pipe is stalled";
  case kIOReturnError:
    return "could not establish a connection to Darin kernel";
  case kIOReturnBadArgument:
    return "invalid argument";
  default:
    return "unknown error";
  }
}

/* not a valid errorno outside darwin.c */
#define LUSBDARWINSTALL (ELAST+1)

static int
darwin_to_errno (int result)
{
  switch (result) {
  case kIOReturnSuccess:
    return 0;
  case kIOReturnNotOpen:
    return EBADF;
  case kIOReturnNoDevice:
  case kIOUSBNoAsyncPortErr:
    return ENXIO;
  case kIOReturnExclusiveAccess:
    return EBUSY;
  case kIOUSBPipeStalled:
    return LUSBDARWINSTALL;
  case kIOReturnBadArgument:
    return EINVAL;
  case kIOReturnError:
  default:
    return 1;
  }
}

typedef enum {
  USB_ERROR_TYPE_NONE = 0,
  USB_ERROR_TYPE_STRING,
  USB_ERROR_TYPE_ERRNO,
} usb_error_type_t;

extern char usb_error_str[1024];
extern int usb_error_errno;
extern usb_error_type_t usb_error_type;

#define USB_ERROR(r, x)				\
	do { \
          usb_error_type = USB_ERROR_TYPE_ERRNO; \
          usb_error_errno = x; \
	  return r; \
	} while (0)

#define USB_ERROR_STR(r, x, format, args...)	\
	do { \
	  usb_error_type = USB_ERROR_TYPE_STRING; \
	  snprintf(usb_error_str, sizeof(usb_error_str) - 1, format, ## args); \
          if (usb_debug) \
            fprintf(stderr, "USB error: %s\n", usb_error_str);	\
	  return r; \
	} while (0)

#define USB_ERROR_STR_ORIG(x, format, args...)	\
	do { \
	  usb_error_type = USB_ERROR_TYPE_STRING; \
	  snprintf(usb_error_str, sizeof(usb_error_str) - 1, format, ## args); \
          if (usb_debug) \
            fprintf(stderr, "USB error: %s\n", usb_error_str);	\
	  return x; \
	} while (0)

#define USB_ERROR_STR_NO_RET(x, format, args...)	\
	do { \
	  usb_error_type = USB_ERROR_TYPE_STRING; \
	  snprintf(usb_error_str, sizeof(usb_error_str) - 1, format, ## args); \
          if (usb_debug) \
            fprintf(stderr, "USB error: %s\n", usb_error_str);	\
	} while (0)

/* simple function that figures out what pipeRef is associated with an endpoint */
static int ep_to_pipeRef (darwin_dev_handle *device, int ep)
{
  io_return_t ret;
  UInt8 numep, direction, number;
  UInt8 dont_care1, dont_care3;
  UInt16 dont_care2;
  int i;

  if (usb_debug > 3)
    fprintf(stderr, "Converting ep address to pipeRef.\n");

  /* retrieve the total number of endpoints on this interface */
  ret = (*(device->interface))->GetNumEndpoints(device->interface, &numep);
  if ( ret ) {
    if ( usb_debug > 3 )
      fprintf ( stderr, "ep_to_pipeRef: interface is %p\n", device->interface );
    USB_ERROR_STR_ORIG ( -ret, "ep_to_pipeRef: can't get number of endpoints for interface" );
  }

  /* iterate through the pipeRefs until we find the correct one */
  for (i = 1 ; i <= numep ; i++) {
    ret = (*(device->interface))->GetPipeProperties(device->interface, i, &direction, &number,
						    &dont_care1, &dont_care2, &dont_care3);

    if (ret != kIOReturnSuccess) {
      fprintf (stderr, "ep_to_pipeRef: an error occurred getting pipe information on pipe %d\n",
	       i );
      USB_ERROR_STR_ORIG (-darwin_to_errno(ret), "ep_to_pipeRef(GetPipeProperties): %s", darwin_error_str(ret));
    }

    if (usb_debug > 3)
      fprintf (stderr, "ep_to_pipeRef: Pipe %i: DIR: %i number: %i\n", i, direction, number);

    /* calculate the endpoint of the pipe and check it versus the requested endpoint */
    if ( ((direction << 7 & USB_ENDPOINT_DIR_MASK) | (number & USB_ENDPOINT_ADDRESS_MASK)) == ep ) {
      if (usb_debug > 3)
	fprintf(stderr, "ep_to_pipeRef: pipeRef for ep address 0x%02x found: 0x%02x\n", ep, i);

      return i;
    }
  }

  if (usb_debug > 3)
    fprintf(stderr, "ep_to_pipeRef: No pipeRef found with endpoint address 0x%02x.\n", ep);
  
  /* none of the found pipes match the requested endpoint */
  return -1;
}

}
#endif /* __DARWIN_LIBUSB_H__ */
