dnl Copyright 2001,2002,2003,2004,2005,2006,2008 Free Software Foundation, Inc.
dnl 
dnl This file is part of GNU Radio
dnl 
dnl GNU Radio is free software; you can redistribute it and/or modify
dnl it under the terms of the GNU General Public License as published by
dnl the Free Software Foundation; either version 3, or (at your option)
dnl any later version.
dnl 
dnl GNU Radio is distributed in the hope that it will be useful,
dnl but WITHOUT ANY WARRANTY; without even the implied warranty of
dnl MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
dnl GNU General Public License for more details.
dnl 
dnl You should have received a copy of the GNU General Public License
dnl along with GNU Radio; see the file COPYING.  If not, write to
dnl the Free Software Foundation, Inc., 51 Franklin Street,
dnl Boston, MA 02110-1301, USA.

AC_DEFUN([GRC_GR_USRP],[
    GRC_ENABLE(gr-usrp)

    dnl Don't do gr-usrp if usrp or gnuradio-core skipped
    GRC_CHECK_DEPENDENCY(gr-usrp, usrp)
    GRC_CHECK_DEPENDENCY(gr-usrp, gnuradio-core)

    AC_CONFIG_FILES([ \
        gr-usrp/Makefile \
        gr-usrp/src/Makefile \
        gr-usrp/src/run_tests \
    ])

    GRC_BUILD_CONDITIONAL(gr-usrp,[
	dnl run_tests is created from run_tests.in.  Make it executable.
	AC_CONFIG_COMMANDS([run_tests_usrp], [chmod +x gr-usrp/src/run_tests])
    ])
])
