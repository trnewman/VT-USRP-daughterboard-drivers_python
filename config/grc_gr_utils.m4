# Copyright 2007,2008 Free Software Foundation, Inc.
# 
# This file is part of GNU Radio
# 
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

AC_DEFUN([GRC_GR_UTILS],[
    GRC_ENABLE(gr-utils)

    dnl Don't do gr-utils if gnuradio-core, usrp, or gr-wxgui skipped
    GRC_CHECK_DEPENDENCY(gr-utils, gnuradio-core)
    GRC_CHECK_DEPENDENCY(gr-utils, usrp)
    GRC_CHECK_DEPENDENCY(gr-utils, gr-wxgui)

    AC_CONFIG_FILES([ \
        gr-utils/Makefile \
        gr-utils/src/Makefile \
        gr-utils/src/lib/Makefile \
        gr-utils/src/python/Makefile \
    ])

    GRC_BUILD_CONDITIONAL(gr-utils)
])
