#
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
# 

import glob
import os.path

# This automatically imports all top-level objects from .py files
# in our directory into the package name space
for _p in __path__:
    _filenames = glob.glob (os.path.join (_p, "*.py"))
    for _f in _filenames:
        _f = os.path.basename(_f).lower()
        _f = _f[:-3]
        if _f == '__init__':
            continue
        exec "from %s import *" % (_f,)
