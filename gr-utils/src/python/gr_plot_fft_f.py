#!/usr/bin/env python
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

try:
    import scipy
    from scipy import fftpack
except ImportError:
    print "Please install SciPy to run this script (http://www.scipy.org/)"
    raise SystemExit, 1

try:
    from pylab import *
except ImportError:
    print "Please install Matplotlib to run this script (http://matplotlib.sourceforge.net/)"
    raise SystemExit, 1

from optparse import OptionParser
from math import log10

matplotlib.interactive(True)
matplotlib.use('TkAgg')

class draw_fft_f:
    def __init__(self, filename, options):
        self.hfile = open(filename, "r")
        self.block_length = options.block
        self.start = options.start
        self.sample_rate = options.sample_rate

        self.datatype = scipy.float32
        self.sizeof_data = self.datatype().nbytes    # number of bytes per sample in file

        self.axis_font_size = 16
        self.label_font_size = 18
        self.title_font_size = 20
        self.text_size = 22

        # Setup PLOT
        self.fig = figure(1, figsize=(16, 9), facecolor='w')
        rcParams['xtick.labelsize'] = self.axis_font_size
        rcParams['ytick.labelsize'] = self.axis_font_size
        
        self.text_file     = figtext(0.10, 0.94, ("File: %s" % filename), weight="heavy", size=self.text_size)
        self.text_file_pos = figtext(0.10, 0.88, "File Position: ", weight="heavy", size=self.text_size)
        self.text_block    = figtext(0.40, 0.88, ("Block Size: %d" % self.block_length),
                                     weight="heavy", size=self.text_size)
        self.text_sr       = figtext(0.60, 0.88, ("Sample Rate: %.2f" % self.sample_rate),
                                     weight="heavy", size=self.text_size)
        self.make_plots()

        self.button_left_axes = self.fig.add_axes([0.45, 0.01, 0.05, 0.05], frameon=True)
        self.button_left = Button(self.button_left_axes, "<")
        self.button_left_callback = self.button_left.on_clicked(self.button_left_click)

        self.button_right_axes = self.fig.add_axes([0.50, 0.01, 0.05, 0.05], frameon=True)
        self.button_right = Button(self.button_right_axes, ">")
        self.button_right_callback = self.button_right.on_clicked(self.button_right_click)

        self.xlim = self.sp_f.get_xlim()

        self.manager = get_current_fig_manager()
        connect('draw_event', self.zoom)
        connect('key_press_event', self.click)
        show()
        
    def get_data(self):
        self.text_file_pos.set_text("File Position: %d" % (self.hfile.tell()//self.sizeof_data))
        self.floats = scipy.fromfile(self.hfile, dtype=self.datatype, count=self.block_length)
        #print "Read in %d items" % len(self.floats)
        if(len(self.floats) == 0):
            print "End of File"
        else:
            self.f_fft = self.dofft(self.floats)

            self.time = [i*(1/self.sample_rate) for i in range(len(self.floats))]
            self.freq = self.calc_freq(self.time, self.sample_rate)
            
    def dofft(self, f):
        N = len(f)
        f_fft = fftpack.fftshift(scipy.fft(f))       # fft and shift axis
        f_dB = list()
        for f in f_fft:
            try:
                f_dB.append(20*log10(abs(f/N)))  # convert to decibels, adjust power
            except OverflowError:                # protect against taking log(0)
                f = 1e-14                        # not sure if this is the best way to do this
                f_dB.append(20*log10(abs(f/N)))
                
        return f_dB

    def calc_freq(self, time, sample_rate):
        N = len(time)
        Fs = 1.0 / (max(time) - min(time))
        Fn = 0.5 * sample_rate
        freq = [-Fn + i*Fs for i in range(N)]
        return freq
        
    def make_plots(self):
        # if specified on the command-line, set file pointer
        self.hfile.seek(self.sizeof_data*self.start, 1)

        self.get_data()
        
        # Subplot for real and imaginary parts of signal
        self.sp_f = self.fig.add_subplot(2,1,1, position=[0.075, 0.2, 0.4, 0.6])
        self.sp_f.set_title(("Amplitude"), fontsize=self.title_font_size, fontweight="bold")
        self.sp_f.set_xlabel("Time (s)", fontsize=self.label_font_size, fontweight="bold")
        self.sp_f.set_ylabel("Amplitude (V)", fontsize=self.label_font_size, fontweight="bold")
        self.plot_f = plot(self.time, self.floats, 'bo-')
        self.sp_f.set_ylim([1.5*min(self.floats),
                            1.5*max(self.floats)])

        # Subplot for constellation plot
        self.sp_fft = self.fig.add_subplot(2,2,1, position=[0.575, 0.2, 0.4, 0.6])
        self.sp_fft.set_title(("FFT"), fontsize=self.title_font_size, fontweight="bold")
        self.sp_fft.set_xlabel("Frequency (Hz)", fontsize=self.label_font_size, fontweight="bold")
        self.sp_fft.set_ylabel("Power (dBm)", fontsize=self.label_font_size, fontweight="bold")
        self.plot_fft = plot(self.freq, self.f_fft, '-bo')
        self.sp_fft.set_ylim([min(self.f_fft)-10, max(self.f_fft)+10])
        
        draw()

    def update_plots(self):
        self.plot_f[0].set_data([self.time, self.floats])
        self.sp_f.set_ylim([1.5*min(self.floats),
                            1.5*max(self.floats)])

        self.plot_fft[0].set_data([self.freq, self.f_fft])
        self.sp_fft.set_ylim([min(self.f_fft)-10, max(self.f_fft)+10])

        draw()
        
    def zoom(self, event):
        newxlim = self.sp_f.get_xlim()
        if(newxlim != self.xlim):
            self.xlim = newxlim
            xmin = max(0, int(ceil(self.sample_rate*self.xlim[0])))
            xmax = min(int(ceil(self.sample_rate*self.xlim[1])), len(self.floats))

            f = self.floats[xmin : xmax]
            time = self.time[xmin : xmax]
            
            f_fft = self.dofft(f)
            freq = self.calc_freq(time, self.sample_rate)
                        
            self.plot_fft[0].set_data(freq, f_fft)
            self.sp_fft.axis([min(freq), max(freq),
                              min(f_fft)-10, max(f_fft)+10])

            draw()

    def click(self, event):
        forward_valid_keys = [" ", "down", "right"]
        backward_valid_keys = ["up", "left"]

        if(find(event.key, forward_valid_keys)):
            self.step_forward()
            
        elif(find(event.key, backward_valid_keys)):
            self.step_backward()

    def button_left_click(self, event):
        self.step_backward()

    def button_right_click(self, event):
        self.step_forward()

    def step_forward(self):
        self.get_data()
        self.update_plots()

    def step_backward(self):
        # Step back in file position
        if(self.hfile.tell() >= 2*self.sizeof_data*self.block_length ):
            self.hfile.seek(-2*self.sizeof_data*self.block_length, 1)
        else:
            self.hfile.seek(-self.hfile.tell(),1)
        self.get_data()
        self.update_plots()
        
            
def find(item_in, list_search):
    try:
	return list_search.index(item_in) != None
    except ValueError:
	return False
		
def main():
    usage="%prog: [options] input_filename"
    description = "Takes a GNU Radio floating point binary file and displays the sample data versus time as well as the frequency domain (FFT) plot. The y-axis values are plotted assuming volts as the amplitude of the I&Q streams and converted into dBm in the frequency domain (the 1/N power adjustment out of the FFT is performed internally). The script plots a certain block of data at a time, specified on the command line as -B or --block. This value defaults to 1000. The start position in the file can be set by specifying -s or --start and defaults to 0 (the start of the file). By default, the system assumes a sample rate of 1, so in time, each sample is plotted versus the sample number. To set a true time and frequency axis, set the sample rate (-R or --sample-rate) to the sample rate used when capturing the samples."

    parser = OptionParser(conflict_handler="resolve", usage=usage, description=description)
    parser.add_option("-B", "--block", type="int", default=1000,
                      help="Specify the block size [default=%default]")
    parser.add_option("-s", "--start", type="int", default=0,
                      help="Specify where to start in the file [default=%default]")
    parser.add_option("-R", "--sample-rate", type="float", default=1.0,
                      help="Set the sampler rate of the data [default=%default]")
    
    (options, args) = parser.parse_args ()
    if len(args) != 1:
        parser.print_help()
        raise SystemExit, 1
    filename = args[0]

    dc = draw_fft_f(filename, options)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    


