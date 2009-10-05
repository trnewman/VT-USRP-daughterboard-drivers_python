#
# Copyright 2007 Free Software Foundation, Inc.
# 
# This file is part of GNU Radio
# 
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
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

from gnuradio import usrp1, gru, eng_notation
import time, math, weakref
from math import floor

from usrpm import usrp_dbid
import db_base
import db_instantiator
from usrpm.usrp_fpga_regs import *

# Convenience function
n2s = eng_notation.num_to_str

# TX/RX Switch IO Pin (on the RX side, pin IO_RX_06)
TX_EN = (1 << 6)           # 1 = TX on, 0 = RX on

# ------------------------------------------------------------------------
# A few comments about the RFIC:
#
# The board is full duplex, meaning that the transmitter and receiver ca
# be used simultaneously.  There are seperate LOs for TX and RX as well
# as a third LO for the feedback from TX to RX, which can be used to 
# offset non-linearity or DC offset.
# The feedback can be enabled from the receiver.  Receiver, transmitter
# and feedback can be set independently.  Gain and frequency can be
# controlled in all three modes, as well as phase offset in the LO
# and bandwidth.
# The board is a direct-conversion transciever, so bandwidth is measured
# at baseband and any received signal will come into the host computer
# at baseband.
#
# Each board is uniquely identified by the *USRP hardware* instance and side
# This dictionary holds a weak reference to existing board controller so it
# can be created or retrieved as needed.

_rfic_inst = weakref.WeakValueDictionary()
def _get_or_make_rfic(usrp, which):
	key = (usrp.serial_number(), which)
	if not _rfic_inst.has_key(key):
		print "Creating new RFIC instance"
		inst = rfic(usrp, which)
		_rfic_inst[key] = inst
	else:
		print "Using existing RFIC instance"
		inst = _rfic_inst[key]
	return inst

# ------------------------------------------------------------------------
# Common, shared object for RFIC board.  Transmit and receive classes
# operate on an instance of this; one instance is created per physical
# daughterboard.
class rfic(object):
	def __init__(self, usrp, which):
		print "RFIC: __init__ with %s: %d" % (usrp.serial_number(), which)
		self.u = usrp
		self.which = which

		# For SPI interface, use MSB with two-byte header
		# Use RX side for SPI interface
		self.spi_format = usrp1.SPI_FMT_MSB | usrp1.SPI_FMT_HDR_2
		self.spi_format_no_header = usrp1.SPI_FMT_MSB | usrp1.SPI_FMT_HDR_0
		self.spi_enable = (usrp1.SPI_ENABLE_RX_A, usrp1.SPI_ENABLE_RX_B)[which]

		# Sane defaults:
		# For more information about setting each variable and SPI register, see RFIC4 SPI Default Variables.xls


		#-------------------------------------------------------------------------------------------------------
		# TRANSMIT SIDE QuIET Frequency Generator
		#-------------------------------------------------------------------------------------------------------
		self.Ngt3 = 0 #Output frequency control bit.  Calculated.#
		self.NorNdiv4 = 1 #Output frequency control word.  Calculated.#
		self.RorFrNpRdiv4_25to18 = 0 #Output frequency control word.  Calculated#
		self.RorFrNpRdiv4_17to10 = 0 ##
		self.RorFrNpRdiv4_9to2 = 0 ##
		self.RorFrNpRdiv4_1to0 = 0 ##
		self.Qu_tx_Ngt3 = 0 #Enables divide-by-4 freq divider - Phase shift control bit.  Calculated#
		self.NorNdiv4_phsh = 1 #Phase shift control word.  Calculated.#
		self.RorFrNpRdiv4_phsh_25to18 = 0 #Phase shift control word.  Calculated.#
		self.RorFrNpRdiv4_phsh_17to10 = 0 ##
		self.RorFrNpRdiv4_phsh_9to2 = 0 ##
		self.RorFrNpRdiv4_phsh_1to0 = 0 ##
		self.Passthru_ref_clk = 0 #A test mode where the 1 GHz input reference is passed directly to the output#
		self.Byp_ram = 1 #Bypass the SRAMs#
		self.Dis_adr_dith = 1 #Disable the dither generator in the ca2adr block#
		self.Dis_p5G_dith = 1 #Disable the dither generator in the lup2decod block#
		self.Byp_fine = 1 #Bypass fine delay line control bit#
		self.Exclude32 = 0 #Bypass fine delay line control bit (exclude 32)#
		self.Dis_risedge = 0 #Disable the rising edges decoders#
		self.Dis_faledge = 0 #Disable the falling edges decoders#
		self.Spr_puls_en = 0 #enable spur pulsing#
		self.Spr_puls_val_a_9to3 = 0 #spur pulsing control word#
		self.Spr_pulse_val_2to0 = 0 ##
		self.Spr_puls_val_b_9to2 = 8 #spur pulsing control word#
		self.Spr_puls_val_b_1to0 = 0 ##
		self.Thru_ris_en = 0 #Put rising edges decoders into through-tap mode#
		self.Thru_ris_tap_11to6 = 32 #Through-tap control word#
		self.Thru_ris_tap_5to0 = 0 ##
		self.Thru_fal_en = 0 #Put falling edges decoders into through-tap mode#
		self.Thru_fal_tap_11to6 = 32 #Through-tap control word#
		self.Thru_fal_tap_5to0 = 0 ##
		self.Dig_delay = 0 #This bit provides delay to the clock going into the digital block. It is a remnant of past designs and should always be left off because the digClkPhase setting in address 23 provides much finer control.#
		self.Clk_driver_en = 0 #This allows the clock to reach the digital block. It first passes through the digital/analog clock synchronization mux, which means that dlEn must be on (dlEn=1) and Clk_driver=1 for the digital block to receive a clock.  See Byp_fine, address 10, bit 6#
		self.qu_reg_en = 0 #This bit enables the voltage regulators that supply 1.2 V to all the analog block functions. There are 6 separate regulators that are simultaneously enabled by this bit.#
		self.qq_reg_en = 0 #This bit enables the voltage regulators that supply 1.2 V to all the Quad Gen functions. There are 3 separate regulators that are simultaneously enabled by this bit.#
		self.win_rst = 0 #When this bit is high, the windowing function is in a reset state, which means that no taps will be passed to the DDS output regardless of the tap select signals coming from the digital block.#
		self.fineEn = 0 #This bit, when high, routes the coarse taps through the fine line before reaching the output RS Flip Flop of the DDS. When low, the coarse tap is routed directly to the output RS Flip Flop.#
		self.fineEnb = 0 #Opposite of  fineEn#
		self.rsffEn = 0 #This bit must be high to send the QuIET 0 and 180 degree calibration signals off chip. It does not control the RS Flip Flop outputs of the DDS, though it may have some second order (coupling) effect.#
		self.dl_en = 1 #Allows the PLL reference to enter the QuIET delay line when enabled.#
		self.cp_en = 1 #This bit, when enables, activates the charge pump that controls the delay line via the single pole (one capacitor) DLL loop filter.#
		self.forceCpUpb = 0 #This bit only matters when pdEn=0 (address 22, bit 1). When low, the pmos device connected to the DLL loop filter cap turns on and sources current into the cap, thereby increasing the delay line control voltage. #
		self.forceCpDn = 0 #This bit only matters when pdEn=0 (address 22, bit 1). When low, the nmos device connected to the DLL loop filter cap turns off and allows the pmos device to charge up the loop cap as described above.#
		self.pdUpTune_1to0 = 3 #These bits control the pulse width from the phase detector into the charge up port of the charge pump. 00 turns the charge up signal off. 01 is the minimum pulse width setting and 11 is the maximum pulse width setting.#
		self.pdDnTune_1to0 = 0 #These bits control the pulse width from the phase detector into the charge down port of the charge pump. 00 turns the charge down signal off. 01 is the minimum pulse width setting and 11 is the maximum pulse width setting.#
		self.cpUpTune_2to0 = 7 #These bits control amount of current that is sourced while the charge up signal from the phase detector is high. 000 is minimum current and 111 is maximum current.#
		self.cpDnTune_2to0 = 2 #These bits control amount of current that is sinked while the charge down signal from the phase detector is high. 000 is minimum current and 111 is maximum current.#
		self.pdEn = 1 #When enables, the phase detector will send charge up and down signals to the charge pump and over ride the forceCpUp and forceCpDn settings in address 21. When disabled, the forceCpUp and forceCpDn settings will control the charge pump.#
		self.digClkPhase_7to0 = 4 #Only one bit in this field should be active at one time. This signal drives a mux that selects one of eight clock phases from the delay line to drive the digital block. This is needed to control the windowing function of the DDS.#
		self.Rst_n_async = 0 #Digital reset#
		self.L1_lup00_15to8 = [] #Read-only#
		self.L1_lup90_15to8 = [] #Read-only#
		self.Merg_ris_fin = [] #Read-only#
		self.Merg_fal_fin = [] #Read-only#
		self.Qg00degDelay_0to4 = 31 #Adjusts series delay in the 0 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.#
		self.Qg90degDelay_0to4 = 7 #Adjusts series delay in the 90 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.#
		self.Qg180degDelay_0to4 = 31 #Adjusts series delay in the 180 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.#
		self.Qg270degDelay_0to4 = 7 #Adjusts series delay in the 270 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.#
		self.DischargeTap16_0to3 = 15 #Adjusts DLL offset error in the Quad Gen delay line by controlling down currents in one of the parallel charge pumps.#
		self.ChargeTap16_0to3 = 4 #Adjusts DLL offset error in the Quad Gen delay line by controlling up currents in one of the parallel charge pumps.#
		self.DischargeTapn_0to3 = 15 #Adjusts DLL offset error in the Quad Gen delay line by controlling down currents in one of the parallel charge pumps.#
		self.ChargeTapn16_0to3 = 2 #Adjusts DLL offset error in the Quad Gen delay line by controlling up currents in one of the parallel charge pumps.#
		self.X1sel_32to39 = 0 #Control for the divide-by-two and x1 functions.#
		self.X1sel_40to47 = 0 #Control for the divide-by-two and x1 functions.#
		self.X2sel_32to36 = 0 #Control for the x2 function.#
		self.X2sel_37to41 = 0 #Control for the x2 function.#
		self.X4sel_32to36 = 0 #Control for the x4 function.#
		self.X4sel_37to41 = 0 #Control for the x4 function.#
		self.X8sel_32to36 = 0 #Bit 41 is used for a fine line windowing control bit. If the fine line is needed, this bit needs to be set high if Fout is close to Fref (greater than ~ 950 MHz) or for some testing modes like pass through or thru_rise_en.#
		self.X8sel_41 = 0 #hiFout - set for passthrough and Fout close to Fref#
		self.X8sel_37to40 = 0 ##
		self.qutx_fwd_180Cal_en = 0 #Enables the pad driver that sends the falling edge signal off chip. This falling edge signal is used internally to trigger the 'Reset' pin of the output RS Flip Flop.#
		self.qutx_fwd_0Cal_en = 0 #Enables the pad driver that sends the rising edge signal off chip. This rising edge signal is used internally to trigger the 'Set' pin of the output RS Flip Flop.#
		#-------------------------------------------------------------------------------------------------------
		# TRANSMIT FEEDBACK QuIET FREQUENCY GENERATOR
		#-------------------------------------------------------------------------------------------------------
		self.Ngt3_2 = 0 #Output frequency control bit.  Calculated.#
		self.NorNdiv4_2 = 1 #Output frequency control word.  Calculated.#
		self.RorFrNpRdiv4_25to18_2 = 0 #Output frequency control word.  Calculated.#
		self.RorFrNpRdiv4_17to10_2 = 0 ##
		self.RorFrNpRdiv4_9to2_2 = 0 ##
		self.RorFrNpRdiv4_1to0_2 = 0 ##
		self.Qu_tx_Ngt3_2 = 0 #Enables divide-by-4 freq divider - Phase shift control bit.  Calculated#
		self.NorNdiv4_phsh_2 = 1 #Phase shift control word.  Calculated#
		self.RorFrNpRdiv4_phsh_25to18_2 = 0 #Phase shift control word.  Calculated#
		self.RorFrNpRdiv4_phsh_17to10_2 = 0 ##
		self.RorFrNpRdiv4_phsh_9to2_2 = 0 ##
		self.RorFrNpRdiv4_phsh_1to0_2 = 0 ##
		self.Passthru_ref_clk_2 = 0 #Enable reference clock pass-through mode#
		self.Byp_ram_2 = 1 #Bypass the SRAMs#
		self.Dis_adr_dith_2 = 1 #Disable the dither generator in the ca2adr block#
		self.Dis_p5G_dith_2 = 1 #Disable the dither generator in the lup2decod block#
		self.Byp_fine_2 = 1 #Bypass fine delay line control bit#
		self.Exclude32_2 = 0 #Bypass fine delay line control bit (exclude 32)#
		self.Dis_risedge_2 = 0 #Disable the rising edges decoders#
		self.Dis_faledge_2 = 0 #Disable the falling edges decoders#
		self.Spr_puls_en_2 = 0 #Enable spur pulsing mode#
		self.Spr_puls_val_a_9to3_2 = 0 #Spur pulsing mode control word#
		self.Spr_pulse_val_2to0_2 = 0 ##
		self.Spr_puls_val_b_9to2_2 = 8 #Spur pulsing mode control word#
		self.Spr_puls_val_b_1to0_2 = 0 ##
		self.Thru_ris_en_2 = 0 #Put rising edges decoders into through-tap mode#
		self.Thru_ris_tap_11to6_2 = 32 #Through-tap mode control word#
		self.Thru_ris_tap_5to0_2 = 0 #Through-tap mode control word#
		self.Thru_fal_en_2 = 0 #Put falling edges decoders into through-tap mode#
		self.Thru_fal_tap_11to6_2 = 32 #Through-tap mode control word#
		self.Thru_fal_tap_5to0_2 = 0 #Through-tap mode control word#
		self.Dig_delay_2 = 0 #This bit provides delay to the clock going into the digital block. It is a remnant of past designs and should always be left off because the digClkPhase setting in address 23 provides much finer control.#
		self.Clk_driver_en_2 = 0 #This bit provides delay to the clock going into the digital block. It is a remnant of past designs and should always be left off because the digClkPhase setting in address 23 provides much finer control.  See Byp_fine, address 10, bit 6#
		self.qu_reg_en_2 = 0 #This bit enables the voltage regulators that supply 1.2 V to all the analog block functions. There are 6 separate regulators that are simultaneously enabled by this bit.#
		self.qq_reg_en_2 = 0 #This bit enables the voltage regulators that supply 1.2 V to all the Quad Gen functions. There are 3 separate regulators that are simultaneously enabled by this bit.#
		self.win_rst_2 = 0 #When this bit is high, the windowing function is in a reset state, which means that no taps will be passed to the DDS output regardless of the tap select signals coming from the digital block.#
		self.fineEn_2 = 0 #This bit, when high, routes the coarse taps through the fine line before reaching the output RS Flip Flop of the DDS. When low, the coarse tap is routed directly to the output RS Flip Flop.#
		self.fineEnb_2 = 0 #Opposite of  fineEn.#
		self.rsffEn_2 = 0 #This bit must be high to send the QuIET 0 and 180 degree calibration signals off chip. It does not control the RS Flip Flop outputs of the DDS, though it may have some second order (coupling) effect.#
		self.dl_en_2 = 1 #Allows the PLL reference to enter the QuIET delay line when enabled.#
		self.cp_en_2 = 1 #This bit, when enables, activates the charge pump that controls the delay line via the single pole (one capacitor) DLL loop filter.#
		self.forceCpUpb_2 = 0 #This bit only matters when pdEn=0 (address 22, bit 1). When low, the pmos device connected to the DLL loop filter cap turns on and sources current into the cap, thereby increasing the delay line control voltage. #
		self.forceCpDn_2 = 0 #This bit only matters when pdEn=0 (address 22, bit 1). When low, the nmos device connected to the DLL loop filter cap turns off and allows the pmos device to charge up the loop cap as described above.#
		self.pdUpTune_1to0_2 = 3 #These bits control the pulse width from the phase detector into the charge up port of the charge pump. 00 turns the charge up signal off. 01 is the minimum pulse width setting and 11 is the maximum pulse width setting.#
		self.pdDnTune_1to0_2 = 0 #These bits control the pulse width from the phase detector into the charge down port of the charge pump. 00 turns the charge down signal off. 01 is the minimum pulse width setting and 11 is the maximum pulse width setting.#
		self.cpUpTune_2to0_2 = 7 #These bits control amount of current that is sourced while the charge up signal from the phase detector is high. 000 is minimum current and 111 is maximum current.#
		self.cpDnTune_2to0_2 = 2 #These bits control amount of current that is sinked while the charge down signal from the phase detector is high. 000 is minimum current and 111 is maximum current.#
		self.pdEn_2 = 1 #When enables, the phase detector will send charge up and down signals to the charge pump and over ride the forceCpUp and forceCpDn settings in address 21. When disabled, the forceCpUp and forceCpDn settings will control the charge pump.#
		self.digClkPhase_7to0_2 = 4 #Only one bit in this field should be active at one time. This signal drives a mux that selects one of eight clock phases from the delay line to drive the digital block. This is needed to control the windowing function of the DDS.#
		self.Rst_n_async_2 = 0 #Digital reset#
		self.L1_lup00_15to8_2 = [] #Read-only#
		self.L1_lup90_15to8_2 = [] #Read-only#
		self.Merg_ris_fin_2 = [] #Read-only#
		self.Merg_fal_fin_2 = [] #Read-only#
		self.Qg00degDelay_0to4_2 = 31 #Adjusts series delay in the 0 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.#
		self.Qg90degDelay_0to4_2 = 7 ##Adjusts series delay in the 90 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.
		self.Qg180degDelay_0to4_2 = 31 #Adjusts series delay in the 180 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.#
		self.Qg270degDelay_0to4_2 = 7 #Adjusts series delay in the 270 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.#
		self.DischargeTap16_3to0 = 15 #Adjusts DLL offset error in the Quad Gen delay line by controlling down currents in one of the parallel charge pumps.#
		self.ChargeTap16_3to0 = 4 #Adjusts DLL offset error in the Quad Gen delay line by controlling up currents in one of the parallel charge pumps.#
		self.DischargeTapn_3to0 = 15 #Adjusts DLL offset error in the Quad Gen delay line by controlling down currents in one of the parallel charge pumps.#
		self.ChargeTapn16_3to0 = 2 #Adjusts DLL offset error in the Quad Gen delay line by controlling up currents in one of the parallel charge pumps.#
		self.X1sel_32to39_2 = 0 #Control for the divide-by-two and x1 functions.#
		self.X1sel_40to47_2 = 0 #Control for the divide-by-two and x1 functions.#
		self.X2sel_32to36_2 = 0 #Control for the x2 function.#
		self.X2sel_37to41_2 = 0 #Control for the x2 function.#
		self.X4sel_32to36_2 = 0 #Control for the x4 function.#
		self.X4sel_37to41_2 = 0 #Control for the x4 function.#
		self.X8sel_32to36_2 = 0 #Bit 41 is used for a fine line windowing control bit. If the fine line is needed, this bit needs to be set high if Fout is close to Fref (greater than ~ 950 MHz) or for some testing modes like pass through or thru_rise_en.#
		self.X8sel_41_2 = 0 #hiFout - set for passthrough and Fout close to Fref#
		self.X8sel_37to40_2 = 0 ##
		self.qutx_fb_180Cal_en = 0 #Enables the pad driver that sends the falling edge signal off chip. This falling edge signal is used internally to trigger the 'Reset' pin of the output RS Flip Flop.#
		self.qutx_fb_0Cal_en = 0 #Enables the pad driver that sends the rising edge signal off chip. This rising edge signal is used internally to trigger the 'Set' pin of the output RS Flip Flop.#
		self.qutx_fb_180Rsff_en = 0 #off#
		self.qutx_fb_0Rsff_en = 0 #off#
		#-------------------------------------------------------------------------------------------------------
		# QuIET Dm
		#-------------------------------------------------------------------------------------------------------
		self.N = 4 ##
		self.R_11to8 = 13 ##
		self.R_7to0 = 172 ##
		self.Asyncrst_n = 0 #off#
		self.Cp_sel_6to0 = 63 ##
		self.Cp_sel_8to7 = 0 ##
		self.ForceFout = 0 #off#
		self.ForceFoutb = 0 #off#
		self.Out_en = 0 #off#
		self.Dll_en = 1 #on#
		self.Ana_en = 1 #off#
		self.Decod_in_0deg = [] #Read Only#
		#-------------------------------------------------------------------------------------------------------
		# RECEIVE QuIET FREQUENCY GENERATOR
		#-------------------------------------------------------------------------------------------------------
		self.Ngt3_3 = 0 #Output frequency control bit.  Calculated.#
		self.NorNdiv4_3 = 0 #Output frequency control word.  Calculated.#
		self.RorFrNpRdiv4_25to18_3 = 0 #Output frequency control word.  Calculated.#
		self.RorFrNpRdiv4_17to10_3 = 0 ##
		self.RorFrNpRdiv4_9to2_3 = 0 ##
		self.RorFrNpRdiv4_1to0_3 = 0 ##
		self.Qu_tx_Ngt3_3 = 0 #Enables divide-by-4 freq divider - Phase shift control bit.  Calculated.#
		self.NorNdiv4_phsh_3 = 1 #Phase shift control word.  Calculated#
		self.RorFrNpRdiv4_phsh_25to18_3 = 0 #Phase shift control word.  Calculated.#
		self.RorFrNpRdiv4_phsh_17to10_3 = 0 ##
		self.RorFrNpRdiv4_phsh_9to2_3 = 0 ##
		self.RorFrNpRdiv4_phsh_1to0_3 = 0 ##
		self.Passthru_ref_clk_3 = 0 #Enable reference clock pass-through mode#
		self.Byp_ram_3 = 1 #Bypass the SRAMs#
		self.Dis_adr_dith_3 = 1 #Disable the dither generator in the ca2adr block#
		self.Dis_p5G_dith_3 = 1 #Disable the dither generator in the lup2decod block#
		self.Byp_fine_3 = 1 #Bypass fine delay line control bit#
		self.Exclude32_3 = 0 #Bypass fine delay line control bit (exclude 32)#
		self.Dis_risedge_3 = 0 #Disable the rising edges decoders#
		self.Dis_faledge_3 = 0 #Disable the falling edges decoders#
		self.Spr_puls_en_3 = 0 #Enable spur pulsing mode#
		self.Spr_puls_val_a_9to3_3 = 0 #Spur pulsing mode control word#
		self.Spr_pulse_val_2to0_3 = 0 ##
		self.Spr_puls_val_b_9to2_3 = 8 #Spur pulsing mode control word#
		self.Spr_puls_val_b_1to0_3 = 0 ##
		self.Thru_ris_en_3 = 0 #Put rising edges decoders into through-tap mode#
		self.Thru_ris_tap_11to6_3 = 32 #Through-tap mode control word#
		self.Thru_ris_tap_5to0_3 = 0 #Through-tap mode control word#
		self.Thru_fal_en_3 = 0 #Put falling edges decoders into through-tap mode#
		self.Thru_fal_tap_11to6_3 = 0 #Through-tap mode control word#
		self.Thru_fal_tap_5to0_3 = 0 #Through-tap mode control word#
		self.Dig_delay_3 = 0 #This bit provides delay to the clock going into the digital block. It is a remnant of past designs and should always be left off because the digClkPhase setting in address 23 provides much finer control.#
		self.Clk_driver_en_3 = 0 #This allows the clock to reach the digital block. It first passes through the digital/analog clock synchronization mux, which means that dlEn must be on (dlEn=1) and Clk_driver=1 for the digital block to receive a clock.  See Byp_fine, address 10, bit 6#
		self.qu_reg_en_3 = 0 #This bit enables the voltage regulators that supply 1.2 V to all the analog block functions. There are 6 separate regulators that are simultaneously enabled by this bit.#
		self.qq_reg_en_3 = 0 #This bit enables the voltage regulators that supply 1.2 V to all the Quad Gen functions. There are 3 separate regulators that are simultaneously enabled by this bit.#
		self.win_rst_3 = 0 #When this bit is high, the windowing function is in a reset state, which means that no taps will be passed to the DDS output regardless of the tap select signals coming from the digital block.#
		self.fineEn_3 = 0 #This bit, when high, routes the coarse taps through the fine line before reaching the output RS Flip Flop of the DDS. When low, the coarse tap is routed directly to the output RS Flip Flop.#
		self.fineEnb_3 = 0 #Opposite of  fineEn.#
		self.rsffEn_3 = 0 #This bit must be high to send the QuIET 0 and 180 degree calibration signals off chip. It does not control the RS Flip Flop outputs of the DDS, though it may have some second order (coupling) effect.#
		self.dl_en_3 = 1 #Allows the PLL reference to enter the QuIET delay line when enabled.#
		self.cp_en_3 = 1 #This bit, when enables, activates the charge pump that controls the delay line via the single pole (one capacitor) DLL loop filter.#
		self.forceCpUpb_3 = 0 #This bit only matters when pdEn=0 (address 22, bit 1). When low, the pmos device connected to the DLL loop filter cap turns on and sources current into the cap, thereby increasing the delay line control voltage. #
		self.forceCpDn_3 = 0 #This bit only matters when pdEn=0 (address 22, bit 1). When low, the nmos device connected to the DLL loop filter cap turns off and allows the pmos device to charge up the loop cap as described above.#
		self.pdUpTune_1to0_3 = 3 #These bits control the pulse width from the phase detector into the charge up port of the charge pump. 00 turns the charge up signal off. 01 is the minimum pulse width setting and 11 is the maximum pulse width setting.#
		self.pdDnTune_1to0_3 = 1 #These bits control the pulse width from the phase detector into the charge down port of the charge pump. 00 turns the charge down signal off. 01 is the minimum pulse width setting and 11 is the maximum pulse width setting.#
		self.cpUpTune_2to0_3 = 7 #These bits control amount of current that is sourced while the charge up signal from the phase detector is high. 000 is minimum current and 111 is maximum current.#
		self.cpDnTune_2to0_3 = 2 #These bits control amount of current that is sinked while the charge down signal from the phase detector is high. 000 is minimum current and 111 is maximum current.#
		self.pdEn_3 = 1 #When enables, the phase detector will send charge up and down signals to the charge pump and over ride the forceCpUp and forceCpDn settings in address 21. When disabled, the forceCpUp and forceCpDn settings will control the charge pump.#
		self.digClkPhase_7to0_3 = 4 #Only one bit in this field should be active at one time. This signal drives a mux that selects one of eight clock phases from the delay line to drive the digital block. This is needed to control the windowing function of the DDS.#
		self.Rst_n_async_3 = 0 #Digital reset.#
		self.L1_lup00_15to8_3 = [] #Read-only#
		self.L1_lup90_15to8_3 = [] #Read-onnly#
		self.Merg_ris_fin_3 = [] #Read-only#
		self.Merg_fal_fin_3 = [] #Read-only#
		self.Qg00degDelay_0to4_3 = 31 #Adjusts series delay in the 0 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.#
		self.Qg90degDelay_0to4_3 = 31 #Adjusts series delay in the 90 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.#
		self.Qg180degDelay_0to4_3 = 31 #Adjusts series delay in the 180 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.#
		self.Qg270degDelay_0to4_3 = 31 #Adjusts series delay in the 270 degree path for the divide-by-two, x1, x2, and x4 quadrature generators.#
		self.DischargeTap16_0to3_3 = 15 #Adjusts DLL offset error in the Quad Gen delay line by controlling down currents in one of the parallel charge pumps.#
		self.ChargeTap16_0to3_3 = 15 #Adjusts DLL offset error in the Quad Gen delay line by controlling up currents in one of the parallel charge pumps.#
		self.DischargeTapn_0to3_3 = 15 #Adjusts DLL offset error in the Quad Gen delay line by controlling down currents in one of the parallel charge pumps.#
		self.ChargeTapn16_0to3_3 = 15 #Adjusts DLL offset error in the Quad Gen delay line by controlling up currents in one of the parallel charge pumps.#
		self.X1sel_32to39_3 = 0 #Control for the divide-by-two and x1 functions.#
		self.X1sel_40to47_3 = 0 #Control for the divide-by-two and x1 functions.#
		self.X2sel_32to36_3 = 0 #Control for the x2 function.#
		self.X2sel_37to41_3 = 0 #Control for the x2 function.#
		self.X4sel_32to36_3 = 0 #Control for the x4 function.#
		self.X4sel_37to41_3 = 0 #Control for the x4 function.#
		self.X8sel_32to36_3 = 0 #Bit 41 is used for a fine line windowing control bit. If the fine line is needed, this bit needs to be set high if Fout is close to Fref (greater than ~ 950 MHz) or for some testing modes like pass through or thru_rise_en.#
		self.X8sel_41_3 = 0 #hiFout - set for passthrough and Fout close to Fref#
		self.X8sel_37to40_3 = 0 ##
		self.qurx_180Cal_en = 0 #Enables the pad driver that sends the falling edge signal off chip. This falling edge signal is used internally to trigger the 'Reset' pin of the output RS Flip Flop.#
		self.qurx_0Cal_en = 0 #Enables the pad driver that sends the rising edge signal off chip. This rising edge signal is used internally to trigger the 'Set' pin of the output RS Flip Flop.#
		#-------------------------------------------------------------------------------------------------------
		# PLL
		#-------------------------------------------------------------------------------------------------------
		self.extClkEn = 0 #PLL Reg 0#
		self.extClkEnBNOTD7 = 1 #on#
		self.div2_rst = 1 #on#
		self.TxChClkSel = 0 ##
		self.TxChClkEn = 0 #PLL Reg 1#
		#-------------------------------------------------------------------------------------------------------
		# TRANSMITTER
		#-------------------------------------------------------------------------------------------------------
		self.tx_bb_en = 0 #BB Fdbk Mux Buffer BW Control.  Enables the Forward BB Reference Section of TX#
		self.tx_bb_fdbk_bw = 0 #Sets the BW of the BB Correction feedback amp#
		self.tx_bb_fdbk_cal_en = 0 #BB Feedback Mux path Routing.  Shorts the BB Correction feedback Amp input for self-calibration#
		self.tx_bb_fdbk_cart_err_en = 0 #Routes the Cartesian error signal through the BB Correction feedback#
		self.tx_bb_fdbk_cart_fb_en = 0 #Routes the Cartesian feedback signal through the BB Correction feedback#
		self.tx_bb_fdbk_cart_fwd_en = 0 #Routes the Cartesian reference signal through the BB Correction feedback#
		self.tx_bb_fdbk_en = 0 #BB Feedback Mux path Routing.  Enables the BB Correction feedback path via the RX pins#
		self.tx_bb_fdbk_1q_sel = 0 #Chooses between I or Q channel for the BB Correction feedback path#
		self.tx_bb_fdbk_lp = 0 #BB Fdbk Mux Buffer current.  Sets the current drive capability for BB Correction feedback Amp#
		self.tx_bb_fdbk_statt = 3 #BB Fdbk Mux Buffer Gain Control.  BB Feedback Attenuator.  Sets the voltage gain for BB Correction feedback Amp#
		self.tx_bb_fdbk_swapi = 0 #Baseband Feedback Swap I & Ix.  Swaps the I and Ix BB signals through the BB Correction feedback path#
		self.tx_bb_fdbk_swapq = 0 #Baseband feedback Swap Q & Qx.  Swaps the Q and Qx BB signal through the BB Correction feedback path#
		self.tx_bb_gain_cmp = 1 #Baseband Gain 1 dB Compensation.  Adds and extra 1.3 dB of Forward Baseband Reference Gain#
		self.tx_bb_lp = 0 #BB ref. stage current.  BB Amp Stage Current.  Sets the current drive capability for Forward BB Reference Amps#
		self.tx_bb_swapi = 1 #Baseband Swap I & Ix.  Swaps the I and Ix BB signals through the Forward BB Reference Path#
		self.tx_bb_swapq = 0 #Baseband Swap Q & Qx.  Swaps the Q and Qx BB signals through the Forward BB Reference Path#
		self.tx_butt_bw = 0 #BB ref. Butterworth filter BW control.  Sets the BW of the Forward BB Reference 4-pole Butterworth Filters#
		self.tx_bw_trck = 5 #TX MIM cap tracking filter BW.  Bandwidth Tracking.  Sets tracking BW of all the MIM cap based TX Filters (16 states)#
		self.tx_cart_en = 0 #Cartesian FB path Enable.  Enables the Cartesian Baseband Section of Tx#
		self.tx_cart_fb_bb_statt = 15 #Cartesian down-mix path BB gain.  Cartesian FB path BB gain.  Sets the voltage gain for Cartesian BB down converter PMA#
		self.tx_cart_fb_dcoc_dac_I1 = 32 #Sets Cartesian BB down converter PMA Dc offset correction DAC I1#
		self.tx_cart_fb_dcoc_dac_I2 = 32 #Sets Cartesian BB down converter PMA Dc offset correction DAC I2#
		self.tx_cart_fb_dcoc_dac_Q1 = 32 #Sets Cartesian BB down converter PMA Dc offset correction DAC Q1#
		self.tx_cart_fb_dcoc_dac_Q2 = 32 #Sets Cartesian BB down converter PMA Dc offset correction DAC Q2#
		self.CartesianFeedbackpathDCOCenable = 0 #Cartesian down-mix path BB BW#
		self.CartesianFeedbackpathenable = 0 #off#
		self.CartesianFBpathHiResolutionDCOCenable = 0 #off#
		self.CartesianFBpathBW = 15 ##
		self.CartesianFBRFGain = 0 #Cartesian down conv. path RF Gain#
		self.CartesianFBpathSwapIandIx = 0 #Swap I & Ix BB in Down Converter#
		self.CartesianFBpathSwapQandQx = 0 #off#
		self.CartesianFBpathSwitchtoforwardSummer = 0 #off#
		self.tx_cart_fb_lo_select = 0 #Cart. down conv LO curr. (tied to Gain)#
		self.CartesianFBpathAmp1Gain = 3 ##
		self.CartesianFBpathAmp2Gain = 3 ##
		self.CartesianFBpathAmp3Gain = 3 ##
		self.CartesianFBpathAmp4Gain = 3 ##
		self.CartesianFBpathAmpCurrentSelect = 7 ##
		self.CartesianFBpathZeroEnable = 0 #off#
		self.tx_cart_zero_statt = 1 #Cartesian FB path Zero Gain.  Sets the voltage gain for Cartesian Forward BB Zero Amp#
		self.tx_inbuf_bw = 0 #Sets the BW of the Forward BB Reference Input Buffers#
		self.tx_inbuf_statt = 0 #Sets the attenuation of the Forward BB Ref. Buffers#
		self.tx_output_channel_sel = 0 #Selects from the 3 RF Forward TX output paths, 000 is full power down#
		self.tx_p1_bw = 0 #Sets the BW of the Cartesian Forward BB Loop Pole 1#
		self.tx_pw_bw1 = 0 #Cartesian FB path Pole 2 Bandwidth.  Sets the BW of the Cartesian Forward BB Loop Pole 2#
		self.tx_p2_bw2 = 0 #Cartesian FB path Pole 2 Bandwidth.  Sets the BW of the Cartesian Forward BB Loop Pole 2#
		self.PushPullBufferCurrent = 7 ##
		self.tx_rf_aoc_bw = 0 #Sets the BW of the AOC control line#
		self.RFForwardPathEnable_toMUX = 0 #off#
		self.RFForwardPathEnable_ExternalPinenable = 1 #on#
		self.tx_rf_fwd_lp = 0 #RF Forward Bias Reference Control.  RF Forward Path Current Drain Select.  Sets the current drive capability for Forward RF Output Drivers#
		self.tx_rf_fwd_statt1 = 0 #RF Passive Step Attenuator control.  RF Forward Path Step Attn1.  Sets the attenuation level for the RF Step attenuators#
		self.tx_rf_fwd_statt2 = 0 #RF Output Driver Step Attn. Control.  RF Forward Path Step Attn2.  Sets the attenuation level for the RF Output Drivers#
		self.BBQDivideby2or4Select = 0 #BBQ Quad Gen Divide by 2 or 4 (High=1/4)#
		self.BBQQuadGenEnable = 0 #Bypass Quiet LO with external LO#
		self.BBQPolyphaseQuadGenEnable = 0 #off#
		self.lofb_tun_s = 8 ##
		self.lofb_tun_sx = 8 ##
		self.lofw_tun_s2 = 8 ##
		self.lofw_tun_sx2 = 8 ##
		self.reserve_tx26 = 0 ##
		self.reserve_tx27 = 0 ##
		#-------------------------------------------------------------------------------------------------------
		# RECEIVER
		#-------------------------------------------------------------------------------------------------------
		self.rx_Idac = 16 #I path DCOC DAC setting.  Digital values for the DC offset adjustment.  11111 represents the maximum positive offset adjust and 00000 represents the maximum negative offset adjust.  By design, codes 10000 and 01111 cause no change in the offset voltage.#
		self.rx_dcs = 0 #DCOC step size select.  Selects the proper current reference in the DAC to maintain constant step size at ouptut of baseband filters.  This value works in tandem with the BiQuad Gain Select (address 198, bits 4:3) to maintain a constant step size of 24 mV at filter output.#
		self.rx_den = 0 #Enables the DC offset correction DACs in the I and Q path.#
		self.rx_Qdac = 12 #Q path DCOC DAC setting.  Digital values for the DC offset adjustment.  11111 represents the maximum positive offset adjust and 00000 represents the maximum negative offset adjust.  By design, codes 10000 and 01111 cause no change in the offset voltage.#
		self.rx_cmpen = 0 #Enables the DC offset correction comparator used in the DCOC circuitry.#
		self.rx_dcoc = 0 #Enables the DC offset correction circuitry for automatic correction of the DC offset in the baseband filters.#
		self.rx_ten = 0 #Enables the RC tuning circuit to tune RX and TX baseband filters.#
		self.rx_ren = 0 #Enables the ramp circuit used in the RC tuning circuitry to tune the RX and TX baseband filters.#
		self.rx_dven = 0 ##
		self.rx_dv = 0 #DCOC/tune clock divider select.  Selects the clock rate used for clocking the DCOC and RC tuning circuit.  Bits 3 and 2 set the divider setting used for both the DCOC circuitry and RC Tune circuitry.  Bits 1 and 0 set the divider setting for the dedicated divider in the DCOC circuitry.  Table below shows the mapping of divider settings.  The DCOC clock divider setting is the total divide ratio of both dividers.  The maximum divide ratio is 8*8 = 64.#
		self.rx_extc = 0 #Enables the external capacitor pins to allow for external low-frequency pole to be placed in the signal path between the mixer and baseband filter.#
		self.rx_cen = 0 #Chopper enable for filter stages.  Settings to enable which amplifier the clock is being applied#
		self.rx_chck = 0 #Divider setting for the chopper clock#
		self.rx_chcken = 0 #Enables the baseband filter chopper clock.#
		self.rx_fen = 0 #Enables baseband filters.  0 puts filter in power save mode.#
		self.rx_onchen = 0 #Enables on-channel detector.#
		self.rx_offchen = 0 #Enables off-channel detector#
		self.rx_foe = 0 #Enables the output of the baseband filters.  Otherwise the baseband filter outputs are in a Hi-Z state to allow transmitter to use filter output pins.  When Filter Enable is set LOW, outputs are disabled (Hi-Z)#
		self.rx_offch = 1 #Sets the Clip Threshold for the Off-channel Detector#
		self.rx_onchf = 0 #Sets the Fade Threshold for the On-channel Detector relative to the On-channel clip point.#
		self.rx_onchc = 2 #Sets the Clip Threshold for the On-channel Detector#
		self.rx_qs = 0 #Sets the BiQuad filter Q#
		self.rx_bqg = 0 #Set BiQuad filter gain#
		#FIXME Maybe set rx_rq to 0
		self.rx_rq = 1 #Sets the BiQuad filter resistor value.  The natural frequency of the BiQuad (wo) is this resistor value multiplied by the BiQuad Capacitor value.#
		self.rx_rv = 1 #Sets the VGA filter (passive filter) resistor value.  The pole frequency of the passive filter is this resistor value multiplied by the VGA capacitor value.#
		self.rx_rip = 0 #Sets the MPA input resistor value that sets the gain of the PMA.  Gain of the PMA is Rf/Rin where Rf is the PMA feedback resistor and Rin is the input resistor.  Note that the input resistance remains at 2 kohm differential for all settings.  An R2R ladder is used to accomplish this while changing the Rin value.#
		self.rx_rfp = 2 #Sets the PMA feedback resistor value that sets the gain of the PMA as well as the pole frequency (along with PMA capacitor value).  Gain of the PMA is Rf/Rin where Rf is the PMA feedback resistor and Rin is the input resistor.#
		self.rx_cp_12to8 = 0 #Sets the PMA filter capacitor value.  The pole frequency of the PMA filter is the PMA feedback resistor value multiplied by this Capacitor value.  PMA Capacitor (in pF) = (PMAC) * 0.0625 + 1#
		self.rx_gs = 0 #Sets the gain of the VGA in the baseband filter#
		self.rx_cp_7to0 = 0 #PMA cap select LSBs.  Sets the PMA filter capacitor value.  The pole frequency of the PMA filter is the PMA feedback resistor value multiplied by this Capacitor value.  PMA Capacitor (in pF) = (PMAC) * 0.0625 + 1#
		self.rx_cv_10to3 = 0 #VGA cap select MSBs.  Sets the VGA (passive) filter capacitor value.  This pole frequency of the passive filter is the VGA resistor value multiplied by this Capacitor value.  VGA Capacitor (in pF) = (VGAC) * 0.0625 + 1#
		self.rx_cv_2to0 = 0 #VGA cap select LSBs.  Sets the VGA (passive) filter capacitor value.  This pole frequency of the passive filter is the VGA resistor value multiplied by this Capacitor value.  VGA Capacitor (in pF) = (VGAC) * 0.0625 + 1#
		self.rx_cc_2to0 = 0 #Compensation control.  Disables additional compensation capacitance in the VGA and BiQuad op-amps to allow for higher bandwidths.  Also increases the op-ampdominate pole-frequency which improves filter response.  Bit 4 controls the VGA amplifier, Bit 3 controls the feedback amplifier in the BiQuad, and Bit 2 controls the output buffer in the BiQuad.#
		self.rx_cq_9to8 = 0 #BiQuad cap select MSBs.  Sets the BiQuad filter capacitor value.  The natural frequency of the BiQuad (wo) is the BiQuad resistor value multiplied by this Capacitor value.  BiQuad Capacitor (in pF) = (BiQuadC) * 0.125 + 2#
		self.rx_cq_7to0 = 0 #BiQuad cap select LSBs.  Sets the BiQuad filter capacitor value.  The natural frequency of the BiQuad (wo) is the BiQuad resistor value multiplied by this Capacitor value.  BiQuad Capacitor (in pF) = (BiQuadC) * 0.125 + 2#
		self.rx_lna = 1 #LNA select#
		self.rx_lnab = 0 #LNA bias select#
		self.rx_rxchen = 0 #RX mixer enable.  Must be set to 1 to enable Mixer operation#
		self.rx_bbq_div2or4 = 0 #Selects divide ratio of RX Quad Gen when using external LO.  0->DIV2, 1 ->DIV1#
		self.rx_Loselect = 0 #RX external LO select.  Enables external LO clock source#
		self.poly_en = 0 #off#
		self.lorx_tun_s = 8 ##
		self.lorx_tun_sx = 8 ##
		self.rx_Icmpo = [] #I path DCOC comparator output.  Output of the DCOC comparator - used for test purposes.  Output only.#
		self.rx_Iodac = [] #I path DCOC DAC output.  Output of the DCOC DACs - used to read result of DCOC correction circuitry.  Output only.#
		self.rx_Qcmpo = [] #Q path DCOC comparator output.  Output of the DCOC comparator - used for test purposes.  Output only.#
		self.rx_Qodac = [] #Q path DCOC DAC output.  Output of the DCOC DACs - used to read result of DCOC correction circuitry.  Output only.#
		self.rx_rc = [] #Output word from RC Tune circuit that is used to calculate adjustment needed to TX and RX filter bandwidths for correct tuning.  Output only.#
		#-------------------------------------------------------------------------------------------------------
		# VAG Generator
		#-------------------------------------------------------------------------------------------------------
		self.shft_cml_in = 0 #Enable - 150mV level shift of Ref. BB VAG#
		self.vagenable1 = 1 #Enable VAG Gen into Sleep Mode (slow ramp up)#
		self.vagenable2 = 1 #Enable VAG Gen in Full On Mode (Fast ramp from sleep)#
		#-------------------------------------------------------------------------------------------------------
		# TEST MULTIPLEXER
		#-------------------------------------------------------------------------------------------------------
		self.TestMuxBufferEnable = 0 #Enable Test Mux Buffer#
		self.TestMuxEnable = 0 #Enable Test Mux#
		self.TestMuxSetting = 0 #Four Output Description (Test1, Test2, Test3, Test4)#

		self.txgain = 0	#Set Transmit Gain#

		self.Fclk = 1000e6 #Default clock frequency, in Hz#

		self.Fouttx = 0	# Default tx frequency is zero#
		self.Foutrx = 0	# Default rx frequency is zero#
		self.Foutfb = 0 # Default feedback frequency is zero#

		# Initialize GPIO and ATR
		# GPIO are the general-purpose IO pins on the daughterboard
		# IO_RX_06 must be used for ATR (1 = TX, 0 = RX)
		# ATR is the automatic transmit/receive switching, done in the FPGA
		# FIXME
		self.rx_write_io(0, TX_EN)
		self.rx_write_oe(TX_EN, TX_EN)
		self.rx_set_atr_rxval(0)
		self.rx_set_atr_txval(TX_EN)
		self.rx_set_atr_mask(TX_EN)

		# Initialize Chipset
		# Set initial SPI values
		# Neither transmit nor receive currently on

		'''self.set_reg_0()
		self.set_reg_1()
		self.set_reg_2()
		self.set_reg_3()
		self.set_reg_4()
		self.set_reg_5()
		self.set_reg_6()
		self.set_reg_7()
		self.set_reg_8()
		self.set_reg_9()
		self.set_reg_10()
		self.set_reg_12()
		self.set_reg_13()
		self.set_reg_14()
		self.set_reg_15()
		self.set_reg_16()
		self.set_reg_17()
		self.set_reg_18()
		self.set_reg_19()
		self.set_reg_20()
		self.set_reg_21()
		self.set_reg_22()
		self.set_reg_23()
		self.set_reg_24()
		self.set_reg_29()
		self.set_reg_30()
		self.set_reg_31()
		self.set_reg_32()
		self.set_reg_33()
		self.set_reg_34()
		self.set_reg_35()
		self.set_reg_36()
		self.set_reg_37()
		self.set_reg_38()
		self.set_reg_39()
		self.set_reg_40()
		self.set_reg_41()
		self.set_reg_42()
		self.set_reg_43()
		self.set_reg_48()
		self.set_reg_49()
		self.set_reg_50()
		self.set_reg_51()
		self.set_reg_52()
		self.set_reg_53()
		self.set_reg_54()
		self.set_reg_55()
		self.set_reg_56()
		self.set_reg_57()
		self.set_reg_58()
		self.set_reg_60()
		self.set_reg_61()
		self.set_reg_62()
		self.set_reg_63()
		self.set_reg_64()
		self.set_reg_65()
		self.set_reg_66()
		self.set_reg_67()
		self.set_reg_68()
		self.set_reg_69()
		self.set_reg_70()
		self.set_reg_71()
		self.set_reg_72()
		self.set_reg_77()
		self.set_reg_78()
		self.set_reg_79()
		self.set_reg_80()
		self.set_reg_81()
		self.set_reg_82()
		self.set_reg_83()
		self.set_reg_84()
		self.set_reg_85()
		self.set_reg_86()
		self.set_reg_87()
		self.set_reg_88()
		self.set_reg_89()
		self.set_reg_90()
		self.set_reg_91()
		self.set_reg_96()
		self.set_reg_97()
		self.set_reg_98()
		self.set_reg_99()
		self.set_reg_104()
		self.set_reg_105()
		self.set_reg_106()
		self.set_reg_107()
		self.set_reg_108()
		self.set_reg_109()
		self.set_reg_110()
		self.set_reg_111()
		self.set_reg_112()
		self.set_reg_113()
		self.set_reg_114()
		self.set_reg_116()
		self.set_reg_117()
		self.set_reg_118()
		self.set_reg_119()
		self.set_reg_120()
		self.set_reg_121()
		self.set_reg_122()
		self.set_reg_123()
		self.set_reg_124()
		self.set_reg_125()
		self.set_reg_126()
		self.set_reg_127()
		self.set_reg_128()
		self.set_reg_133()
		self.set_reg_134()
		self.set_reg_135()
		self.set_reg_136()
		self.set_reg_137()
		self.set_reg_138()
		self.set_reg_139()
		self.set_reg_140()
		self.set_reg_141()
		self.set_reg_142()
		self.set_reg_143()
		self.set_reg_144()
		self.set_reg_145()
		self.set_reg_146()
		self.set_reg_147()
		self.set_reg_152()
		self.set_reg_153()
		self.set_reg_156()
		self.set_reg_157()
		self.set_reg_158()
		self.set_reg_159()
		self.set_reg_160()
		self.set_reg_161()
		self.set_reg_162()
		self.set_reg_163()
		self.set_reg_164()
		self.set_reg_165()
		self.set_reg_166()
		self.set_reg_167()
		self.set_reg_168()
		self.set_reg_169()
		self.set_reg_170()
		self.set_reg_171()
		self.set_reg_172()
		self.set_reg_173()
		self.set_reg_174()
		self.set_reg_175()
		self.set_reg_176()
		self.set_reg_177()
		self.set_reg_178()
		self.set_reg_179()
		self.set_reg_180()
		self.set_reg_181()
		self.set_reg_192()
		self.set_reg_193()
		self.set_reg_194()
		self.set_reg_195()
		self.set_reg_196()
		self.set_reg_197()
		self.set_reg_198()
		self.set_reg_199()
		self.set_reg_200()
		self.set_reg_201()
		self.set_reg_202()
		self.set_reg_203()
		self.set_reg_204()
		self.set_reg_205()
		self.set_reg_206()
		self.set_reg_207()
		self.set_reg_220()
		self.set_reg_222()'''

		self.set_reg_96()
		self.set_reg_97()
		self.set_reg_98()
		self.set_reg_99()

		self.set_reg_152()
		self.set_reg_153()

		self.set_reg_220()
		self.set_reg_222()

	def __del__(self):
		# Delete instance, shut down
		# FIXME
		print "RFIC: __del__"

		# Reset all three QuIET synthesizers
		self.Rst_n_async = 0
		self.set_reg_24()
		self.Rst_n_async2 = 0
		self.set_reg_72()
		self.Rst_n_async3 = 0
		self.set_reg_128()

		self.X1sel_32to39_3 = 0
		self.X1sel_40to47_3 = 0
		self.X2sel_32to36_3 = 0
		self.X2sel_37to41_3 = 0
		self.X4sel_32to36_3 = 0
		self.X4sel_37to41_3 = 0

		self.set_reg_139()
		self.set_reg_140()
		self.set_reg_141()
		self.set_reg_142()
		self.set_reg_143()
		self.set_reg_144()

		self.X1sel_32to39 = 0
		self.X1sel_40to47 = 0
		self.X2sel_32to36 = 0
		self.X2sel_37to41 = 0
		self.X4sel_32to36 = 0
		self.X4sel_37to41 = 0

		self.set_reg_35()
		self.set_reg_36()
		self.set_reg_37()
		self.set_reg_38()
		self.set_reg_39()
		self.set_reg_40()

		self.X1sel_32to39_2 = 0
		self.X1sel_40to47_2 = 0
		self.X2sel_32to36_2 = 0
		self.X2sel_37to41_2 = 0
		self.X4sel_32to36_2 = 0
		self.X4sel_37to41_2 = 0

		self.set_reg_83()
		self.set_reg_84()
		self.set_reg_85()
		self.set_reg_86()
		self.set_reg_87()
		self.set_reg_88()

	# --------------------------------------------------------------------
	# These methods set the RFIC onboard registers over the SPI bus.
	# Thus, the shift values here are the 0-7 values from the data sheet

	# For more information about setting each variable and SPI register, see RFIC4 SPI Default Variables.xls 

	def set_reg_0(self):	   
		reg_0 = (   
		self.Ngt3 << 7 |
		self.NorNdiv4 << 0 )
		self.send_reg(0, reg_0)   
	def set_reg_1(self):	   
		reg_1 = (   
		self.RorFrNpRdiv4_25to18 << 0 )
		self.send_reg(1, reg_1)   
	def set_reg_2(self):	   
		reg_2 = (   
		self.RorFrNpRdiv4_17to10 << 0 )
		self.send_reg(2, reg_2)   
	def set_reg_3(self):	   
		reg_3 = (   
		self.RorFrNpRdiv4_9to2 << 0 )
		self.send_reg(3, reg_3)   
	def set_reg_4(self):	   
		reg_4 = (   
		self.RorFrNpRdiv4_1to0 << 6 )
		self.send_reg(4, reg_4)   
	def set_reg_5(self):	   
		reg_5 = (   
		self.Qu_tx_Ngt3 << 7 |
		self.NorNdiv4_phsh << 0 )
		self.send_reg(5, reg_5)   
	def set_reg_6(self):	   
		reg_6 = (   
		self.RorFrNpRdiv4_phsh_25to18 << 0 )
		self.send_reg(6, reg_6)   
	def set_reg_7(self):	   
		reg_7 = (   
		self.RorFrNpRdiv4_phsh_17to10 << 0 )
		self.send_reg(7, reg_7)   
	def set_reg_8(self):	   
		reg_8 = (   
		self.RorFrNpRdiv4_phsh_9to2 << 0 )
		self.send_reg(8, reg_8)   
	def set_reg_9(self):	   
		reg_9 = (   
		self.RorFrNpRdiv4_phsh_1to0 << 6 )
		self.send_reg(9, reg_9)   
	def set_reg_10(self):	   
		reg_10 = (   
		self.Passthru_ref_clk << 7 |
		self.Byp_ram << 6 |
		self.Dis_adr_dith << 5 |
		self.Dis_p5G_dith << 4 |
		self.Byp_fine << 3 |
		self.Exclude32 << 2 |
		self.Dis_risedge << 1 |
		self.Dis_faledge << 0 )
		self.send_reg(10, reg_10)   
	def set_reg_12(self):	   
		reg_12 = (   
		self.Spr_puls_en << 7 |
		self.Spr_puls_val_a_9to3 << 0 )
		self.send_reg(12, reg_12)   
	def set_reg_13(self):	   
		reg_13 = (   
		self.Spr_pulse_val_2to0 << 5 )
		self.send_reg(13, reg_13)   
	def set_reg_14(self):	   
		reg_14 = (   
		self.Spr_puls_val_b_9to2 << 0 )
		self.send_reg(14, reg_14)   
	def set_reg_15(self):	   
		reg_15 = (   
		self.Spr_puls_val_b_1to0 << 6 )
		self.send_reg(15, reg_15)   
	def set_reg_16(self):	   
		reg_16 = (   
		self.Thru_ris_en << 7 |
		self.Thru_ris_tap_11to6 << 1 )
		self.send_reg(16, reg_16)   
	def set_reg_17(self):	   
		reg_17 = (   
		self.Thru_ris_tap_5to0 << 2 )
		self.send_reg(17, reg_17)   
	def set_reg_18(self):	   
		reg_18 = (   
		self.Thru_fal_en << 7 |
		self.Thru_fal_tap_11to6 << 1 )
		self.send_reg(18, reg_18)   
	def set_reg_19(self):	   
		reg_19 = (   
		self.Thru_fal_tap_5to0 << 2 )
		self.send_reg(19, reg_19)   
	def set_reg_20(self):	   
		reg_20 = (   
		self.Dig_delay << 7 |
		self.Clk_driver_en << 6 |
		self.qu_reg_en << 5 |
		self.qq_reg_en << 4 |
		self.win_rst << 3 |
		self.fineEn << 2 |
		self.fineEnb << 1 |
		self.rsffEn << 0 )
		self.send_reg(20, reg_20)   
	def set_reg_21(self):	   
		reg_21 = (   
		self.dl_en << 7 |
		self.cp_en << 6 |
		self.forceCpUpb << 5 |
		self.forceCpDn << 4 |
		self.pdUpTune_1to0 << 2 |
		self.pdDnTune_1to0 << 0 )
		self.send_reg(21, reg_21)   
	def set_reg_22(self):	   
		reg_22 = (   
		self.cpUpTune_2to0 << 5 |
		self.cpDnTune_2to0 << 2 |
		self.pdEn << 1 )
		self.send_reg(22, reg_22)   
	def set_reg_23(self):	   
		reg_23 = (   
		self.digClkPhase_7to0 << 0 )
		self.send_reg(23, reg_23)   
	def set_reg_24(self):	   
		reg_24 = (   
		self.Rst_n_async << 7 )
		self.send_reg(24, reg_24)   
	def read_reg_25(self):	   
		reg_25 = self.get_reg(25)   
		self.L1_lup00_15to8 = reg_25   
		   
	def read_reg_26(self):	   
		reg_26 = self.get_reg(26)   
		self.L1_lup90_15to8 = reg_26   
		   
	def read_reg_27(self):	   
		reg_27 = self.get_reg(27)   
		self.Merg_ris_fin = reg_27 >> 2   
		   
	def read_reg_28(self):	   
		reg_28 = self.get_reg(28)   
		self.Merg_fal_fin = reg_28 >> 2   
		   
	def set_reg_29(self):	   
		reg_29 = (   
		self.Qg00degDelay_0to4 << 3 )
		self.send_reg(29, reg_29)   
	def set_reg_30(self):	   
		reg_30 = (   
		self.Qg90degDelay_0to4 << 3 )
		self.send_reg(30, reg_30)   
	def set_reg_31(self):	   
		reg_31 = (   
		self.Qg180degDelay_0to4 << 3 )
		self.send_reg(31, reg_31)   
	def set_reg_32(self):	   
		reg_32 = (   
		self.Qg270degDelay_0to4 << 3 )
		self.send_reg(32, reg_32)   
	def set_reg_33(self):	   
		reg_33 = (   
		self.DischargeTap16_0to3 << 4 |
		self.ChargeTap16_0to3 << 0 )
		self.send_reg(33, reg_33)   
	def set_reg_34(self):	   
		reg_34 = (   
		self.DischargeTapn_0to3 << 4 |
		self.ChargeTapn16_0to3 << 0 )
		self.send_reg(34, reg_34)   
	def set_reg_35(self):	   
		reg_35 = (   
		self.X1sel_32to39 << 0 )
		self.send_reg(35, reg_35)   
	def set_reg_36(self):	   
		reg_36 = (   
		self.X1sel_40to47 << 0 )
		self.send_reg(36, reg_36)   
	def set_reg_37(self):	   
		reg_37 = (   
		self.X2sel_32to36 << 3 )
		self.send_reg(37, reg_37)   
	def set_reg_38(self):	   
		reg_38 = (   
		self.X2sel_37to41 << 3 )
		self.send_reg(38, reg_38)   
	def set_reg_39(self):	   
		reg_39 = (   
		self.X4sel_32to36 << 3 )
		self.send_reg(39, reg_39)   
	def set_reg_40(self):	   
		reg_40 = (   
		self.X4sel_37to41 << 3 )
		self.send_reg(40, reg_40)   
	def set_reg_41(self):	   
		reg_41 = (   
		self.X8sel_32to36 << 3 )
		self.send_reg(41, reg_41)   
	def set_reg_42(self):	   
		reg_42 = (   
		self.X8sel_41 << 7 |
		self.X8sel_37to40 << 3 )
		self.send_reg(42, reg_42)   
	def set_reg_43(self):	   
		reg_43 = (   
		self.qutx_fwd_180Cal_en << 7 |
		self.qutx_fwd_0Cal_en << 6 )
		self.send_reg(43, reg_43)   
	def set_reg_48(self):	   
		reg_48 = (   
		self.Ngt3_2 << 7 |
		self.NorNdiv4_2 << 0 )
		self.send_reg(48, reg_48)   
	def set_reg_49(self):	   
		reg_49 = (   
		self.RorFrNpRdiv4_25to18_2 << 0 )
		self.send_reg(49, reg_49)   
	def set_reg_50(self):	   
		reg_50 = (   
		self.RorFrNpRdiv4_17to10_2 << 0 )
		self.send_reg(50, reg_50)   
	def set_reg_51(self):	   
		reg_51 = (   
		self.RorFrNpRdiv4_9to2_2 << 0 )
		self.send_reg(51, reg_51)   
	def set_reg_52(self):	   
		reg_52 = (   
		self.RorFrNpRdiv4_1to0_2 << 6 )
		self.send_reg(52, reg_52)   
	def set_reg_53(self):	   
		reg_53 = (   
		self.Qu_tx_Ngt3_2 << 7 |
		self.NorNdiv4_phsh_2 << 0 )
		self.send_reg(52, reg_53)   
	def set_reg_54(self):	   
		reg_54 = (   
		self.RorFrNpRdiv4_phsh_25to18_2 << 0 )
		self.send_reg(54, reg_54)   
	def set_reg_55(self):	   
		reg_55 = (   
		self.RorFrNpRdiv4_phsh_17to10_2 << 0 )
		self.send_reg(55, reg_55)   
	def set_reg_56(self):	   
		reg_56 = (   
		self.RorFrNpRdiv4_phsh_9to2_2 << 0 )
		self.send_reg(56, reg_56)   
	def set_reg_57(self):	   
		reg_57 = (   
		self.RorFrNpRdiv4_phsh_1to0_2 << 6 )
		self.send_reg(57, reg_57)   
	def set_reg_58(self):	   
		reg_58 = (   
		self.Passthru_ref_clk_2 << 7 |
		self.Byp_ram_2 << 6 |
		self.Dis_adr_dith_2 << 5 |
		self.Dis_p5G_dith_2 << 4 |
		self.Byp_fine_2 << 3 |
		self.Exclude32_2 << 2 |
		self.Dis_risedge_2 << 1 |
		self.Dis_faledge_2 << 0 )
		self.send_reg(58, reg_58)   
	def set_reg_60(self):	   
		reg_60 = (   
		self.Spr_puls_en_2 << 7 |
		self.Spr_puls_val_a_9to3_2 << 0 )
		self.send_reg(60, reg_60)   
	def set_reg_61(self):	   
		reg_61 = (   
		self.Spr_pulse_val_2to0_2 << 5 )
		self.send_reg(61, reg_61)   
	def set_reg_62(self):	   
		reg_62 = (   
		self.Spr_puls_val_b_9to2_2 << 0 )
		self.send_reg(62, reg_62)   
	def set_reg_63(self):	   
		reg_63 = (   
		self.Spr_puls_val_b_1to0_2 << 6 )
		self.send_reg(63, reg_63)   
	def set_reg_64(self):	   
		reg_64 = (   
		self.Thru_ris_en_2 << 7 |
		self.Thru_ris_tap_11to6_2 << 1 )
		self.send_reg(64, reg_64)   
	def set_reg_65(self):	   
		reg_65 = (   
		self.Thru_ris_tap_5to0_2 << 2 )
		self.send_reg(65, reg_65)   
	def set_reg_66(self):	   
		reg_66 = (   
		self.Thru_fal_en_2 << 7 |
		self.Thru_fal_tap_11to6_2 << 1 )
		self.send_reg(66, reg_66)   
	def set_reg_67(self):	   
		reg_67 = (   
		self.Thru_fal_tap_5to0_2 << 2 )
		self.send_reg(67, reg_67)   
	def set_reg_68(self):	   
		reg_68 = (   
		self.Dig_delay_2 << 7 |
		self.Clk_driver_en_2 << 6 |
		self.qu_reg_en_2 << 5 |
		self.qq_reg_en_2 << 4 |
		self.win_rst_2 << 3 |
		self.fineEn_2 << 2 |
		self.fineEnb_2 << 1 |
		self.rsffEn_2 << 0 )
		self.send_reg(68, reg_68)   
	def set_reg_69(self):	   
		reg_69 = (   
		self.dl_en_2 << 7 |
		self.cp_en_2 << 6 |
		self.forceCpUpb_2 << 5 |
		self.forceCpDn_2 << 4 |
		self.pdUpTune_1to0_2 << 2 |
		self.pdDnTune_1to0_2 << 0 )
		self.send_reg(69, reg_69)   
	def set_reg_70(self):	   
		reg_70 = (   
		self.cpUpTune_2to0_2 << 5 |
		self.cpDnTune_2to0_2 << 2 |
		self.pdEn_2 << 1 )
		self.send_reg(70, reg_70)   
	def set_reg_71(self):	   
		reg_71 = (   
		self.digClkPhase_7to0_2 << 0 )
		self.send_reg(71, reg_71)   
	def set_reg_72(self):	   
		reg_72 = (   
		self.Rst_n_async_2 << 7 )
		self.send_reg(72, reg_72)   
	def read_reg_73(self):	   
		reg_73 = self.get_reg(73)   
		self.L1_lup00_15to8_2 = reg_73   
		   
	def read_reg_74(self):	   
		reg_74 = self.get_reg(74)   
		self.L1_lup90_15to8_2 = reg_74   
		   
	def read_reg_75(self):	   
		reg_75 = self.get_reg(75)   
		self.Merg_ris_fin_2 = reg_75 >> 2   
		   
	def read_reg_76(self):	   
		reg_76 = self.get_reg(76)   
		self.Merg_fal_fin_2 = reg_76 >> 2   
		   
	def set_reg_77(self):	   
		reg_77 = (   
		self.Qg00degDelay_0to4_2 << 3 )
		self.send_reg(77, reg_77)   
	def set_reg_78(self):	   
		reg_78 = (   
		self.Qg90degDelay_0to4_2 << 3 )
		self.send_reg(78, reg_78)   
	def set_reg_79(self):	   
		reg_79 = (   
		self.Qg180degDelay_0to4_2 << 3 )
		self.send_reg(79, reg_79)   
	def set_reg_80(self):	   
		reg_80 = (   
		self.Qg270degDelay_0to4_2 << 3 )
		self.send_reg(80, reg_80)   
	def set_reg_81(self):	   
		reg_81 = (   
		self.DischargeTap16_3to0 << 4 |
		self.ChargeTap16_3to0 << 0 )
		self.send_reg(81, reg_81)   
	def set_reg_82(self):	   
		reg_82 = (   
		self.DischargeTapn_3to0 << 4 |
		self.ChargeTapn16_3to0 << 0 )
		self.send_reg(82, reg_82)   
	def set_reg_83(self):	   
		reg_83 = (   
		self.X1sel_32to39_2 << 0 )
		self.send_reg(83, reg_83)   
	def set_reg_84(self):	   
		reg_84 = (   
		self.X1sel_40to47_2 << 0 )
		self.send_reg(84, reg_84)   
	def set_reg_85(self):	   
		reg_85 = (   
		self.X2sel_32to36_2 << 3 )
		self.send_reg(85, reg_85)   
	def set_reg_86(self):	   
		reg_86 = (   
		self.X2sel_37to41_2 << 3 )
		self.send_reg(86, reg_86)   
	def set_reg_87(self):	   
		reg_87 = (   
		self.X4sel_32to36_2 << 3 )
		self.send_reg(87, reg_87)   
	def set_reg_88(self):	   
		reg_88 = (   
		self.X4sel_37to41_2 << 3 )
		self.send_reg(88, reg_88)   
	def set_reg_89(self):	   
		reg_89 = (   
		self.X8sel_32to36_2 << 3 )
		self.send_reg(89, reg_89)   
	def set_reg_90(self):	   
		reg_90 = (   
		self.X8sel_41_2 << 7 |
		self.X8sel_37to40_2 << 3 )
		self.send_reg(90, reg_90)   
	def set_reg_91(self):	   
		reg_91 = (   
		self.qutx_fb_180Cal_en << 7 |
		self.qutx_fb_0Cal_en << 6 |
		self.qutx_fb_180Rsff_en << 5 |
		self.qutx_fb_0Rsff_en << 4 )
		self.send_reg(91, reg_91)   
	def set_reg_96(self):	   
		reg_96 = (   
		self.N << 4 |
		self.R_11to8 << 0 )
		self.send_reg(96, reg_96)   
	def set_reg_97(self):	   
		reg_97 = (   
		self.R_7to0 << 0 )
		self.send_reg(97, reg_97)   
	def set_reg_98(self):	   
		reg_98 = (   
		self.Asyncrst_n << 7 |
		self.Cp_sel_6to0 << 0 )
		self.send_reg(98, reg_98)   
	def set_reg_99(self):	   
		reg_99 = (   
		self.Cp_sel_8to7 << 6 |
		self.ForceFout << 5 |
		self.ForceFoutb << 4 |
		self.Out_en << 3 |
		self.Dll_en << 2 |
		self.Ana_en << 1 )
		self.send_reg(99, reg_99)   
	def read_reg_100(self):	   
		reg_100 = self.get_reg(100)   
		self.Decod_in_0deg = reg_100 >> 3   
		   
	def set_reg_104(self):	   
		reg_104 = (   
		self.Ngt3_3 << 7 |
		self.NorNdiv4_3 << 0 )
		self.send_reg(104, reg_104)   
	def set_reg_105(self):	   
		reg_105 = (   
		self.RorFrNpRdiv4_25to18_3 << 0 )
		self.send_reg(105, reg_105)   
	def set_reg_106(self):	   
		reg_106 = (   
		self.RorFrNpRdiv4_17to10_3 << 0 )
		self.send_reg(106, reg_106)   
	def set_reg_107(self):	   
		reg_107 = (   
		self.RorFrNpRdiv4_9to2_3 << 0 )
		self.send_reg(107, reg_107)   
	def set_reg_108(self):	   
		reg_108 = (   
		self.RorFrNpRdiv4_1to0_3 << 6 )
		self.send_reg(108, reg_108)   
	def set_reg_109(self):	   
		reg_109 = (   
		self.Qu_tx_Ngt3_3 << 7 |
		self.NorNdiv4_phsh_3 << 0 )
		self.send_reg(109, reg_109)   
	def set_reg_110(self):	   
		reg_110 = (   
		self.RorFrNpRdiv4_phsh_25to18_3 << 0 )
		self.send_reg(110, reg_110)   
	def set_reg_111(self):	   
		reg_111 = (   
		self.RorFrNpRdiv4_phsh_17to10_3 << 0 )
		self.send_reg(111, reg_111)   
	def set_reg_112(self):	   
		reg_112 = (   
		self.RorFrNpRdiv4_phsh_9to2_3 << 0 )
		self.send_reg(112, reg_112)   
	def set_reg_113(self):	   
		reg_113 = (   
		self.RorFrNpRdiv4_phsh_1to0_3 << 6 )
		self.send_reg(113, reg_113)   
	def set_reg_114(self):	   
		reg_114 = (   
		self.Passthru_ref_clk_3 << 7 |
		self.Byp_ram_3 << 6 |
		self.Dis_adr_dith_3 << 5 |
		self.Dis_p5G_dith_3 << 4 |
		self.Byp_fine_3 << 3 |
		self.Exclude32_3 << 2 |
		self.Dis_risedge_3 << 1 |
		self.Dis_faledge_3 << 0 )
		self.send_reg(114, reg_114)   
	def set_reg_116(self):	   
		reg_116 = (   
		self.Spr_puls_en_3 << 7 |
		self.Spr_puls_val_a_9to3_3 << 0 )
		self.send_reg(116, reg_116)   
	def set_reg_117(self):	   
		reg_117 = (   
		self.Spr_pulse_val_2to0_3 << 5 )
		self.send_reg(117, reg_117)   
	def set_reg_118(self):	   
		reg_118 = (   
		self.Spr_puls_val_b_9to2_3 << 0 )
		self.send_reg(118, reg_118)   
	def set_reg_119(self):	   
		reg_119 = (   
		self.Spr_puls_val_b_1to0_3 << 6 )
		self.send_reg(119, reg_119)   
	def set_reg_120(self):	   
		reg_120 = (   
		self.Thru_ris_en_3 << 7 |
		self.Thru_ris_tap_11to6_3 << 1 )
		self.send_reg(120, reg_120)   
	def set_reg_121(self):	   
		reg_121 = (   
		self.Thru_ris_tap_5to0_3 << 2 )
		self.send_reg(121, reg_121)   
	def set_reg_122(self):	   
		reg_122 = (   
		self.Thru_fal_en_3 << 7 |
		self.Thru_fal_tap_11to6_3 << 1 )
		self.send_reg(122, reg_122)   
	def set_reg_123(self):	   
		reg_123 = (   
		self.Thru_fal_tap_5to0_3 << 2 )
		self.send_reg(123, reg_123)   
	def set_reg_124(self):	   
		reg_124 = (   
		self.Dig_delay_3 << 7 |
		self.Clk_driver_en_3 << 6 |
		self.qu_reg_en_3 << 5 |
		self.qq_reg_en_3 << 4 |
		self.win_rst_3 << 3 |
		self.fineEn_3 << 2 |
		self.fineEnb_3 << 1 |
		self.rsffEn_3 << 0 )
		self.send_reg(124, reg_124)   
	def set_reg_125(self):	   
		reg_125 = (   
		self.dl_en_3 << 7 |
		self.cp_en_3 << 6 |
		self.forceCpUpb_3 << 5 |
		self.forceCpDn_3 << 4 |
		self.pdUpTune_1to0_3 << 2 |
		self.pdDnTune_1to0_3 << 0 )
		self.send_reg(125, reg_125)   
	def set_reg_126(self):	   
		reg_126 = (   
		self.cpUpTune_2to0_3 << 5 |
		self.cpDnTune_2to0_3 << 2 |
		self.pdEn_3 << 1 )
		self.send_reg(126, reg_126)   
	def set_reg_127(self):	   
		reg_127 = (   
		self.digClkPhase_7to0_3 << 0 )
		self.send_reg(127, reg_127)   
	def set_reg_128(self):	   
		reg_128 = (   
		self.Rst_n_async_3 << 7 )
		self.send_reg(128, reg_128)   
	def read_reg_129(self):	   
		reg_129 = self.get_reg(129)   
		self.L1_lup00_15to8_3 = reg_129   
		   
	def read_reg_130(self):	   
		reg_130 = self.get_reg(130)   
		self.L1_lup90_15to8_3 = reg_130   
		   
	def read_reg_131(self):	   
		reg_131 = self.get_reg(131)   
		self.Merg_ris_fin_3 = reg_131 >> 2   
		   
	def read_reg_132(self):	   
		reg_132 = self.get_reg(132)   
		self.Merg_fal_fin_3 = reg_132 >> 2   
		   
	def set_reg_133(self):	   
		reg_133 = (   
		self.Qg00degDelay_0to4_3 << 3 )
		self.send_reg(133, reg_133)   
	def set_reg_134(self):	   
		reg_134 = (   
		self.Qg90degDelay_0to4_3 << 3 )
		self.send_reg(134, reg_134)   
	def set_reg_135(self):	   
		reg_135 = (   
		self.Qg180degDelay_0to4_3 << 3 )
		self.send_reg(135, reg_135)   
	def set_reg_136(self):	   
		reg_136 = (   
		self.Qg270degDelay_0to4_3 << 3 )
		self.send_reg(136, reg_136)   
	def set_reg_137(self):	   
		reg_137 = (   
		self.DischargeTap16_0to3_3 << 4 |
		self.ChargeTap16_0to3_3 << 0 )
		self.send_reg(137, reg_137)   
	def set_reg_138(self):	   
		reg_138 = (   
		self.DischargeTapn_0to3_3 << 4 |
		self.ChargeTapn16_0to3_3 << 0 )
		self.send_reg(138, reg_138)   
	def set_reg_139(self):	   
		reg_139 = (   
		self.X1sel_32to39_3 << 0 )
		self.send_reg(139, reg_139)   
	def set_reg_140(self):	   
		reg_140 = (   
		self.X1sel_40to47_3 << 0 )
		self.send_reg(140, reg_140)   
	def set_reg_141(self):	   
		reg_141 = (   
		self.X2sel_32to36_3 << 3 )
		self.send_reg(141, reg_141)   
	def set_reg_142(self):	   
		reg_142 = (   
		self.X2sel_37to41_3 << 3 )
		self.send_reg(142, reg_142)   
	def set_reg_143(self):	   
		reg_143 = (   
		self.X4sel_32to36_3 << 3 )
		self.send_reg(143, reg_143)   
	def set_reg_144(self):	   
		reg_144 = (   
		self.X4sel_37to41_3 << 3 )
		self.send_reg(144, reg_144)   
	def set_reg_145(self):	   
		reg_145 = (   
		self.X8sel_32to36_3 << 3 )
		self.send_reg(145, reg_145)   
	def set_reg_146(self):	   
		reg_146 = (   
		self.X8sel_41_3 << 7 |
		self.X8sel_37to40_3 << 3 )
		self.send_reg(146, reg_146)   
	def set_reg_147(self):	   
		reg_147 = (   
		self.qurx_180Cal_en << 7 |
		self.qurx_0Cal_en << 6 )
		self.send_reg(147, reg_147)   
	def set_reg_152(self):	   
		reg_152 = (   
		self.extClkEn << 7 |
		self.extClkEnBNOTD7 << 6 |
		self.div2_rst << 5 |
		self.TxChClkSel << 3 )
		self.send_reg(152, reg_152)   
	def set_reg_153(self):	   
		reg_153 = (   
		self.TxChClkEn << 5 )
		self.send_reg(153, reg_153)   
	def set_reg_156(self):	   
		reg_156 = (   
		self.tx_bb_en << 7 |
		self.tx_bb_fdbk_bw << 5 |
		self.tx_bb_fdbk_cal_en << 4 |
		self.tx_bb_fdbk_cart_err_en << 3 |
		self.tx_bb_fdbk_cart_fb_en << 2 |
		self.tx_bb_fdbk_cart_fwd_en << 1 )
		self.send_reg(156, reg_156)   
	def set_reg_157(self):	   
		reg_157 = (   
		self.tx_bb_fdbk_en << 6 |
		self.tx_bb_fdbk_1q_sel << 5 |
		self.tx_bb_fdbk_lp << 2 )
		self.send_reg(157, reg_157)   
	def set_reg_158(self):	   
		reg_158 = (   
		self.tx_bb_fdbk_statt << 5 |
		self.tx_bb_fdbk_swapi << 4 |
		self.tx_bb_fdbk_swapq << 3 |
		self.tx_bb_gain_cmp << 2 )
		self.send_reg(158, reg_158)   
	def set_reg_159(self):	   
		reg_159 = (   
		self.tx_bb_lp << 5 |
		self.tx_bb_swapi << 4 |
		self.tx_bb_swapq << 3 |
		self.tx_butt_bw << 0 )
		self.send_reg(159, reg_159)   
	def set_reg_160(self):	   
		reg_160 = (   
		self.tx_bw_trck << 4 |
		self.tx_cart_en << 3 )
		self.send_reg(160, reg_160)   
	def set_reg_161(self):	   
		reg_161 = (   
		self.tx_cart_fb_bb_statt << 3 )
		self.send_reg(161, reg_161)   
	def set_reg_162(self):	   
		reg_162 = (   
		self.tx_cart_fb_dcoc_dac_I1 << 2 )
		self.send_reg(162, reg_162)   
	def set_reg_163(self):	   
		reg_163 = (   
		self.tx_cart_fb_dcoc_dac_I2 << 2 )
		self.send_reg(163, reg_163)   
	def set_reg_164(self):	   
		reg_164 = (   
		self.tx_cart_fb_dcoc_dac_Q1 << 2 )
		self.send_reg(164, reg_164)   
	def set_reg_165(self):	   
		reg_165 = (   
		self.tx_cart_fb_dcoc_dac_Q2 << 2 )
		self.send_reg(165, reg_165)   
	def set_reg_166(self):	   
		reg_166 = (   
		self.CartesianFeedbackpathDCOCenable << 7 |
		self.CartesianFeedbackpathenable << 6 |
		self.CartesianFBpathHiResolutionDCOCenable << 5 |
		self.CartesianFBpathBW << 1 )
		self.send_reg(166, reg_166)   
	def set_reg_167(self):	   
		reg_167 = (   
		self.CartesianFBRFGain << 2 )
		self.send_reg(167, reg_167)   
	def set_reg_168(self):	   
		reg_168 = (   
		self.CartesianFBpathSwapIandIx << 7 |
		self.CartesianFBpathSwapQandQx << 6 |
		self.CartesianFBpathSwitchtoforwardSummer << 5 |
		self.tx_cart_fb_lo_select << 0 )
		self.send_reg(168, reg_168)   
	def set_reg_169(self):	   
		reg_169 = (   
		self.CartesianFBpathAmp1Gain << 6 |
		self.CartesianFBpathAmp2Gain << 4 |
		self.CartesianFBpathAmp3Gain << 2 |
		self.CartesianFBpathAmp4Gain << 0 )
		self.send_reg(169, reg_169)   
	def set_reg_170(self):	   
		reg_170 = (   
		self.CartesianFBpathAmpCurrentSelect << 5 |
		self.CartesianFBpathZeroEnable << 4 |
		self.tx_cart_zero_statt << 0 )
		self.send_reg(170, reg_170)   
	def set_reg_171(self):	   
		reg_171 = (   
		self.tx_inbuf_bw << 6 |
		self.tx_inbuf_statt << 3 )
		self.send_reg(171, reg_171)   
	def set_reg_172(self):	   
		reg_172 = (   
		self.tx_output_channel_sel << 5 )
		self.send_reg(172, reg_172)   
	def set_reg_173(self):	   
		reg_173 = (   
		self.tx_p1_bw << 4 |
		self.tx_pw_bw1 << 2 )
		self.send_reg(173, reg_173)   
	def set_reg_174(self):	   
		reg_174 = (   
		self.tx_p2_bw2 << 4 |
		self.PushPullBufferCurrent << 1 )
		self.send_reg(174, reg_174)   
	def set_reg_175(self):	   
		reg_175 = (   
		self.tx_rf_aoc_bw << 6 |
		self.RFForwardPathEnable_toMUX << 5 |
		self.RFForwardPathEnable_ExternalPinenable << 4 |
		self.tx_rf_fwd_lp << 1 )
		self.send_reg(175, reg_175)   
	def set_reg_176(self):	   
		reg_176 = (   
		self.tx_rf_fwd_statt1 << 5 |
		self.tx_rf_fwd_statt2 << 2 )
		self.send_reg(176, reg_176)   
	def set_reg_177(self):	   
		reg_177 = (   
		self.BBQDivideby2or4Select << 7 |
		self.BBQQuadGenEnable << 6 |
		self.BBQPolyphaseQuadGenEnable << 5 )
		self.send_reg(177, reg_177)   
	def set_reg_178(self):	   
		reg_178 = (   
		self.lofb_tun_s << 4 |
		self.lofb_tun_sx << 0 )
		self.send_reg(178, reg_178)   
	def set_reg_179(self):	   
		reg_179 = (   
		self.lofw_tun_s2 << 4 |
		self.lofw_tun_sx2 << 0 )
		self.send_reg(179, reg_179)   
	def set_reg_180(self):	   
		reg_180 = (   
		self.reserve_tx26 << 0 )
		self.send_reg(180, reg_180)   
	def set_reg_181(self):	   
		reg_181 = (   
		self.reserve_tx27 << 0 )
		self.send_reg(181, reg_181)   
	def set_reg_192(self):	   
		reg_192 = (   
		self.rx_Idac << 3 |
		self.rx_dcs << 1 |
		self.rx_den << 0 )
		self.send_reg(192, reg_192)   
	def set_reg_193(self):	   
		reg_193 = (   
		self.rx_Qdac << 3 |
		self.rx_cmpen << 1 |
		self.rx_dcoc << 0 )
		self.send_reg(193, reg_193)   
	def set_reg_194(self):	   
		reg_194 = (   
		self.rx_ten << 7 |
		self.rx_ren << 6 |
		self.rx_dven << 4 |
		self.rx_dv << 0 )
		self.send_reg(194, reg_194)   
	def set_reg_195(self):	   
		reg_195 = (   
		self.rx_extc << 7 |
		self.rx_cen << 4 |
		self.rx_chck << 2 |
		self.rx_chcken << 1 |
		self.rx_fen << 0 )
		self.send_reg(195, reg_195)   
	def set_reg_196(self):	   
		reg_196 = (   
		self.rx_onchen << 7 |
		self.rx_offchen << 6 |
		self.rx_foe << 0 )
		self.send_reg(196, reg_196)   
	def set_reg_197(self):	   
		reg_197 = (   
		self.rx_offch << 5 |
		self.rx_onchf << 3 |
		self.rx_onchc << 1 )
		self.send_reg(197, reg_197)   
	def set_reg_198(self):	   
		reg_198 = (   
		self.rx_qs << 5 |
		self.rx_bqg << 3 |
		self.rx_rq << 0 )
		self.send_reg(198, reg_198)   
	def set_reg_199(self):	   
		reg_199 = (   
		self.rx_rv << 5 |
		self.rx_rip << 2 |
		self.rx_rfp << 0 )
		self.send_reg(199, reg_199)   
	def set_reg_200(self):	   
		reg_200 = (   
		self.rx_cp_12to8 << 3 |
		self.rx_gs << 0 )
		self.send_reg(200, reg_200)   
	def set_reg_201(self):	   
		reg_201 = (   
		self.rx_cp_7to0 << 0 )
		self.send_reg(201, reg_201)   
	def set_reg_202(self):	   
		reg_202 = (   
		self.rx_cv_10to3 << 0 )
		self.send_reg(202, reg_202)   
	def set_reg_203(self):	   
		reg_203 = (   
		self.rx_cv_2to0 << 5 |
		self.rx_cc_2to0 << 2 |
		self.rx_cq_9to8 << 0 )
		self.send_reg(203, reg_203)   
	def set_reg_204(self):	   
		reg_204 = (   
		self.rx_cq_7to0 << 0 )
		self.send_reg(204, reg_204)   
	def set_reg_205(self):	   
		reg_205 = (   
		self.rx_lna << 5 |
		self.rx_lnab << 3 |
		self.rx_rxchen << 2 |
		self.rx_bbq_div2or4 << 1 |
		self.rx_Loselect << 0 )
		self.send_reg(205, reg_205)   
	def set_reg_206(self):	   
		reg_206 = (   
		self.poly_en << 7 )
		self.send_reg(206, reg_206)   
	def set_reg_207(self):	   
		reg_207 = (   
		self.lorx_tun_s << 4 |
		self.lorx_tun_sx << 0 )
		self.send_reg(207, reg_207)   
	def read_reg_208(self):	   
		reg_208 = self.get_reg(208)   
		self.rx_Icmpo = reg_208 >> 5   
		self.rx_Iodac = reg_208 % 64   
		   
	def read_reg_209(self):	   
		reg_209 = self.get_reg(209)   
		self.rx_Qcmpo = reg_209 >> 5   
		self.rx_Qodac = reg_209 % 64   
		   
	def read_reg_210(self):	   
		reg_210 = self.get_reg(210)   
		self.rx_rc = reg_210   
		   
	def set_reg_220(self):	   
		reg_220 = (   
		self.shft_cml_in << 7 |
		self.vagenable1 << 6 |
		self.vagenable2 << 5 )
		self.send_reg(220, reg_220)   
	def set_reg_222(self):	   
		reg_222 = (   
		self.TestMuxBufferEnable << 7 |
		self.TestMuxEnable << 6 |
		self.TestMuxSetting << 0 )
		self.send_reg(222, reg_222)

	#The SPI format is 8 bits, plus a two-byte header
	# The format is:
	# Byte sent on MOSI	Bit	Description
	# ------------------------------------------------------------------------------
	# 1			7	Not W - Read/write indicator, where 0 indicates
	#				a write and 1 indicates a read
	#			6-0	Upper 7 bits of the register address
	# ------------------------------------------------------------------------------
	# 2			7-1	Lower 7 bits of the register address
	#			0	If 1, will disable the auto-increment of the
	#				register address.
	# ------------------------------------------------------------------------------
	# 3, ..., n+3		7-0	Optionally n words of write data byte
	#
	# Byte sent on MISO	Bit	Description
	# ------------------------------------------------------------------------------
	# 1			7-0	Read data returned that was read during
	#				the last transfer
	# ------------------------------------------------------------------------------
	# 2			7-0	0s will be forced
	# ------------------------------------------------------------------------------
	# 3, ..., n+3		7-0	Optionally n words of read data byte

	#Send register read to SPI, get result
	#	    _read_spi()
	#Type           Sub Function
	#Description    Read data from SPI bus peripheral.
	#	       Return the data read if successful, else a zero length string.
	#Usage          usrp.source_x._read_spi(optional_header, enables, format, len)
	#	       optional_header : 0,1 or 2 bytes to write before buf.
	#Parameters
	#	       enables : bitmask of peripherals to write.
	#	        format : transaction format. SPI_FMT_*
	#	       len : number of bytes to read.#

	#Write register to SPI
	#Type        Sub Function
	#Description Write data to SPI bus peripheral.
	#	    SPI == "Serial Port Interface". SPI is a 3 wire bus plus a separate enable for each
	#	    peripheral. The common lines are SCLK,SDI and SDO. The FX2 always drives SCLK
	#	    and SDI, the clock and data lines from the FX2 to the peripheral. When enabled, a
	#	    peripheral may drive SDO, the data line from the peripheral to the FX2.
	#	    The SPI_READ and SPI_WRITE commands are formatted identically.
	#	    Each specifies which peripherals to enable, whether the bits should be transmistted Most
	#	    Significant Bit first or Least Significant Bit first, the number of bytes in the optional
	#	    header, and the number of bytes to read or write in the body.
	#	    The body is limited to 64 bytes. The optional header may contain 0, 1 or 2 bytes. For an
	#	    SPI_WRITE, the header bytes are transmitted to the peripheral followed by the the body
	#	    bytes. For an SPI_READ, the header bytes are transmitted to the peripheral, then len
	#	    bytes are read back from the peripheral.(see : usrp_spi_defs.h file). If format specifies
	#	    that optional_header bytes are present, they are written to the peripheral immediately
	#	    prior to writing buf.
	#	    Return true if successful. Writes are limited to a maximum of 64 bytes.
	#Usage       usrp.source_x._write_spi(optional_header, enables, format, buf)
	#Parameters  optional_header: 0,1 or 2 bytes to write before buf.
	#	    enables: bitmask of peripherals to write.
	#	     format: transaction format. SPI_FMT_*
	#	    buf : the data to write#

	def send_reg(self, regnum, dat):
		#Send 16 bit header over SPI to send register number
		#Write 8 bit register
		#Set first byte of header
		#hdr_hi = int( (regnum >> 7) & 0x7f)
		#Set second byte of header
		#hdr_lo = int( (regnum << 1) & 0xff)
		#Set full two-byte header
		#hdr = ((hdr_hi << 8) + hdr_lo) & 0x7fff

		hdr = int( (regnum << 1) & 0x7ffe)

		#Set byte of write data
		s = chr(dat & 0xff)
		#Send data over SPI
		self.u._write_spi(hdr, self.spi_enable, self.spi_format, s)
		print 'RFIC4: Writing register %d with %d' % (regnum, dat)

	def get_reg(self, regnum):
		#Send 16 bit header over SPI to send register number
		#Read 8 bit register
		#Set first byte of header
		#hdr_hi = chr( ( (regnum >> 7) + (1 << 7) ) & 0xff)
		#Set second byte of header
		#hdr_lo = chr( (regnum << 1) & 0xff)
		#Set full two-byte header
		#hdr = ((hdr_hi << 8) + hdr_lo) & 0xffff

		#Send data over SPI, get register contents
		#r = self.u._read_spi(hdr, self.spi_enable, self.spi_format, 1)

		# First set register zero, to set the SPI register number to zero, then get all registers, then return desired register as integer

		# Get data to set register zero
		dat = self.Ngt3 << 7 | self.NorNdiv4 << 0

		r = self.u._write_spi(0, self.spi_enable, self.spi_format, chr(dat & 0xff))

		# Get all registers, no header required
		read = self.u._read_spi(0, self.spi_enable, self.spi_format_no_header, 64)
		read = read + self.u._read_spi(0, self.spi_enable, self.spi_format_no_header, 64)
		read = read + self.u._read_spi(0, self.spi_enable, self.spi_format_no_header, 64)
		read = read + self.u._read_spi(0, self.spi_enable, self.spi_format_no_header, 64)
		read = read + self.u._read_spi(0, self.spi_enable, self.spi_format_no_header, 64)

		# Return desired register as integer
		r = ord(read[regnum])

		print 'RFIC4: Reading register %d' % (regnum)
		return r
		

	# --------------------------------------------------------------------
	# These methods control the GPIO bus.  Since the board has to access
	# both the io_rx_* and io_tx_* pins, we define our own methods to do so.
	# This bypasses any code in db_base.
	#
	# The board operates in ATR mode, always.  Thus, when the board is first
	# initialized, it is in receive mode, until bits show up in the TX FIFO.
	#
	def rx_write_oe(self, value, mask):
		return self.u._write_fpga_reg((FR_OE_1, FR_OE_3)[self.which], gru.hexint((mask << 16) | value))

	def rx_write_io(self, value, mask):
		return self.u._write_fpga_reg((FR_IO_1, FR_IO_3)[self.which], gru.hexint((mask << 16) | value))

	def rx_read_io(self):
		t = self.u._read_fpga_reg((FR_RB_IO_RX_A_IO_TX_A, FR_RB_IO_RX_B_IO_TX_B)[self.which])
		return (t >> 16) & 0xffff

	def rx_set_atr_mask(self, v):
		#print 'Set mask to %s' % (v)
		return self.u._write_fpga_reg((FR_ATR_MASK_1,FR_ATR_MASK_3)[self.which], gru.hexint(v))

	def rx_set_atr_txval(self, v):
		#print 'Set TX value to %s' % (v)
		return self.u._write_fpga_reg((FR_ATR_TXVAL_1,FR_ATR_TXVAL_3)[self.which], gru.hexint(v))

	def rx_set_atr_rxval(self, v):
		#print 'Set RX value to %s' % (v)
		return self.u._write_fpga_reg((FR_ATR_RXVAL_1,FR_ATR_RXVAL_3)[self.which], gru.hexint(v))

	# --------------------------------------------------------------------
	# These methods set control the high-level operating parameters.

	def set_rx_gain(self, gain):
		# Set RX gain
		# @param gain: gain in dB
		# Four parameters: self.rx_bqg, self.rx_dcs, self.rx_gs, self.rx_rip
		# 1 to 39 dB of gain (0 to 38)
		# Not all steps available
		if gain < 0.0: gain = 0.0
		if gain > 38.0: gain = 38.0

		if gain <= 3:
			self.rx_bqg = 3	#reg 198
			self.rx_dcs = 0	#reg 192
			self.rx_gs = 4	#reg 200
			self.rx_rip = 4	#reg 199

		elif gain >= 3 and gain < 4:
			self.rx_bqg = 3
			self.rx_dcs = 0
			self.rx_gs = 4
			self.rx_rip = 3

		elif gain >= 4 and gain < 5:
			self.rx_bqg = 3
			self.rx_dcs = 0
			self.rx_gs = 2
			self.rx_rip = 4

		elif gain >=5  and gain < 6:
			self.rx_bqg = 3
			self.rx_dcs = 0
			self.rx_gs = 3
			self.rx_rip = 3

		elif gain >= 6 and gain < 7:
			self.rx_bqg = 3
			self.rx_dcs = 0
			self.rx_gs = 4
			self.rx_rip = 2

		elif gain >= 7 and gain < 8:
			self.rx_bqg = 3
			self.rx_dcs = 0
			self.rx_gs = 2
			self.rx_rip = 3

		elif gain >= 8 and gain < 9:
			self.rx_bqg = 3
			self.rx_dcs = 0
			self.rx_gs = 3
			self.rx_rip = 2

		elif gain >= 9 and gain < 10:
			self.rx_bqg = 3
			self.rx_dcs = 0
			self.rx_gs = 1
			self.rx_rip = 3

		elif gain >= 10 and gain < 11:
			self.rx_bqg = 3
			self.rx_dcs = 0
			self.rx_gs = 2
			self.rx_rip = 2

		elif gain >= 11 and gain < 12:
			self.rx_bqg = 3
			self.rx_dcs = 0
			self.rx_gs = 0
			self.rx_rip = 3

		elif gain >= 12 and gain < 13:
			self.rx_bqg = 2
			self.rx_dcs = 0
			self.rx_gs = 4
			self.rx_rip = 2

		elif gain >= 13 and gain < 14:
			self.rx_bqg = 3
			self.rx_dcs = 0
			self.rx_gs = 2
			self.rx_rip = 1

		elif gain >= 14 and gain < 15:
			self.rx_bqg = 3
			self.rx_dcs = 0
			self.rx_gs = 0
			self.rx_rip = 2

		elif gain >= 15 and gain < 16:
			self.rx_bqg = 2
			self.rx_dcs = 0
			self.rx_gs = 1
			self.rx_rip = 3

		elif gain >= 16 and gain < 17:
			self.rx_bqg = 2
			self.rx_dcs = 0
			self.rx_gs = 2
			self.rx_rip = 2

		elif gain >= 17 and gain < 18:
			self.rx_bqg = 3
			self.rx_dcs = 0
			self.rx_gs = 0
			self.rx_rip = 2

		elif gain >= 18 and gain < 19:
			self.rx_bqg = 3
			self.rx_dcs = 0
			self.rx_gs = 1
			self.rx_rip = 0

		elif gain >= 19 and gain < 20:
			self.rx_bqg = 2
			self.rx_dcs = 0
			self.rx_gs = 2
			self.rx_rip = 1

		elif gain >= 20 and gain < 21:
			self.rx_bqg = 3
			self.rx_dcs = 0
			self.rx_gs = 0
			self.rx_rip = 0

		elif gain >= 21 and gain < 22:
			self.rx_bqg = 2
			self.rx_dcs = 0
			self.rx_gs = 1
			self.rx_rip = 1

		elif gain >= 22 and gain < 23:
			self.rx_bqg = 1
			self.rx_dcs = 1
			self.rx_gs = 2
			self.rx_rip = 2

		elif gain >= 23 and gain < 24:
			self.rx_bqg = 2
			self.rx_dcs = 0
			self.rx_gs = 0
			self.rx_rip = 1

		elif gain >= 24 and gain < 25:
			self.rx_bqg = 1
			self.rx_dcs = 1
			self.rx_gs = 1
			self.rx_rip = 2

		elif gain >= 25 and gain < 26:
			self.rx_bqg = 1
			self.rx_dcs = 1
			self.rx_gs = 2
			self.rx_rip = 1

		elif gain >= 26 and gain < 27:
			self.rx_bqg = 1
			self.rx_dcs = 1
			self.rx_gs = 3
			self.rx_rip = 0

		elif gain >= 27 and gain < 28:
			self.rx_bqg = 1
			self.rx_dcs = 1
			self.rx_gs = 1
			self.rx_rip = 1

		elif gain >= 28 and gain < 29:
			self.rx_bqg = 1
			self.rx_dcs = 1
			self.rx_gs = 2
			self.rx_rip = 0

		elif gain >= 29 and gain < 30:
			self.rx_bqg = 1
			self.rx_dcs = 1
			self.rx_gs = 0
			self.rx_rip = 1

		elif gain >= 30 and gain < 31:
			self.rx_bqg = 1
			self.rx_dcs = 1
			self.rx_gs = 1
			self.rx_rip = 0

		elif gain >= 31 and gain < 32:
			self.rx_bqg = 0
			self.rx_dcs = 3
			self.rx_gs = 2
			self.rx_rip = 1

		elif gain >= 32 and gain < 33:
			self.rx_bqg = 1
			self.rx_dcs = 1
			self.rx_gs = 0
			self.rx_rip = 0

		elif gain >= 33 and gain < 34:
			self.rx_bqg = 0
			self.rx_dcs = 3
			self.rx_gs = 1
			self.rx_rip = 1

		elif gain >= 34 and gain < 35:
			self.rx_bqg = 0
			self.rx_dcs = 3
			self.rx_gs = 2
			self.rx_rip = 0

		elif gain >= 35 and gain < 36:
			self.rx_bqg = 0
			self.rx_dcs = 3
			self.rx_gs = 0
			self.rx_rip = 1

		elif gain >= 36 and gain < 38:
			self.rx_bqg = 0
			self.rx_dcs = 3
			self.rx_gs = 1
			self.rx_rip = 0

		elif gain >= 38:
			self.rx_bqg = 0
			self.rx_dcs = 3
			self.rx_gs = 0
			self.rx_rip = 0

		self.set_reg_198()
		self.set_reg_192()
		self.set_reg_200()
		self.set_reg_199()


	def set_tx_gain(self, gain):
		# Set TX gain
		# @param gain: output gain in dB
		# Two parameters:
		# self.tx_rf_fwd_statt1, self.tx_rf_fwd_statt2
		# (45 dB of range)
		# 5 dB steps
		if gain < 0.0: gain = 0.0
		if gain > 45.0: gain = 45.0

		if gain <= 2.5:
			self.tx_rf_fwd_statt1 = 7
			self.tx_rf_fwd_statt2 = 7

		elif gain > 2.5 and gain <= 7.5:
			self.tx_rf_fwd_statt1 = 3
			self.tx_rf_fwd_statt2 = 7

		elif gain > 7.5 and gain <= 12.5:
			self.tx_rf_fwd_statt1 = 1
			self.tx_rf_fwd_statt2 = 7

		elif gain > 12.5 and gain <= 17.5:
			self.tx_rf_fwd_statt1 = 3
			self.tx_rf_fwd_statt2 = 3

		elif gain > 17.5 and gain <= 22.5:
			self.tx_rf_fwd_statt1 = 1
			self.tx_rf_fwd_statt2 = 3

		elif gain > 22.5 and gain <= 27.5:
			self.tx_rf_fwd_statt1 = 0
			self.tx_rf_fwd_statt2 = 3

		elif gain > 27.5 and gain <= 32.5:
			self.tx_rf_fwd_statt1 = 1
			self.tx_rf_fwd_statt2 = 1

		elif gain > 32.5 and gain <= 37.5:
			self.tx_rf_fwd_statt1 = 0
			self.tx_rf_fwd_statt2 = 1

		elif gain > 37.5 and gain <= 42.5:
			self.tx_rf_fwd_statt1 = 1
			self.tx_rf_fwd_statt2 = 0

		elif gain > 42.5:
			self.tx_rf_fwd_statt1 = 0
			self.tx_rf_fwd_statt2 = 0

		self.set_reg_176()

	def set_fb_gain(self, gain):
		# Set Feedback path gain
		# @param gain: output gain in dB
		#  parameters:
		# self.CartesianFBpathAmp1Gain, self.CartesianFBpathAmp2Gain,
		# self.CartesianFBpathAmp3Gain, self.CartesianFBpathAmp4Gain
		# (40 dB of range)
		# 5 dB steps
		# FIXME
		if gain < 0.0: gain = 0.0
		if gain > 40.0: gain = 40.0

		if gain <= 2.5:
			self.CartesianFBpathAmp1Gain = 3
			self.CartesianFBpathAmp2Gain = 3
			self.CartesianFBpathAmp3Gain = 3
			self.CartesianFBpathAmp4Gain = 3

		elif gain > 2.5 and gain <= 7.5:
			self.CartesianFBpathAmp1Gain = 3
			self.CartesianFBpathAmp2Gain = 3
			self.CartesianFBpathAmp3Gain = 3
			self.CartesianFBpathAmp4Gain = 1

		elif gain > 7.5 and gain <= 12.5:
			self.CartesianFBpathAmp1Gain = 3
			self.CartesianFBpathAmp2Gain = 3
			self.CartesianFBpathAmp3Gain = 1
			self.CartesianFBpathAmp4Gain = 1

		elif gain > 12.5 and gain <= 17.5:
			self.CartesianFBpathAmp1Gain = 3
			self.CartesianFBpathAmp2Gain = 1
			self.CartesianFBpathAmp3Gain = 1
			self.CartesianFBpathAmp4Gain = 1

		elif gain > 17.5 and gain <= 22.5:
			self.CartesianFBpathAmp1Gain = 1
			self.CartesianFBpathAmp2Gain = 1
			self.CartesianFBpathAmp3Gain = 1
			self.CartesianFBpathAmp4Gain = 1

		elif gain > 22.5 and gain <= 27.5:
			self.CartesianFBpathAmp1Gain = 1
			self.CartesianFBpathAmp2Gain = 1
			self.CartesianFBpathAmp3Gain = 1
			self.CartesianFBpathAmp4Gain = 0

		elif gain > 27.5 and gain <= 32.5:
			self.CartesianFBpathAmp1Gain = 1
			self.CartesianFBpathAmp2Gain = 1
			self.CartesianFBpathAmp3Gain = 0
			self.CartesianFBpathAmp4Gain = 0

		elif gain > 32.5 and gain <= 37.5:
			self.CartesianFBpathAmp1Gain = 1
			self.CartesianFBpathAmp2Gain = 0
			self.CartesianFBpathAmp3Gain = 0
			self.CartesianFBpathAmp4Gain = 0

		elif gain > 37.5:
			self.CartesianFBpathAmp1Gain = 0
			self.CartesianFBpathAmp2Gain = 0
			self.CartesianFBpathAmp3Gain = 0
			self.CartesianFBpathAmp4Gain = 0

		self.set_reg_169()

	def calc_freq_vars(self, Fclk, Fout):
		#
		#@param Fclk: Clock frequency of board (Hz)
		#@type Fclk: float
		#@param Fout: Desired clock frequency for one of three frequency synthesizers (Hz)
		#@type Fout: float
		#
		# Calculate RFIC register variables to set frequency of frequency synthesizers
		# data1 corresponds to Ngt, D7, a single bit
		# data2 corresponds to NorNdiv4, D6-D0, up to seven bits
		# data3 corresponds to RorFrNpRdiv4, up to 26 bits
		# D7-D0, D7-D0, D7-D0, D7-D6
		# Returns Ngt, NorNdiv4, RorFrNpRdiv4_25to18, RorFrNpRdiv4_17to10,
		# RorFrNpRdiv4_9to2, RorFrNpRdiv4_1to0

		if (Fout > Fclk / 4):
			NpR = (2 ** -26) * int(2 ** 26 * Fclk / Fout)
			data1 = 0;
			data2 = int(floor(NpR))
			data3 = int(2 ** 26 * (NpR - floor(NpR)))
		else:
			NpR = (2 ** -24) * int(2 ** 24 * Fclk / Fout)
			data1 = 1
			data2 = int(floor(NpR / 4))
			data3 = int(2 ** 26 * (NpR / 4 - floor(NpR / 4)))

		Ngt = data1
		NorNdiv4 = data2
		RorFrNpRdiv4_25to18 = data3 >> 18
		temp = data3 % (2 ** 18)
		RorFrNpRdiv4_17to10 = temp >> 10
		temp = data3 % (2 ** 10)
		RorFrNpRdiv4_9to2 = temp >> 2
		RorFrNpRdiv4_1to0 = data3 % (2 ** 2)

		return (Ngt, NorNdiv4, RorFrNpRdiv4_25to18, RorFrNpRdiv4_17to10, RorFrNpRdiv4_9to2, RorFrNpRdiv4_1to0)

	def calc_phase_vars(self, Fclk, Fout, phsh):
		#
		#@param Fclk: Clock frequency of board (Hz)
		#@type Fclk: float
		#@param Fout: Desired clock frequency for one of three frequency synthesizers (Hz)
		#@type Fout: float
		#@param phsh: Desired phase shift in degrees
		#@type phsh: float
		#
		# Calculate RFIC register variables to set phase of frequency synthesizers
		# data1 is NGT3_phsh, D7, a single bit
		# data2 is NorNdiv4_phsh, D6-D0, up to 7 bits
		# data3 is RorFrNpRdiv4_phsh, up to 26 bits
		# D7-D0, D7-D0, D7-D0, D7-D6
		# Returns Ngt_phsh, NorNdiv4_phsh, RorFrNpRdiv4_25to18_phsh, 
		# RorFrNpRdiv4_17to10_phsh, RorFrNpRdiv4_9to2_phsh, RorFrNpRdiv4_1to0_phsh

		if (Fout <= Fclk / 4):
			mod1 = phsh - 360 * floor(phsh / 360)
			NpR = (2 ** -24) * int(2 ** 24 * Fclk / Fout)
			tmp = (1 + mod1 / 360 / 2) * NpR
		else:
			mod1 = phsh - 360 * floor(phsh / 360)
			NpR = (2 ** -26) * int(2 ** 26 * Fclk / Fout)
			tmp = (1 + mod1 / 360 / 2) * NpR

		if (tmp < 4):
			NpR_ph = (2 ** -26) * int(2 ** 26 * (1 + mod1 / 360 / 8) * NpR)
			data1 = 0
			data2 = int(floor(NpR_ph))
			data3 = int(2 ** 26 * (NpR_ph - floor(NpR_ph)))
		elif ((tmp >=4) and (tmp < 508)):
			NpR_ph = (2 ** -24) * int(2 ** 24 * tmp)
			data1 = 1
			data2 = int(floor(NpR_ph / 4))
			data3 = int(2 ** 26 * (NpR_ph / 4 - floor(NpR_ph / 4)))
		elif (tmp >= 508):
			NpR_ph = (2 ** -24) * int(2 ** 24 * (1 + (mod1 - 360) / 360 /2) * NpR)
			data1 = 1
			data2 = int(floor(NpR_ph / 4))
			data3 = int(2 ** 26 * (NpR_ph / 4 - floor(NpR_ph / 4)))

		Ngt_phsh = data1
		NorNdiv4_phsh = data2
		RorFrNpRdiv4_25to18_phsh = data3 >> 18
		temp = data3 % (2 ** 18)
		RorFrNpRdiv4_17to10_phsh = temp >> 10
		temp = data3 % (2 ** 10)
		RorFrNpRdiv4_9to2_phsh = temp >> 2
		RorFrNpRdiv4_1to0_phsh = data3 % (2 ** 2)

		return (Ngt_phsh, NorNdiv4_phsh, RorFrNpRdiv4_25to18_phsh, RorFrNpRdiv4_17to10_phsh, RorFrNpRdiv4_9to2_phsh, RorFrNpRdiv4_1to0_phsh)

	def set_rx_freq(self, target_freq):
		#
		#@param target_freq: desired receiver frequency in Hz
		#@returns (ok, actual_baseband_freq) where:
		#   ok is True or False and indicates success or failure,
		#   actual_baseband_freq is the RF frequency that corresponds to DC in the IF.
		#

		# Go through Quadrature Generation Initialization Sequence

		#target_freq = target_freq + 4000000

		if (target_freq <= 500000000):
			# Below 500 MHz
			print 'Below 500 MHz, divide by 2'
			# Use QuIET frequency divided by 2
			# Step 1
			self.X1sel_32to39_3 = 0
			self.X1sel_40to47_3 = 62
			self.X2sel_32to36_3 = 0
			self.X2sel_37to41_3 = 0
			self.X4sel_32to36_3 = 0
			self.X4sel_37to41_3 = 0

			self.set_reg_139()
			self.set_reg_140()
			self.set_reg_141()
			self.set_reg_142()
			self.set_reg_143()
			self.set_reg_144()

			# Step 2
			self.X1sel_40to47_3 = 63

			self.set_reg_140()

			try_freq = target_freq * 2

		elif ((target_freq > 500000000) and (target_freq <= 1000000000)):
			# Between 500 MHz and 1 GHz
			print 'Between 500 MHz and 1 GHz'
			# Use QuIET frequency
			# Step 1
			self.X1sel_32to39_3 = 1
			self.X1sel_40to47_3 = 192
			self.X2sel_32to36_3 = 0
			self.X2sel_37to41_3 = 0
			self.X4sel_32to36_3 = 0
			self.X4sel_37to41_3 = 0

			self.set_reg_139()
			self.set_reg_140()
			self.set_reg_141()
			self.set_reg_142()
			self.set_reg_143()
			self.set_reg_144()

			# Step 2
			self.X1sel_32to39_3 = 73

			self.set_reg_139()

			# Step 3
			self.X1sel_32to39_3 = 201

			self.set_reg_139()

			try_freq = target_freq

			# Set Quadrature Generator Charge/Discharge Taps
			self.DischargeTap16_0to3_3 = 6
			self.ChargeTap16_0to3_3 = 7
			self.DischargeTapn_0to3_3 = 0
			self.ChargeTapn16_0to3_3 = 5

			# Set Quadrature Generator Delays
			self.Qg00degDelay_0to4_3 = 16
			self.Qg90degDelay_0to4_3 = 31
			self.Qg180degDelay_0to4_3 = 0
			self.Qg270degDelay_0to4_3 = 31

			self.set_reg_133()
			self.set_reg_134()
			self.set_reg_135()
			self.set_reg_136()
			self.set_reg_137()
			self.set_reg_138()

		elif ((target_freq > 1000000000) and (target_freq <= 2000000000)):
			# Between 1 GHz and 2 GHz
			print 'Between 1 GHz and 2 GHz, multiply by 2'
			# Use QuIET multiplied by 2
			# Step 1
			self.X1sel_32to39_3 = 0
			self.X1sel_40to47_3 = 0
			self.X2sel_32to36_3 = 0
			self.X2sel_37to41_3 = 7
			self.X4sel_32to36_3 = 0
			self.X4sel_37to41_3 = 0

			self.set_reg_139()
			self.set_reg_140()
			self.set_reg_141()
			self.set_reg_142()
			self.set_reg_143()
			self.set_reg_144()

			# Step 2
			self.X2sel_32to36_3 = 9

			self.set_reg_141()

			# Step 3
			self.X2sel_32to36_3 = 25

			self.set_reg_141()

			# Step 4
			self.X2sel_32to36_3 = 16

			self.set_reg_141()

			try_freq = target_freq / 2

			# Set Quadrature Generator Charge/Discharge Taps
			self.DischargeTap16_0to3_3 = 9
			self.ChargeTap16_0to3_3 = 3
			self.DischargeTapn_0to3_3 = 3
			self.ChargeTapn16_0to3_3 = 5

			# Set Quadrature Generator Delays
			self.Qg00degDelay_0to4_3 = 31
			self.Qg90degDelay_0to4_3 = 31
			self.Qg180degDelay_0to4_3 = 0
			self.Qg270degDelay_0to4_3 = 31

			self.set_reg_133()
			self.set_reg_134()
			self.set_reg_135()
			self.set_reg_136()
			self.set_reg_137()
			self.set_reg_138()

		elif ((target_freq > 2000000000) and (target_freq <= 4000000000)):
			# 2 to 4 GHz
			print 'From 2 to 4 GHz, multiply by 4'
			# Use QuIET frequency multiplied by 4
			# Step 1
			self.X1sel_32to39_3 = 0
			self.X1sel_40to47_3 = 0
			self.X2sel_32to36_3 = 0
			self.X2sel_37to41_3 = 0
			self.X4sel_32to36_3 = 0
			self.X4sel_37to41_3 = 7

			self.set_reg_139()
			self.set_reg_140()
			self.set_reg_141()
			self.set_reg_142()
			self.set_reg_143()
			self.set_reg_144()

			# Step 2
			self.X4sel_32to36_3 = 9

			self.set_reg_143()

			# Step 3
			self.X4sel_32to36_3 = 25

			self.set_reg_143()

			try_freq = target_freq / 4

			# Set Quadrature Generator Charge/Discharge Taps
			self.DischargeTap16_0to3_3 = 16
			self.ChargeTap16_0to3_3 = 0
			self.DischargeTapn_0to3_3 = 7
			self.ChargeTapn16_0to3_3 = 7

			# Set Quadrature Generator Delays
			self.Qg00degDelay_0to4_3 = 0
			self.Qg90degDelay_0to4_3 = 31
			self.Qg180degDelay_0to4_3 = 0
			self.Qg270degDelay_0to4_3 = 31

			self.set_reg_133()
			self.set_reg_134()
			self.set_reg_135()
			self.set_reg_136()
			self.set_reg_137()
			self.set_reg_138()

		elif (target_freq > 4000000000):
			# Above 4 GHz, doesn't work
			return (False, target_freq)

			# FIXME

			'''# Above 4 GHz
			print 'Above 4 GHz, multiply by 8'
			# Use QuIET frequency multiplied by 8
			# Step 1
			self.X1sel_32to39_3 = 0
			self.X1sel_40to47_3 = 0
			self.X2sel_32to36_3 = 0
			self.X2sel_37to41_3 = 0
			self.X4sel_32to36_3 = 0
			self.X4sel_37to41_3 = 0
			self.X8sel_32to36_3 = 0
			self.X8sel_41_3 = 0
			self.X8sel_37to40_3 = 7

			self.set_reg_139()
			self.set_reg_140()
			self.set_reg_141()
			self.set_reg_142()
			self.set_reg_143()
			self.set_reg_144()
			self.set_reg_145()
			self.set_reg_146()

			# Step 2
			self.X8sel_32to36_3 = 9

			self.set_reg_145()

			# Step 3
			self.X8sel_32to36_3 = 25

			self.set_reg_145()

			try_freq = target_freq / 8

			# Set Quadrature Generator Charge/Discharge Taps
			self.ChargeTap16_0to3_3 = 15
			self.ChargeTapn16_0to3_3 = 15

			self.DischargeTap16_0to3_3 = 6
			self.DischargeTapn16_0to3_3 = 4

			self.set_reg_137()
			self.set_reg_138()'''


		self.Foutrx = target_freq

		(self.Ngt3_3, self.NorNdiv4_3, self.RorFrNpRdiv4_25to18_3, self.RorFrNpRdiv4_17to10_3, self.RorFrNpRdiv4_9to2_3, self.RorFrNpRdiv4_1to0_3) = self.calc_freq_vars(self.Fclk, try_freq)

		self.set_reg_104()
		self.set_reg_105()
		self.set_reg_106()
		self.set_reg_107()
		self.set_reg_108()

		return (1, target_freq)
		#FIXME -- How do I know if the RFIC successfully attained the desired frequency?#


	def set_tx_freq(self, target_freq):
		#
		#@param target_freq: desired transmitter frequency in Hz
		#@returns (ok, actual_baseband_freq) where:
		#   ok is True or False and indicates success or failure,
		#   actual_baseband_freq is the RF frequency that corresponds to DC in the IF.
		#

		# Go through Quadrature Generation Initialization Sequence

		# FIXME
		#target_freq = target_freq + 4000000
		#target_freq = target_freq + 1000000

		if (target_freq <= 500000000):
			print 'Below 500 MHz, divide by 2'
			# Use QuIET frequency divided by 2
			# Step 1
			self.X1sel_32to39 = 0
			self.X1sel_40to47 = 62
			self.X2sel_32to36 = 0
			self.X2sel_37to41 = 0
			self.X4sel_32to36 = 0
			self.X4sel_37to41 = 0

			self.set_reg_35()
			self.set_reg_36()
			self.set_reg_37()
			self.set_reg_38()
			self.set_reg_39()
			self.set_reg_40()

			# Step 2
			self.X1sel_40to47 = 63

			self.set_reg_36()

			try_freq = target_freq * 2

		elif ((target_freq > 500000000) and (target_freq <= 1000000000)):
			print 'Between 500 MHz and 1 GHz'
			# Use QuIET frequency
			# Step 1
			self.X1sel_32to39 = 1
			self.X1sel_40to47 = 192
			self.X2sel_32to36 = 0
			self.X2sel_37to41 = 0
			self.X4sel_32to36 = 0
			self.X4sel_37to41 = 0

			self.set_reg_35()
			self.set_reg_36()
			self.set_reg_37()
			self.set_reg_38()
			self.set_reg_39()
			self.set_reg_40()

			# Step 2
			self.X1sel_32to39 = 73

			self.set_reg_35()

			# Step 3
			self.X1sel_32to39 = 201

			self.set_reg_35()

			try_freq = target_freq

			# Set Quadrature Generator Charge/Discharge Taps and Delays
			self.Qg00degDelay_0to4 = 15
			self.Qg90degDelay_0to4 = 12
			self.Qg180degDelay_0to4 = 3
			self.Qg270degDelay_0to4 = 12

			self.set_reg_29()
			self.set_reg_30()
			self.set_reg_31()
			self.set_reg_32()

			self.DischargeTap16_0to3 = 1
			self.ChargeTap16_0to3 = 8
			self.DischargeTapn_0to3 = 7
			self.ChargeTapn16_0to3 = 0

			self.set_reg_33()
			self.set_reg_34()

		elif ((target_freq > 1000000000) and (target_freq <= 2000000000)):
			print 'Between 1 GHz and 2 GHz, multiply by 2'
			# Use QuIET multiplied by 2
			# Step 1
			self.X1sel_32to39 = 0
			self.X1sel_40to47 = 0
			self.X2sel_32to36 = 0
			self.X2sel_37to41 = 7
			self.X4sel_32to36 = 0
			self.X4sel_37to41 = 0

			self.set_reg_35()
			self.set_reg_36()
			self.set_reg_37()
			self.set_reg_38()
			self.set_reg_39()
			self.set_reg_40()

			# Step 2
			self.X2sel_32to36 = 9

			self.set_reg_37()

			# Step 3
			self.X2sel_32to36 = 25

			self.set_reg_37()

			# Step 4
			#self.X2sel_32to36 = 16

			#self.set_reg_37()

			try_freq = target_freq / 2

			# Set Quadrature Generator Charge/Discharge Taps and Delays
			self.Qg00degDelay_0to4 = 7
			self.Qg90degDelay_0to4 = 8
			self.Qg180degDelay_0to4 = 7
			self.Qg270degDelay_0to4 = 5

			self.set_reg_29()
			self.set_reg_30()
			self.set_reg_31()
			self.set_reg_32()

			self.DischargeTap16_0to3 = 1
			self.ChargeTap16_0to3 = 13

			self.DischargeTapn_0to3 = 3
			self.ChargeTapn16_0to3 = 9

			self.set_reg_33()
			self.set_reg_34()

		elif ((target_freq > 2000000000) and (target_freq <= 4000000000)):
			print '2-4 GHz, multiply by 4'
			# Use QuIET frequency multiplied by 4
			# Step 1
			self.X1sel_32to39 = 0
			self.X1sel_40to47 = 0
			self.X2sel_32to36 = 0
			self.X2sel_37to41 = 0
			self.X4sel_32to36 = 0
			self.X4sel_37to41 = 7

			self.set_reg_35()
			self.set_reg_36()
			self.set_reg_37()
			self.set_reg_38()
			self.set_reg_39()
			self.set_reg_40()

			# Step 2
			self.X4sel_32to36 = 9

			self.set_reg_39()

			# Step 3
			self.X4sel_32to36 = 25

			self.set_reg_39()

			try_freq = target_freq / 4

			# Set Quadrature Generator Charge/Discharge Taps and Delays
			self.Qg00degDelay_0to4 = 0
			self.Qg90degDelay_0to4 = 17
			self.Qg180degDelay_0to4 = 15
			self.Qg270degDelay_0to4 = 20

			self.set_reg_29()
			self.set_reg_30()
			self.set_reg_31()
			self.set_reg_32()

			self.DischargeTap16_0to3 = 15
			self.ChargeTap16_0to3 = 0

			self.DischargeTapn_0to3 = 10
			self.ChargeTapn16_0to3 = 8

			self.set_reg_33()
			self.set_reg_34()

		elif (target_freq > 4000000000):
			# Above 4 GHz, doesn't work
			return (False, target_freq)

		self.Fouttx = target_freq

		(self.Ngt3, self.NorNdiv4, self.RorFrNpRdiv4_25to18, self.RorFrNpRdiv4_17to10, self.RorFrNpRdiv4_9to2, self.RorFrNpRdiv4_1to0) = self.calc_freq_vars(self.Fclk, try_freq)

		self.set_reg_0()
		self.set_reg_1()
		self.set_reg_2()
		self.set_reg_3()
		self.set_reg_4()

		return (1, target_freq)
		#FIXME -- How do I know if the RFIC successfully attained the desired frequency?#

	def set_fb_freq(self, target_freq):
		#
		#@param target_freq: desired transmitter frequency in Hz
		#@returns (ok, actual_baseband_freq) where:
		#   ok is True or False and indicates success or failure,
		#   actual_baseband_freq is the RF frequency that corresponds to DC in the IF.
		#

		# Go through Quadrature Generation Initialization Sequence

		if (target_freq <= 500000000):
			print 'Below 500 MHz, divide by 2'
			# Use QuIET frequency divided by 2
			# Step 1
			self.X1sel_32to39_2 = 0
			self.X1sel_40to47_2 = 62
			self.X2sel_32to36_2 = 0
			self.X2sel_37to41_2 = 0
			self.X4sel_32to36_2 = 0
			self.X4sel_37to41_2 = 0

			self.set_reg_83()
			self.set_reg_84()
			self.set_reg_85()
			self.set_reg_86()
			self.set_reg_87()
			self.set_reg_88()

			# Step 2
			self.X1sel_40to47_2 = 63

			self.set_reg_84()

			try_freq = target_freq * 2

		elif ((target_freq > 500000000) and (target_freq <= 1000000000)):
			print 'Between 500 MHz and 1 GHz'
			# Use QuIET frequency
			# Step 1
			self.X1sel_32to39_2 = 1
			self.X1sel_40to47_2 = 192
			self.X2sel_32to36_2 = 0
			self.X2sel_37to41_2 = 0
			self.X4sel_32to36_2 = 0
			self.X4sel_37to41_2 = 0

			self.set_reg_83()
			self.set_reg_84()
			self.set_reg_85()
			self.set_reg_86()
			self.set_reg_87()
			self.set_reg_88()

			# Step 2
			self.X1sel_32to39_2 = 73

			self.set_reg_83()

			# Step 3
			self.X1sel_32to39_2 = 201

			self.set_reg_83()

			try_freq = target_freq

			# Set Quadrature Generator Charge/Discharge Taps
			# FIXME
			#self.ChargeTap16_0to3_2 = 7
			#self.ChargeTapn16_0to3_2 = 5

			#self.DischargeTap16_0to3_2 = 6
			#self.DischargeTapn16_0to3_2 = 0

			#self.set_reg_81()
			#self.set_reg_82()

		elif ((target_freq > 1000000000) and (target_freq <= 2000000000)):
			print 'Between 1 GHz and 2 GHz, multiply by 2'
			# Use QuIET multiplied by 2
			# Step 1
			self.X1sel_32to39_2 = 0
			self.X1sel_40to47_2 = 0
			self.X2sel_32to36_2 = 0
			self.X2sel_37to41_2 = 7
			self.X4sel_32to36_2 = 0
			self.X4sel_37to41_2 = 0

			self.set_reg_83()
			self.set_reg_84()
			self.set_reg_85()
			self.set_reg_86()
			self.set_reg_87()
			self.set_reg_88()

			# Step 2
			self.X2sel_32to36_2 = 9

			self.set_reg_85()

			# Step 3
			self.X2sel_32to36_2 = 25

			# Step 4
			#self.X2sel_32to36 = 16

			self.set_reg_85()

			try_freq = target_freq / 2

			# Set Quadrature Generator Charge/Discharge Taps
			# FIXME
			#self.ChargeTap16_0to3_2 = 7
			#self.ChargeTapn16_0to3_2 = 8

			#self.DischargeTap16_0to3_2 = 15
			#self.DischargeTapn16_0to3_2 = 15

			#self.set_reg_81()
			#self.set_reg_82()

		elif ((target_freq > 2000000000) and (target_freq <= 4000000000)):
			print '2-4 GHz, multiply by 4'
			# Use QuIET frequency multiplied by 4
			# Step 1
			self.X1sel_32to39_2 = 0
			self.X1sel_40to47_2 = 0
			self.X2sel_32to36_2 = 0
			self.X2sel_37to41_2 = 0
			self.X4sel_32to36_2 = 0
			self.X4sel_37to41_2 = 7

			self.set_reg_83()
			self.set_reg_84()
			self.set_reg_85()
			self.set_reg_86()
			self.set_reg_87()
			self.set_reg_88()

			# Step 2
			self.X4sel_32to36_2 = 9

			self.set_reg_87()

			# Step 3
			self.X4sel_32to36_2 = 25

			self.set_reg_87()

			try_freq = target_freq / 4

			# Set Quadrature Generator Charge/Discharge Taps
			# FIXME
			#self.ChargeTap16_0to3_2 = 15
			#self.ChargeTapn16_0to3_2 = 15

			#self.DischargeTap16_0to3_2 = 6
			#self.DischargeTapn16_0to3_2 = 4

			#self.set_reg_81()
			#self.set_reg_82()

		elif (target_freq > 4000000000):
			# Above 4 GHz, doesn't work
			return (False, target_freq)

		self.Foutfb = target_freq

		(self.Ngt3_2, self.NorNdiv4_2, self.RorFrNpRdiv4_25to18_2, self.RorFrNpRdiv4_17to10_2, self.RorFrNpRdiv4_9to2_2, self.RorFrNpRdiv4_1to0_2) = self.calc_freq_vars(self.Fclk, try_freq)

		self.set_reg_48()
		self.set_reg_49()
		self.set_reg_50()
		self.set_reg_51()
		self.set_reg_52()

		return (1, target_freq)
		#FIXME -- How do I know if the RFIC successfully attained the desired frequency?#

	def set_rx_phase(self, phsh):
		#
		#@param phsh: desired phase shift in degrees
		#@returns (ok) where:
		#   ok is True or False and indicates success or failure
		#

		phsh = phsh % 360

		if (self.Foutrx <= 500000000):
			synth_freq = self.Foutrx * 2
		elif ( (self.Foutrx > 500000000) and (self.Foutrx <= 1000000000)):
			synth_freq = self.Foutrx
		elif ( (self.Foutrx > 1000000000) and (self.Foutrx < 2000000000)):
			synth_freq = self.Foutrx / 2
		elif (self.Foutrx > 2000000000):
			synth_freq = self.Foutrx / 4

		(self.Qu_tx_Ngt3_3, self.NorNdiv4_phsh_3, self.RorFrNpRdiv4_phsh_25to18_3, self.RorFrNpRdiv4_phsh_17to10_3, self.RorFrNpRdiv4_phsh_9to2_3, self.RorFrNpRdiv4_phsh_1to0_3) = self.calc_phase_vars(self.Fclk, synth_freq, phsh)

		self.set_reg_109()
		self.set_reg_110()
		self.set_reg_111()
		self.set_reg_112()
		self.set_reg_113()

		return (1)
		#FIXME -- How do I know if the RFIC successfully attained the desired phase?#


	def set_tx_phase(self, phsh):
		#
		#@param phsh: desired phase shift in degrees
		#@returns (ok) where:
		#   ok is True or False and indicates success or failure
		#

		phsh = phsh % 360

		if (self.Fouttx <= 500000000):
			synth_freq = self.Fouttx * 2
		elif ( (self.Fouttx > 500000000) and (self.Fouttx <= 1000000000)):
			synth_freq = self.Fouttx
		elif ( (self.Fouttx > 1000000000) and (self.Fouttx < 2000000000)):
			synth_freq = self.Fouttx / 2
		elif (self.Fouttx > 2000000000):
			synth_freq = self.Fouttx / 4

		(self.Qu_tx_Ngt3_3, self.NorNdiv4_phsh_3, self.RorFrNpRdiv4_phsh_25to18_3, self.RorFrNpRdiv4_phsh_17to10_3, self.RorFrNpRdiv4_phsh_9to2_3, self.RorFrNpRdiv4_phsh_1to0_3) = self.calc_phase_vars(self.Fclk, synth_freq, phsh)

		self.set_reg_5()
		self.set_reg_6()
		self.set_reg_7()
		self.set_reg_8()
		self.set_reg_9()

		#FIXME -- How do I know if the RFIC successfully attained the desired phase?#
		return (1)

	def set_fb_phase(self, phsh):
		#
		#@param phsh: desired phase shift in degrees
		#@returns (ok) where:
		#   ok is True or False and indicates success or failure
		#

		phsh = phsh % 360

		if (self.Foutfb <= 500000000):
			synth_freq = self.Foutfb * 2
		elif ( (self.Foutfb > 500000000) and (self.Foutfb <= 1000000000)):
			synth_freq = self.Foutfb
		elif ( (self.Foutfb > 1000000000) and (self.Foutfb < 2000000000)):
			synth_freq = self.Foutfb / 2
		elif (self.Foutfb > 2000000000):
			synth_freq = self.Foutfb / 4

		(self.Qu_tx_Ngt3_3, self.NorNdiv4_phsh_3, self.RorFrNpRdiv4_phsh_25to18_3, self.RorFrNpRdiv4_phsh_17to10_3, self.RorFrNpRdiv4_phsh_9to2_3, self.RorFrNpRdiv4_phsh_1to0_3) = self.calc_phase_vars(self.Fclk, synth_freq, phsh)

		self.set_reg_53()
		self.set_reg_54()
		self.set_reg_55()
		self.set_reg_56()
		self.set_reg_57()

		#FIXME -- How do I know if the RFIC successfully attained the desired phase?#
		return (1)

	def set_rx_bw(self, bw):
		#
		#@param bw: desired bandwidth in Hz
		#
		# Available bandwidth: 4.25 kHz to 14 MHz (baseband)
		# FIXME
		print 'Desired bandwidth: %s' % (bw)
		if bw <= 5250:
			# Set BW to 3.532 kHz
			self.rx_rfp = 3
			self.rx_cp_12to8 = 31
			self.rx_cp_7to0 = 240

			self.rx_rv = 7
			self.rx_cv_10to3 = 254
			self.rx_cv_2to0 = 0

			self.rx_rq = 7
			self.rx_cq_9to8 = 3
			self.rx_cq_7to0 = 240

		elif bw > 5250 and bw <= 10500:
			# Set BW to 7.065 kHz
			self.rx_rfp = 3
			self.rx_cp_12to8 = 31
			self.rx_cp_7to0 = 240

			self.rx_rv = 5
			self.rx_cv_10to3 = 254
			self.rx_cv_2to0 = 0

			self.rx_rq = 5
			self.rx_cq_9to8 = 3
			self.rx_cq_7to0 = 240

		elif bw > 10500 and bw <= 21000:
			# Set BW to 14.130 kHz
			self.rx_rfp = 2
			self.rx_cp_12to8 = 31
			self.rx_cp_7to0 = 240

			self.rx_rv = 4
			self.rx_cv_10to3 = 254
			self.rx_cv_2to0 = 0

			self.rx_rq = 4
			self.rx_cq_9to8 = 3
			self.rx_cq_7to0 = 240

		elif bw > 21000 and bw <= 42000:
			# Set BW to 28.259 kHz
			self.rx_rfp = 2
			self.rx_cp_12to8 = 15
			self.rx_cp_7to0 = 240

			self.rx_rv = 3
			self.rx_cv_10to3 = 254
			self.rx_cv_2to0 = 0

			self.rx_rq = 3
			self.rx_cq_9to8 = 3
			self.rx_cq_7to0 = 240

		elif bw > 42000 and bw <= 84500:
			# Set BW to 56.518 kHz
			self.rx_rfp = 2
			self.rx_cp_12to8 = 7
			self.rx_cp_7to0 = 240

			self.rx_rv = 2
			self.rx_cv_10to3 = 254
			self.rx_cv_2to0 = 0

			self.rx_rq = 2
			self.rx_cq_9to8 = 3
			self.rx_cq_7to0 = 240

		elif bw > 84500 and bw <= 169500:
			# Set BW to 113.036 kHz
			self.rx_rfp = 2
			self.rx_cp_12to8 = 3
			self.rx_cp_7to0 = 240

			self.rx_rv = 1
			self.rx_cv_10to3 = 254
			self.rx_cv_2to0 = 0

			self.rx_rq = 1
			self.rx_cq_9to8 = 3
			self.rx_cq_7to0 = 240

		elif bw > 169500 and bw <= 339000:
			# Set BW to 226.072 kHz
			self.rx_rfp = 2
			self.rx_cp_12to8 = 1
			self.rx_cp_7to0 = 240

			self.rx_rv = 1
			self.rx_cv_10to3 = 126
			self.rx_cv_2to0 = 0

			self.rx_rq = 1
			self.rx_cq_9to8 = 1
			self.rx_cq_7to0 = 240

		elif bw > 339000 and bw <= 667000:
			# Set BW to 452.145 kHz
			self.rx_rfp = 1
			self.rx_cp_12to8 = 1
			self.rx_cp_7to0 = 240

			self.rx_rv = 0
			self.rx_cv_10to3 = 254
			self.rx_cv_2to0 = 0

			self.rx_rq = 1
			self.rx_cq_9to8 = 0
			self.rx_cq_7to0 = 240

		elif bw > 667000 and bw <= 1356000:
			# Set BW to 904.289 kHz
			self.rx_rfp = 1
			self.rx_cp_12to8 = 0
			self.rx_cp_7to0 = 240

			self.rx_rv = 0
			self.rx_cv_10to3 = 126
			self.rx_cv_2to0 = 0

			self.rx_rq = 0
			self.rx_cq_9to8 = 3
			self.rx_cq_7to0 = 240

		elif bw > 1356000 and bw <= 2712500:
			# Set BW to 1808.579 kHz
			self.rx_rfp = 1
			self.rx_cp_12to8 = 0
			self.rx_cp_7to0 = 112

			self.rx_rv = 0
			self.rx_cv_10to3 = 62
			self.rx_cv_2to0 = 0

			self.rx_rq = 0
			self.rx_cq_9to8 = 1
			self.rx_cq_7to0 = 240

		elif bw > 2712500 and bw <= 5425500:
			# Set BW to 3617.157 kHz
			self.rx_rfp = 0
			self.rx_cp_12to8 = 0
			self.rx_cp_7to0 = 112

			self.rx_rv = 0
			self.rx_cv_10to3 = 30
			self.rx_cv_2to0 = 0

			self.rx_rq = 0
			self.rx_cq_9to8 = 0
			self.rx_cq_7to0 = 240

		elif bw > 5425500 and bw <= 10851000:
			# Set BW to 7234.315 kHz
			self.rx_rfp = 0
			self.rx_cp_12to8 = 0
			self.rx_cp_7to0 = 48

			self.rx_rv = 0
			self.rx_cv_10to3 = 14
			self.rx_cv_2to0 = 0

			self.rx_rq = 0
			self.rx_cq_9to8 = 0
			self.rx_cq_7to0 = 112

		elif bw > 10851000:
			# Set BW to 14468.630 kHz
			self.rx_rfp = 0
			self.rx_cp_12to8 = 0
			self.rx_cp_7to0 = 16

			self.rx_rv = 0
			self.rx_cv_10to3 = 6
			self.rx_cv_2to0 = 0

			self.rx_rq = 0
			self.rx_cq_9to8 = 0
			self.rx_cq_7to0 = 48

		self.set_reg_198()
		self.set_reg_199()
		self.set_reg_200()
		self.set_reg_201()
		self.set_reg_202()
		self.set_reg_203()
		self.set_reg_204()


	def set_tx_bw(self, bw):
		#
		#@param bw: desired bandwidth in Hz
		#
		# Available bandwidth: 6.25 kHz to 14+ MHz (baseband)
		# FIXME
		print 'Desired bandwidth: %s' % (bw)
		if bw <= 20000:
			# Set BW to 12.5 kHz
			self.tx_p1_bw = 3
			self.tx_p2_bw2 = 15

		elif bw > 20000 and bw <= 37500:
			# Set BW to 25 kHz
			self.tx_p1_bw = 3
			self.tx_p2_bw2 = 7
			
		elif bw > 37500 and bw <= 75000:
			# Set BW to 50 kHz
			self.tx_p1_bw = 3
			self.tx_p2_bw2 = 3

		elif bw > 75000 and bw <= 150000:
			# Set BW to 100 kHz
			self.tx_p1_bw = 3
			self.tx_p2_bw2 = 1

		elif bw > 150000 and bw <= 425000:
			# Set BW to 200 kHz
			self.tx_p1_bw = 3
			self.tx_p2_bw2 = 0

		elif bw > 425000 and bw <= 1125000:
			# Set BW to 750 kHz
			self.tx_p1_bw = 1
			self.tx_p2_bw2 = 15

		elif bw > 1125000 and bw <= 2250000:
			# Set BW to 1.5 MHz
			self.tx_p1_bw = 1
			self.tx_p2_bw2 = 7

		elif bw > 2250000 and bw <= 4500000:
			# Set BW to 3 MHz
			self.tx_p1_bw = 1
			self.tx_p2_bw2 = 3

		elif bw > 4500000 and bw <= 9000000:
			# Set BW to 6 MHz
			self.tx_p1_bw = 1
			self.tx_p2_bw2 = 1

		elif bw > 9000000 and bw <= 13000000:
			# Set BW to 12 MHz
			self.tx_p1_bw = 1
			self.tx_p2_bw2 = 0

		elif bw > 13000000:
			# Set BW to 14+ MHz
			self.tx_p1_bw = 0
			self.tx_p2_bw2 = 0

		self.set_reg_173()
		self.set_reg_174()

	def set_fb_bw(self, bw):
		#
		#@param bw: desired bandwidth in Hz
		#
		# Available bandwidth: 5 MHz to 14+ MHz (baseband)
		# FIXME
		print 'Desired bandwidth: %s' % (bw)
		if bw <= 7500000:
			# Set BW to 5 MHz
			self.tx_bb_fdbk_bw = 3

		elif bw > 7500000 and bw <= 12000000:
			# Set BW to 10 MHz
			self.tx_bb_fdbk_bw = 1

		elif bw > 12000000:
			# Set BW to 14+ MHz
			self.tx_bb_fdbk_bw = 0

		self.set_reg_156()

	def enable_tx_fb(self):
		#
		# Enable transmitter feedback to RX port for DC offset correction, etc.
		#
		# FIXME
		print 'Enabling Transmit Feedback'

		# Disable RX Filter
		self.rx_foe = 0
		self.set_reg_196()

		# Enable Baseband Feedback, TX I and Q via RX I and Q
		self.tx_bb_fdbk_en = 3
		self.set_reg_157()

		# Disable Baseband Feedback Calibration
		# FIXME
		#self.tx_bb_fdbk_cal_en = 0

		# Enable Baseband Feedback Cartesian Forward Path
		self.tx_bb_fdbk_cart_fwd_en = 1
		self.set_reg_156()

		# Enable Cartesian Feedback Path
		self.tx_cart_en = 1
		self.set_reg_160()

		# Enable Cartesian Feedback
		self.CartesianFeedbackpathenable = 1

		# Enable Cartesian Feedback Path DCOC
		self.CartesianFeedbackpathDCOCenable = 1
		self.set_reg_166()

		# Set Cartesian Feedback Path Amplifier Gain
		self.CartesianFBpathAmp1Gain = 0
		self.CartesianFBpathAmp2Gain = 0
		self.CartesianFBpathAmp3Gain = 0
		self.CartesianFBpathAmp4Gain = 0
		self.set_reg_169()

		# Enable Cartesian Feedback Path Zero
		self.CartesianFBpathZeroEnable = 1
		self.set_reg_170()

	def disable_tx_fb(self):
		#
		# Disable transmitter feedback to RX port
		#
		# FIXME
		print 'Disabling Transmit Feedback'

		# Enable RX Filter
		self.rx_foe = 1
		self.set_reg_196()

		# Disable Baseband Feedback
		self.tx_bb_fdbk_en = 0
		self.set_reg_157()

		# Enable Baseband Feedback Calibration
		# FIXME
		#self.tx_bb_fdbk_cal_en = 1

		# Disable Baseband Feedback Cartesian Forward Path
		self.tx_bb_fdbk_cart_fwd_en = 0
		self.set_reg_156()

		# Disable Cartesian Feedback Path
		self.tx_cart_en = 0
		self.set_reg_160()

		# Disable Cartesian Feedback
		self.CartesianFeedbackpathenable = 0

		# Disable Cartesian Feedback Path DCOC
		self.CartesianFeedbackpathDCOCenable = 0
		self.set_reg_166()

		# Set Cartesian Feedback Path Amplifier Gain
		self.CartesianFBpathAmp1Gain = 3
		self.CartesianFBpathAmp2Gain = 3
		self.CartesianFBpathAmp3Gain = 3
		self.CartesianFBpathAmp4Gain = 3
		self.set_reg_169()

		# Disable Cartesian Feedback Path Zero
		self.CartesianFBpathZeroEnable = 0
		self.set_reg_170()

	def RSSI(self):
		# Return fade, clip from the two RX-side ADCs.
		#@returns fade, clip
		# variables proportional to how much fading (low signal strength)
		# or clipping (high signal strength) is going on

		# Turn off test mux
		self.TestMuxBufferEnable = 0 #Disable Test Mux Buffer
		self.TestMuxEnable = 0 #Disable Test Mux
		self.TestMuxSetting = 0 #Four Output Description (Test1, Test2, Test3, Test4)
		self.set_reg_222()

		# Turn on on-channel detectors
		# Off-channel doesn't work - leave it off
		self.rx_onchen = 1 #Enables on-channel detector.
		self.rx_offchen = 0 #Disables off-channel detector
		self.set_reg_196()

		# Set clip and fade thresholds
		self.rx_offch = 1 #Sets the Clip Threshold for the Off-channel Detector
		self.rx_onchf = 0 #Sets the Fade Threshold for the On-channel Detector relative to the On-channel clip point.
		self.rx_onchc = 2 #Sets the Clip Threshold for the On-channel Detector
		self.set_reg_197()

		fade = self.u.read_aux_adc(self.which, 0)
		clip = self.u.read_aux_adc(self.which, 1)
		return (fade, clip)

class db_rfic_base(db_base.db_base):
	#
	#Abstract base class for all RFIC boards.
	#
	#Derive board specific subclasses from db_rfic_base_{tx,rx}
	#

	def __init__(self, usrp, which):
		#
		#@param usrp: instance of usrp.source_c
		#@param which: which side: 0 or 1 corresponding to side A or B respectively
		#@type which: int
		#

		# sets _u  _which _tx and _slot
		db_base.db_base.__init__(self, usrp, which)
		self.rfic = _get_or_make_rfic(usrp, which)

	def __del__(self):

		#FIXME#
		return True

	def is_quadrature(self):
		#
		#Return True if this board requires both I and Q analog channels.
		#
		#This bit of info is useful when setting up the USRP Rx mux register.
		#
		return True

	def freq_range(self):
		# Return frequency range of RFIC daughterboard
		#FIXME#
		return (1e8, 2.5e6, 1e3)

	# ----------------------------------------------------------------------------------

class db_rfic_tx(db_rfic_base):
	def __init__(self, usrp, which):
		#
		#@param usrp: instance of usrp.sink_c
		#@param which: 0 or 1 corresponding to side TX_A or TX_B, respectively.
		#
		print "db_rfic_tx: __init__"
		db_rfic_base.__init__(self, usrp, which)

		# FIXME
		self.rfic.set_reg_0()
		self.rfic.set_reg_1()
		self.rfic.set_reg_2()
		self.rfic.set_reg_3()
		self.rfic.set_reg_4()
		self.rfic.set_reg_5()
		self.rfic.set_reg_6()
		self.rfic.set_reg_7()
		self.rfic.set_reg_8()
		self.rfic.set_reg_9()
		self.rfic.set_reg_10()
		self.rfic.set_reg_12()
		self.rfic.set_reg_13()
		self.rfic.set_reg_14()
		self.rfic.set_reg_15()
		self.rfic.set_reg_16()
		self.rfic.set_reg_17()
		self.rfic.set_reg_18()
		self.rfic.set_reg_19()
		self.rfic.set_reg_20()
		self.rfic.set_reg_21()
		self.rfic.set_reg_22()
		self.rfic.set_reg_23()
		self.rfic.set_reg_24()
		self.rfic.set_reg_29()
		self.rfic.set_reg_30()
		self.rfic.set_reg_31()
		self.rfic.set_reg_32()
		self.rfic.set_reg_33()
		self.rfic.set_reg_34()
		self.rfic.set_reg_35()
		self.rfic.set_reg_36()
		self.rfic.set_reg_37()
		self.rfic.set_reg_38()
		self.rfic.set_reg_39()
		self.rfic.set_reg_40()
		self.rfic.set_reg_41()
		self.rfic.set_reg_42()
		self.rfic.set_reg_43()
		self.rfic.set_reg_156()
		self.rfic.set_reg_157()
		self.rfic.set_reg_158()
		self.rfic.set_reg_159()
		self.rfic.set_reg_160()
		self.rfic.set_reg_161()
		self.rfic.set_reg_162()
		self.rfic.set_reg_163()
		self.rfic.set_reg_164()
		self.rfic.set_reg_165()
		self.rfic.set_reg_166()
		self.rfic.set_reg_167()
		self.rfic.set_reg_168()
		self.rfic.set_reg_169()
		self.rfic.set_reg_170()
		self.rfic.set_reg_171()
		self.rfic.set_reg_172()
		self.rfic.set_reg_173()
		self.rfic.set_reg_174()
		self.rfic.set_reg_175()
		self.rfic.set_reg_176()
		self.rfic.set_reg_177()
		self.rfic.set_reg_178()
		self.rfic.set_reg_179()
		self.rfic.set_reg_180()
		self.rfic.set_reg_181()

		# Get digital block out of digital reset state
		self.rfic.Rst_n_async = 1
		self.rfic.set_reg_24()

		# Turn on forward baseband reference section
		self.rfic.tx_bb_en = 1
		# FIXME
		#self.rfic.set_reg_156()

		# Unroutes the Cartesian error signal through the BB Correction feedback
		# FIXME
		self.rfic.tx_bb_fdbk_cart_err_en = 0

		# Routes the Cartesian feedback signal through the BB Correction feedback
		# FIXME
		self.rfic.tx_bb_fdbk_cart_fb_en = 1
		self.rfic.set_reg_156()

		# Turn on baseband feedback section
		# FIXME
		#self.rfic.tx_bb_fdbk_en = 3
		#self.rfic.set_reg_157()

		# Turn on forward RF transmit path
		self.rfic.RFForwardPathEnable_toMUX = 1
		self.rfic.set_reg_175()

		# Turn on Cartesian FB path switch to forward summer
		self.rfic.CartesianFBpathSwitchtoforwardSummer = 1
		self.rfic.set_reg_168()

		# Turn on Cartesian zero
		self.CartesianFBpathZeroEnable = 1
		self.rfic.set_reg_170()

		# Select TX output path, default tx1
		# FIXME
		self.rfic.tx_output_channel_sel = 2
		#self.rfic.tx_output_channel_sel = 1
		self.rfic.set_reg_172()

		# Set TX Channel 1 Gain
		# The gain control on TX channel 1 is controlled by this DAC
		# The maximum voltage is 2.2 volts, which corresponds to 2750
		# This controls about 35 dB of gain ONLY ON TX 1
		self.rfic.u.write_aux_dac(self.rfic.which, 3, 2750)


		# POR On.  This enables the clock that drives the digital block (which provides the tap selection process).  It must be enabled to generate an output.  See Byp_fine, address 10, bit 6
		self.rfic.Clk_driver_en = 1

		# POR On
		self.rfic.qu_reg_en = 1

		# POR On
		self.rfic.qq_reg_en = 1

		# POR Off
		self.rfic.win_rst = 0

		# POR On
		self.rfic.fineEn = 0

		# POR Off
		self.rfic.fineEnb = 1

		# POR On
		#self.rfic.rsffEn = 0

		# POR On
		self.rfic.dl_en = 1

		# POR On
		self.rfic.cp_en = 1

		self.rfic.set_reg_20()
		self.rfic.set_reg_21()

	def __del__(self):
		# print "rfic_base_tx.__del__"
		# Power down

		# Turn off output channel
		self.rfic.tx_output_channel_sel = 0
		self.rfic.set_reg_172()

		# Turn off forward RF transmit path
		self.rfic.RFForwardPathEnable_toMUX = 0
		self.rfic.set_reg_17()

		# Turn off forward baseband reference section
		self.rfic.tx_bb_en = 0
		self.rfic.set_reg_156()

		# Unroutes the Cartesian error signal through the BB Correction feedback
		# FIXME
		self.rfic.tx_bb_fdbk_cart_err_en = 0

		# Unroutes the Cartesian feedback signal through the BB Correction feedback
		self.rfic.tx_bb_fdbk_cart_fb_en = 0
		self.rfic.set_reg_156()

		# Turn off Cartesian FB path switch to forward summer
		self.rfic.CartesianFBpathSwitchtoforwardSummer = 0
		self.rfic.set_reg_168()

		# Turn off Cartesian zero
		self.CartesianFBpathZeroEnable = 0
		self.rfic.set_reg_170()

		# Turn off baseband feedback section
		# FIXME
		#self.rfic.tx_bb_fdbk_en = 0
		#self.rfic.set_reg_157()

		# POR Off.  This enables the clock that drives the digital block (which provides the tap selection process).  It must be enabled to generate an output.  See Byp_fine, address 10, bit 6
		self.rfic.Clk_driver_en = 0

		# POR Off
		self.rfic.qu_reg_en = 0

		# POR Off
		self.rfic.qq_reg_en = 0

		# POR Off
		self.rfic.win_rst = 0

		# POR Off
		self.rfic.fineEn = 0

		# POR Off
		self.rfic.fineEnb = 0

		# POR Off
		#self.rfic.rsffEn = 0

		# POR Off
		self.rfic.dl_en = 0

		# POR Off
		self.rfic.cp_en = 0

		self.rfic.set_reg_20()
		self.rfic.set_reg_21()

		# Put digital block in digital reset state
		self.rfic.Rst_n_async = 0
		self.rfic.set_reg_24()

		db_rfic_base.__del__(self)

	def select_tx_antenna(self, which_antenna):
		#
		#Specify which antenna port to use for transmission.
		#@param which_antenna: either 'tx1', 'tx2' or 'tx3'
		#
		if which_antenna in (0, 'tx1'):
			self.rfic.tx_output_channel_sel = 1
			self.rfic.set_reg_172()
		elif which_antenna in (1, 'tx2'):
			self.rfic.tx_output_channel_sel = 2
			self.rfic.set_reg_172()
		elif which_antenna in (2, 'tx3'):
			self.rfic.tx_output_channel_sel = 4
			self.rfic.set_reg_172()
		else:
			raise ValueError, "which_antenna must be either 'tx1', 'tx2' or 'tx3'"

	def gain_range(self):
		# Gain range for transmitter, in dB, 0 to 45 in increments of 5 dB
		return (0.0, 45.0, 5)

	def set_gain(self, gain):
		# Set transmit gain, in dB
		return self.rfic.set_tx_gain(gain)

	def set_freq(self, target_freq):
		# Set transmit frequency, in Hz
		return self.rfic.set_tx_freq(target_freq)

	def set_phase(self, phase):
		# Set transmit phase offset, in degrees
		return self.rfic.set_tx_phase(phase)

	def set_bw(self, bw):
		# Set transmit bandwidth, in Hz
		return self.rfic.set_tx_bw(bw)

	def spectrum_inverted(self):
		# FIXME
		# Return True if the dboard gives an inverted spectrum
		return True
		#return False

class db_rfic_rx(db_rfic_base):
	def __init__(self, usrp, which):
		#
		#@param usrp: instance of usrp.sink_c
		#@param which: 0 or 1 corresponding to side TX_A or TX_B, respectively.
		#
		print "db_rfic_rx: __init__"
		db_rfic_base.__init__(self, usrp, which)

		# FIXME
		self.rfic.set_reg_48()
		self.rfic.set_reg_49()
		self.rfic.set_reg_50()
		self.rfic.set_reg_51()
		self.rfic.set_reg_52()
		self.rfic.set_reg_53()
		self.rfic.set_reg_54()
		self.rfic.set_reg_55()
		self.rfic.set_reg_56()
		self.rfic.set_reg_57()
		self.rfic.set_reg_58()
		self.rfic.set_reg_60()
		self.rfic.set_reg_61()
		self.rfic.set_reg_62()
		self.rfic.set_reg_63()
		self.rfic.set_reg_64()
		self.rfic.set_reg_65()
		self.rfic.set_reg_66()
		self.rfic.set_reg_67()
		self.rfic.set_reg_68()
		self.rfic.set_reg_69()
		self.rfic.set_reg_70()
		self.rfic.set_reg_71()
		self.rfic.set_reg_72()
		self.rfic.set_reg_77()
		self.rfic.set_reg_78()
		self.rfic.set_reg_79()
		self.rfic.set_reg_80()
		self.rfic.set_reg_81()
		self.rfic.set_reg_82()
		self.rfic.set_reg_83()
		self.rfic.set_reg_84()
		self.rfic.set_reg_85()
		self.rfic.set_reg_86()
		self.rfic.set_reg_87()
		self.rfic.set_reg_88()
		self.rfic.set_reg_89()
		self.rfic.set_reg_90()
		self.rfic.set_reg_91()
		self.rfic.set_reg_96()
		self.rfic.set_reg_97()
		self.rfic.set_reg_98()
		self.rfic.set_reg_99()
		self.rfic.set_reg_104()
		self.rfic.set_reg_105()
		self.rfic.set_reg_106()
		self.rfic.set_reg_107()
		self.rfic.set_reg_108()
		self.rfic.set_reg_109()
		self.rfic.set_reg_110()
		self.rfic.set_reg_111()
		self.rfic.set_reg_112()
		self.rfic.set_reg_113()
		self.rfic.set_reg_114()
		self.rfic.set_reg_116()
		self.rfic.set_reg_117()
		self.rfic.set_reg_118()
		self.rfic.set_reg_119()
		self.rfic.set_reg_120()
		self.rfic.set_reg_121()
		self.rfic.set_reg_122()
		self.rfic.set_reg_123()
		self.rfic.set_reg_124()
		self.rfic.set_reg_125()
		self.rfic.set_reg_126()
		self.rfic.set_reg_127()
		self.rfic.set_reg_128()
		self.rfic.set_reg_133()
		self.rfic.set_reg_134()
		self.rfic.set_reg_135()
		self.rfic.set_reg_136()
		self.rfic.set_reg_137()
		self.rfic.set_reg_138()
		self.rfic.set_reg_139()
		self.rfic.set_reg_140()
		self.rfic.set_reg_141()
		self.rfic.set_reg_142()
		self.rfic.set_reg_143()
		self.rfic.set_reg_144()
		self.rfic.set_reg_145()
		self.rfic.set_reg_146()
		self.rfic.set_reg_147()
		self.rfic.set_reg_152()
		self.rfic.set_reg_153()
		self.rfic.set_reg_192()
		self.rfic.set_reg_193()
		self.rfic.set_reg_194()
		self.rfic.set_reg_195()
		self.rfic.set_reg_196()
		self.rfic.set_reg_197()
		self.rfic.set_reg_198()
		self.rfic.set_reg_199()
		self.rfic.set_reg_200()
		self.rfic.set_reg_201()
		self.rfic.set_reg_202()
		self.rfic.set_reg_203()
		self.rfic.set_reg_204()
		self.rfic.set_reg_205()
		self.rfic.set_reg_206()
		self.rfic.set_reg_207()

		# Get digital block out of digital reset state
		self.rfic.Rst_n_async_3 = 1
		self.rfic.set_reg_128()

		# Set RX LNA port to LNA1 (SGO non-chopping mixer)
		# FIXME
		#self.rfic.rx_lna = 1
		self.rfic.rx_lna = 5

		# Set LNA bias
		self.rfic.rx_lnab = 1

		# Enable LO clock to mixer
		self.rfic.rx_rxchen = 1

		self.rfic.set_reg_205()

		# Enable RX Filter
		self.rfic.rx_fen = 1

		# Enable baseband filter chopper clock
		self.rfic.rx_chcken = 1

		# Enable chopper clock to all mixers
		self.rfic.rx_cen = 7

		# Set chopper divide setting
		# FIXME
		#self.rfic.rx_chck = 0
		self.rfic.rx_chck = 1

		self.rfic.set_reg_195()

		# Enable filter output
		self.rfic.rx_foe = 1

		# Enable on-channel detector
		#self.rfic.rx_onchen = 1

		# Enable off-channel detector
		#self.rfic.rx_offchen = 1


		self.rfic.set_reg_196()

		# Set BQ filter Q to 1.33
		self.rfic.rx_qs = 2

		# Set BQ resistor value to 1.4 kohms
		self.rfic.rx_rq = 0

		self.rfic.set_reg_198()

		# Set VGA resistor value to 2.5 kohms
		self.rfic.rx_rv = 0

		# Set PMA Rf resistor to 5 kohms
		self.rfic.rx_rfp = 00

		self.rfic.set_reg_199()

		# Set compensation control
		self.rfic.rx_cc = 0

		self.rfic.set_reg_203()

		# Enable DCOC DAC
		self.rfic.rx_den = 1

		self.rfic.set_reg_192()

		# Enable DCOC comparator
		self.rfic.rx_cmpen = 1

		self.rfic.set_reg_193()

		# RC Tune enable
		# FIXME
		#self.rfic.rx_ten = 1
		self.rfic.rx_ten = 0

		# RC Tune ramp circuit enable
		# FIXME
		#self.rfic.rx_ren = 1
		self.rfic.rx_ren = 0

		# Select DCOC/RC Tune divider, divide by 8
		self.rfic.rx_dv = 3

		self.rfic.set_reg_194()

		# Enable DCOC
		self.rfic.rx_dcoc = 1

		self.rfic.set_reg_193()

		# POR On.  This enables the clock that drives the digital block (which provides the tap selection process).  It must be enabled to generate an output.  See Byp_fine, address 10, bit 6
		self.rfic.Clk_driver_en_3 = 1

		# POR On
		self.rfic.qu_reg_en_3 = 1

		# POR On
		self.rfic.qq_reg_en_3 = 1

		# POR Off
		self.rfic.win_rst_3 = 0

		# POR On
		self.rfic.fineEn_3 = 0 

		# POR Off
		self.rfic.fineEnb_3 = 1

		# POR Off
		#self.rfic.rsffEn_3 = 0

		# POR On
		self.rfic.dl_en_3 = 1

		# POR On
		self.rfic.cp_en_3 = 1

		self.rfic.set_reg_124()
		self.rfic.set_reg_125()


	def __del__(self):
		# print "rfic_base_rx.__del__"
		# Power down

		# Set RX LNA path (off)
		self.rfic.rx_lna = 0

		# Disable LO clock to mixer
		self.rfic.rx_rxchen = 0

		self.rfic.set_reg_205()

		# Disable RX Filter
		self.rfic.rx_fen = 0

		# Disable baseband filter chipper clock
		self.rfic.rx_chcken = 0

		# Disable chopper clock to all mixers
		self.rfic.rx_cen = 0

		self.rfic.set_reg_195()

		# Disable filter output
		self.rfic.rx_foe = 0

		# Disable on-channel detector
		self.rfic.rx_onchen = 0

		# Disable off-channel detector
		self.rfic.rx_offchen = 0

		self.rfic.set_reg_196()

		# Disable DCOC DAC
		self.rfic.rx_den = 0

		self.rfic.set_reg_192()

		# Disable DCOC comparator
		self.rfic.rx_cmpen = 0

		self.rfic.set_reg_193()

		# RC Tune disable
		self.rfic.rx_ten = 0

		# RC Tune ramp circuit disable
		self.rfic.rx_ren = 0

		self.rfic.set_reg_194()

		# Disable DCOC
		self.rfic.rx_dcoc = 0

		self.rfic.set_reg_193()

		# POR Off.  This enables the clock that drives the digital block (which provides the tap selection process).  It must be enabled to generate an output.  See Byp_fine, address 10, bit 6
		self.rfic.Clk_driver_en_3 = 0

		# POR Off
		self.rfic.qu_reg_en_3 = 0

		# POR Off
		self.rfic.qq_reg_en_3 = 0

		# POR Off
		self.rfic.win_rst_3 = 0

		# POR Off
		self.rfic.fineEn_3 = 0

		# POR Off
		self.rfic.fineEnb_3 = 0

		# POR Off
		#self.rfic.rsffEn_3 = 0

		# POR Off
		self.rfic.dl_en_3 = 0

		# POR Off
		self.rfic.cp_en_3 = 0

		self.rfic.set_reg_124()
		self.rfic.set_reg_125()

		# Put digital block into digital reset state
		self.rfic.Rst_n_async_3 = 0
		self.rfic.set_reg_58()

		db_rfic_base.__del__(self)

	def select_rx_antenna(self, which_antenna):
		#
		#Specify which antenna port to use for reception.
		#@param which_antenna: either 'LNA1', 'LNA2', 'LNA3', 'LNA4' or 'MIX5'
		#
		if which_antenna in (0, 'LNA1'):
			self.rfic.rx_lna = 1
			self.rfic.set_reg_205()
		elif which_antenna in (1, 'LNA2'):
			self.rfic.rx_lna = 2
			self.rfic.set_reg_205()
		elif which_antenna in (2, 'LNA3'):
			self.rfic.rx_lna = 3
			self.rfic.set_reg_205()
		elif which_antenna in (3, 'LNA4'):
			self.rfic.rx_lna = 4
			self.rfic.set_reg_205()
		elif which_antenna in (4, 'MIX5'):
			self.rfic.rx_lna = 5
			self.rfic.set_reg_205()
		else:
			raise ValueError, "which_antenna must be either 'LNA1', 'LNA2', 'LNA3', 'LNA4' or 'MIX5'"

	def gain_range(self):
		# Receiver gain range, in dB
		return (0.0, 38.0, 1)

	def set_gain(self, gain):
		# Set receiver gain, in dB
		return self.rfic.set_rx_gain(gain)

	def set_freq(self, target_freq):
		# Set receiver frequency, in Hz
		return self.rfic.set_rx_freq(target_freq)

	def set_phase(self, phase):
		# Set receiver phase offset, in degrees
		return self.rfic.set_rx_phase(phase)

	def set_bw(self, bw):
		# Set receiver bandwidth, in Hz
		return self.rfic.set_rx_bw(bw)

	def enable_fb(self):
		# Enable transmitter feedback to receiver for DC offset, etc.
		return self.rfic.enable_tx_fb()

	def disable_fb(self):
		# Disable transmitter feedback to receiver
		return self.rfic.disable_tx_fb()

	def fb_gain_range(self):
		# Feedback gain range, in dB
		# FIXME
		return (0.0, 40.0, 5)

	def set_fb_gain(self, gain):
		# Set feedback gain, in dB
		return self.rfic.set_fb_gain(gain)

	def set_fb_freq(self, target_freq):
		# Set feedback frequency, in Hz
		return self.rfic.set_fb_freq(target_freq)

	def set_fb_phase(self, phase):
		# Set feedback phase offset, in degrees
		return self.rfic.set_fb_phase(phase)

	def set_fb_bw(self, bw):
		# Set feedback bandwidth, in Hz
		return self.rfic.set_fb_bw(bw)

	def RSSI(self):
		# Get received signal strength indicators
		# Returns (fade, clip)
		# Fade is proportional to how often the signal is low
		# Clip is proportional to how often the signal is high
		return self.rfic.RSSI()


#------------------------------------------------------------    
# hook these daughterboard classes into the auto-instantiation framework
db_instantiator.add(usrp_dbid.RFIC_TX, lambda usrp, which : (db_rfic_tx(usrp, which),))
db_instantiator.add(usrp_dbid.RFIC_RX, lambda usrp, which : (db_rfic_rx(usrp, which),))

