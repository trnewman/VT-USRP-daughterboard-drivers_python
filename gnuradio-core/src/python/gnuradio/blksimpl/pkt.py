#
# Copyright 2005,2006 Free Software Foundation, Inc.
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

from math import pi
from gnuradio import gr, packet_utils
import gnuradio.gr.gr_threading as _threading


# /////////////////////////////////////////////////////////////////////////////
#                   mod/demod with packets as i/o
# /////////////////////////////////////////////////////////////////////////////

class mod_pkts(gr.hier_block):
    """
    Wrap an arbitrary digital modulator in our packet handling framework.

    Send packets by calling send_pkt
    """
    def __init__(self, fg, modulator, access_code=None, msgq_limit=2, pad_for_usrp=True, use_whitener_offset=False):
        """
	Hierarchical block for sending packets

        Packets to be sent are enqueued by calling send_pkt.
        The output is the complex modulated signal at baseband.

	@param fg: flow graph
	@type fg: flow graph
        @param modulator: instance of modulator class (gr_block or hier_block)
        @type modulator: complex baseband out
        @param access_code: AKA sync vector
        @type access_code: string of 1's and 0's between 1 and 64 long
        @param msgq_limit: maximum number of messages in message queue
        @type msgq_limit: int
        @param pad_for_usrp: If true, packets are padded such that they end up a multiple of 128 samples
        @param use_whitener_offset: If true, start of whitener XOR string is incremented each packet
        
        See gmsk_mod for remaining parameters
        """
        self._modulator = modulator
        self._pad_for_usrp = pad_for_usrp
        self._use_whitener_offset = use_whitener_offset
        self._whitener_offset = 0
        
        if access_code is None:
            access_code = packet_utils.default_access_code
        if not packet_utils.is_1_0_string(access_code):
            raise ValueError, "Invalid access_code %r. Must be string of 1's and 0's" % (access_code,)
        self._access_code = access_code

        # accepts messages from the outside world
        self._pkt_input = gr.message_source(gr.sizeof_char, msgq_limit)
        fg.connect(self._pkt_input, self._modulator)
        gr.hier_block.__init__(self, fg, None, self._modulator)

    def send_pkt(self, payload='', eof=False):
        """
        Send the payload.

        @param payload: data to send
        @type payload: string
        """
        if eof:
            msg = gr.message(1) # tell self._pkt_input we're not sending any more packets
        else:
            # print "original_payload =", string_to_hex_list(payload)
            pkt = packet_utils.make_packet(payload,
                                           self._modulator.samples_per_symbol(),
                                           self._modulator.bits_per_symbol(),
                                           self._access_code,
                                           self._pad_for_usrp,
                                           self._whitener_offset)
            #print "pkt =", string_to_hex_list(pkt)
            msg = gr.message_from_string(pkt)
            if self._use_whitener_offset is True:
                self._whitener_offset = (self._whitener_offset + 1) % 16
                
        self._pkt_input.msgq().insert_tail(msg)



class demod_pkts(gr.hier_block):
    """
    Wrap an arbitrary digital demodulator in our packet handling framework.

    The input is complex baseband.  When packets are demodulated, they are passed to the
    app via the callback.
    """

    def __init__(self, fg, demodulator, access_code=None, callback=None, threshold=-1):
        """
	Hierarchical block for demodulating and deframing packets.

	The input is the complex modulated signal at baseband.
        Demodulated packets are sent to the handler.

	@param fg: flow graph
	@type fg: flow graph
        @param demodulator: instance of demodulator class (gr_block or hier_block)
        @type demodulator: complex baseband in
        @param access_code: AKA sync vector
        @type access_code: string of 1's and 0's
        @param callback:  function of two args: ok, payload
        @type callback: ok: bool; payload: string
        @param threshold: detect access_code with up to threshold bits wrong (-1 -> use default)
        @type threshold: int
	"""

        self._demodulator = demodulator
        if access_code is None:
            access_code = packet_utils.default_access_code
        if not packet_utils.is_1_0_string(access_code):
            raise ValueError, "Invalid access_code %r. Must be string of 1's and 0's" % (access_code,)
        self._access_code = access_code

        if threshold == -1:
            threshold = 12              # FIXME raise exception

        self._rcvd_pktq = gr.msg_queue()          # holds packets from the PHY
        self.correlator = gr.correlate_access_code_bb(access_code, threshold)

        self.framer_sink = gr.framer_sink_1(self._rcvd_pktq)
        fg.connect(self._demodulator, self.correlator, self.framer_sink)
        
        gr.hier_block.__init__(self, fg, self._demodulator, None)
        self._watcher = _queue_watcher_thread(self._rcvd_pktq, callback)


class _queue_watcher_thread(_threading.Thread):
    def __init__(self, rcvd_pktq, callback):
        _threading.Thread.__init__(self)
        self.setDaemon(1)
        self.rcvd_pktq = rcvd_pktq
        self.callback = callback
        self.keep_running = True
        self.start()


    def run(self):
        while self.keep_running:
            msg = self.rcvd_pktq.delete_head()
            ok, payload = packet_utils.unmake_packet(msg.to_string(), int(msg.arg1()))
            if self.callback:
                self.callback(ok, payload)
