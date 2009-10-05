//
// This file is machine generated.  All edits will be overwritten
//
%{
#include <gr_add_cc.h>
#include <gr_add_const_cc.h>
#include <gr_add_const_ff.h>
#include <gr_add_const_ii.h>
#include <gr_add_const_sf.h>
#include <gr_add_const_ss.h>
#include <gr_add_const_vcc.h>
#include <gr_add_const_vff.h>
#include <gr_add_const_vii.h>
#include <gr_add_const_vss.h>
#include <gr_add_ff.h>
#include <gr_add_ii.h>
#include <gr_add_ss.h>
#include <gr_add_vcc.h>
#include <gr_add_vff.h>
#include <gr_add_vii.h>
#include <gr_add_vss.h>
#include <gr_and_bb.h>
#include <gr_and_ii.h>
#include <gr_and_ss.h>
#include <gr_argmax_fs.h>
#include <gr_argmax_is.h>
#include <gr_argmax_ss.h>
#include <gr_chunks_to_symbols_bc.h>
#include <gr_chunks_to_symbols_bf.h>
#include <gr_chunks_to_symbols_ic.h>
#include <gr_chunks_to_symbols_if.h>
#include <gr_chunks_to_symbols_sc.h>
#include <gr_chunks_to_symbols_sf.h>
#include <gr_divide_cc.h>
#include <gr_divide_ff.h>
#include <gr_divide_ii.h>
#include <gr_divide_ss.h>
#include <gr_integrate_cc.h>
#include <gr_integrate_ff.h>
#include <gr_integrate_ii.h>
#include <gr_integrate_ss.h>
#include <gr_max_ff.h>
#include <gr_max_ii.h>
#include <gr_max_ss.h>
#include <gr_moving_average_cc.h>
#include <gr_moving_average_ff.h>
#include <gr_moving_average_ii.h>
#include <gr_moving_average_ss.h>
#include <gr_multiply_cc.h>
#include <gr_multiply_const_cc.h>
#include <gr_multiply_const_ff.h>
#include <gr_multiply_const_ii.h>
#include <gr_multiply_const_ss.h>
#include <gr_multiply_const_vcc.h>
#include <gr_multiply_const_vff.h>
#include <gr_multiply_const_vii.h>
#include <gr_multiply_const_vss.h>
#include <gr_multiply_ff.h>
#include <gr_multiply_ii.h>
#include <gr_multiply_ss.h>
#include <gr_multiply_vcc.h>
#include <gr_multiply_vff.h>
#include <gr_multiply_vii.h>
#include <gr_multiply_vss.h>
#include <gr_mute_cc.h>
#include <gr_mute_ff.h>
#include <gr_mute_ii.h>
#include <gr_mute_ss.h>
#include <gr_noise_source_c.h>
#include <gr_noise_source_f.h>
#include <gr_noise_source_i.h>
#include <gr_noise_source_s.h>
#include <gr_not_bb.h>
#include <gr_not_ii.h>
#include <gr_not_ss.h>
#include <gr_or_bb.h>
#include <gr_or_ii.h>
#include <gr_or_ss.h>
#include <gr_packed_to_unpacked_bb.h>
#include <gr_packed_to_unpacked_ii.h>
#include <gr_packed_to_unpacked_ss.h>
#include <gr_peak_detector_fb.h>
#include <gr_peak_detector_ib.h>
#include <gr_peak_detector_sb.h>
#include <gr_sample_and_hold_bb.h>
#include <gr_sample_and_hold_ff.h>
#include <gr_sample_and_hold_ii.h>
#include <gr_sample_and_hold_ss.h>
#include <gr_sig_source_c.h>
#include <gr_sig_source_f.h>
#include <gr_sig_source_i.h>
#include <gr_sig_source_s.h>
#include <gr_sub_cc.h>
#include <gr_sub_ff.h>
#include <gr_sub_ii.h>
#include <gr_sub_ss.h>
#include <gr_unpacked_to_packed_bb.h>
#include <gr_unpacked_to_packed_ii.h>
#include <gr_unpacked_to_packed_ss.h>
#include <gr_vector_sink_b.h>
#include <gr_vector_sink_c.h>
#include <gr_vector_sink_f.h>
#include <gr_vector_sink_i.h>
#include <gr_vector_sink_s.h>
#include <gr_vector_source_b.h>
#include <gr_vector_source_c.h>
#include <gr_vector_source_f.h>
#include <gr_vector_source_i.h>
#include <gr_vector_source_s.h>
#include <gr_xor_bb.h>
#include <gr_xor_ii.h>
#include <gr_xor_ss.h>
%}

%include <gr_add_cc.i>
%include <gr_add_const_cc.i>
%include <gr_add_const_ff.i>
%include <gr_add_const_ii.i>
%include <gr_add_const_sf.i>
%include <gr_add_const_ss.i>
%include <gr_add_const_vcc.i>
%include <gr_add_const_vff.i>
%include <gr_add_const_vii.i>
%include <gr_add_const_vss.i>
%include <gr_add_ff.i>
%include <gr_add_ii.i>
%include <gr_add_ss.i>
%include <gr_add_vcc.i>
%include <gr_add_vff.i>
%include <gr_add_vii.i>
%include <gr_add_vss.i>
%include <gr_and_bb.i>
%include <gr_and_ii.i>
%include <gr_and_ss.i>
%include <gr_argmax_fs.i>
%include <gr_argmax_is.i>
%include <gr_argmax_ss.i>
%include <gr_chunks_to_symbols_bc.i>
%include <gr_chunks_to_symbols_bf.i>
%include <gr_chunks_to_symbols_ic.i>
%include <gr_chunks_to_symbols_if.i>
%include <gr_chunks_to_symbols_sc.i>
%include <gr_chunks_to_symbols_sf.i>
%include <gr_divide_cc.i>
%include <gr_divide_ff.i>
%include <gr_divide_ii.i>
%include <gr_divide_ss.i>
%include <gr_integrate_cc.i>
%include <gr_integrate_ff.i>
%include <gr_integrate_ii.i>
%include <gr_integrate_ss.i>
%include <gr_max_ff.i>
%include <gr_max_ii.i>
%include <gr_max_ss.i>
%include <gr_moving_average_cc.i>
%include <gr_moving_average_ff.i>
%include <gr_moving_average_ii.i>
%include <gr_moving_average_ss.i>
%include <gr_multiply_cc.i>
%include <gr_multiply_const_cc.i>
%include <gr_multiply_const_ff.i>
%include <gr_multiply_const_ii.i>
%include <gr_multiply_const_ss.i>
%include <gr_multiply_const_vcc.i>
%include <gr_multiply_const_vff.i>
%include <gr_multiply_const_vii.i>
%include <gr_multiply_const_vss.i>
%include <gr_multiply_ff.i>
%include <gr_multiply_ii.i>
%include <gr_multiply_ss.i>
%include <gr_multiply_vcc.i>
%include <gr_multiply_vff.i>
%include <gr_multiply_vii.i>
%include <gr_multiply_vss.i>
%include <gr_mute_cc.i>
%include <gr_mute_ff.i>
%include <gr_mute_ii.i>
%include <gr_mute_ss.i>
%include <gr_noise_source_c.i>
%include <gr_noise_source_f.i>
%include <gr_noise_source_i.i>
%include <gr_noise_source_s.i>
%include <gr_not_bb.i>
%include <gr_not_ii.i>
%include <gr_not_ss.i>
%include <gr_or_bb.i>
%include <gr_or_ii.i>
%include <gr_or_ss.i>
%include <gr_packed_to_unpacked_bb.i>
%include <gr_packed_to_unpacked_ii.i>
%include <gr_packed_to_unpacked_ss.i>
%include <gr_peak_detector_fb.i>
%include <gr_peak_detector_ib.i>
%include <gr_peak_detector_sb.i>
%include <gr_sample_and_hold_bb.i>
%include <gr_sample_and_hold_ff.i>
%include <gr_sample_and_hold_ii.i>
%include <gr_sample_and_hold_ss.i>
%include <gr_sig_source_c.i>
%include <gr_sig_source_f.i>
%include <gr_sig_source_i.i>
%include <gr_sig_source_s.i>
%include <gr_sub_cc.i>
%include <gr_sub_ff.i>
%include <gr_sub_ii.i>
%include <gr_sub_ss.i>
%include <gr_unpacked_to_packed_bb.i>
%include <gr_unpacked_to_packed_ii.i>
%include <gr_unpacked_to_packed_ss.i>
%include <gr_vector_sink_b.i>
%include <gr_vector_sink_c.i>
%include <gr_vector_sink_f.i>
%include <gr_vector_sink_i.i>
%include <gr_vector_sink_s.i>
%include <gr_vector_source_b.i>
%include <gr_vector_source_c.i>
%include <gr_vector_source_f.i>
%include <gr_vector_source_i.i>
%include <gr_vector_source_s.i>
%include <gr_xor_bb.i>
%include <gr_xor_ii.i>
%include <gr_xor_ss.i>