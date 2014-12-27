#include "simd.h"

const vfp_t vfp_zero   = vfp_t(0.0);
const vfp_t vfp_one    = vfp_t(1.0);
const vfp_t vfp_true   = vfp_t(bit_cast<unsigned, fp_t>(0xffffffff));

const vfp_t index_to_mask_lut[SIMD_WIDTH]       = { vfp_t((fp_t)0.0,                            (fp_t)0.0,                            (fp_t)0.0,                            (fp_t)0.0),
                                                    vfp_t(bit_cast<unsigned, fp_t>(0xffffffff), (fp_t)0.0,                            (fp_t)0.0,                            (fp_t)0.0),
                                                    vfp_t(bit_cast<unsigned, fp_t>(0xffffffff), bit_cast<unsigned, fp_t>(0xffffffff), (fp_t)0.0,                            (fp_t)0.0),
                                                    vfp_t(bit_cast<unsigned, fp_t>(0xffffffff), bit_cast<unsigned, fp_t>(0xffffffff), bit_cast<unsigned, fp_t>(0xffffffff), (fp_t)0.0) };

const int   mask_to_index_lut[1 << SIMD_WIDTH]  = { 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3 };
