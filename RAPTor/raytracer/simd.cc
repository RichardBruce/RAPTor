#include "simd.h"

const vfp_t vfp_zero   = vfp_t(0.0f);
const vfp_t vfp_one    = vfp_t(1.0f);
const vfp_t vfp_true   = vfp_t(bit_cast<unsigned, float>(0xffffffff));

const vfp_t index_to_mask_lut[SIMD_WIDTH]       = { vfp_t(0.0f,                                  0.0f,                                  0.0f,                                  0.0f),
                                                    vfp_t(bit_cast<unsigned, float>(0xffffffff), 0.0f,                                  0.0f,                                  0.0f),
                                                    vfp_t(bit_cast<unsigned, float>(0xffffffff), bit_cast<unsigned, float>(0xffffffff), 0.0f,                                  0.0f),
                                                    vfp_t(bit_cast<unsigned, float>(0xffffffff), bit_cast<unsigned, float>(0xffffffff), bit_cast<unsigned, float>(0xffffffff), 0.0f) };

const int   mask_to_index_lut[1 << SIMD_WIDTH]  = { 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3 };
