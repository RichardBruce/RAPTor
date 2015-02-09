/* Standard headers */
#include <limits>

/* Boost headers */

/* Comon headers */
#include "common.h"


const float PI = 3.1415926535897932384626433f;

namespace raptor_raytracer
{
const float EPSILON         = 1.0e-3f;
const float FP_DELTA        = 32.0f;
const float FP_DELTA_SMALL  = 1.0e-4f;
const float MAX_DIST        = std::numeric_limits<float>::max();
const float EXP             = 2.7182818284590452353602875f;

/* Modulus 3 look up table for values less than 6 */
const unsigned  mod_3_lut[6]    = { 0, 1, 2, 0, 1, 2 };

/* LUT for sine and cosine value look up */
const static_cos<COS_LUT_SIZE> cos_lut;

/* Display related constants */
const int BPP        = 4;
const int DEPTH      = 32;


/**********************************************************
 This is an implementation of the Mersenne twister random
 number generator
**********************************************************/
static unsigned int mt[624];
static unsigned int mt_index = 625;

/* Seed the Mersenne twister */
inline void init_mersenne_twister(unsigned int seed)
{
    mt[0] = seed;
    for (int i = 1; i < 624; i++)
    {
         mt[i] = 1812433253 * (mt[i - 1] ^ (mt[i - 1]) >> 30) + i;
    }
}

/* Fill the array with 624 untempered numbers */
inline void generate_array_mersenne_twister()
{
    unsigned int y;

    /* Process the lower part of the array */
    for (int i = 0; i < 227; i++)
    {
        y = (0x80000000 & mt[i]) + (0x7FFFFFFF & mt[i + 1]);
        if (y & 0x1)
        {
            mt[i] = mt[i + 397] ^ ((y >> 1)) ^ (0x9908b0df);
        }
        else 
        {
            mt[i] = mt[i + 397] ^ (y >> 1);
        }
    }

    /* Process the upper part of the array ie/ After mod((i+397),624) as wrapped around */
    for (int i = 227; i < 624; i++)
    {
        y = (0x80000000 & mt[i]) + (0x7FFFFFFF & mt[i + 1]);
        if (y & 0x1)
        {
            mt[i] = mt[i - 227] ^ (( y>> 1)) ^ (0x9908b0df);
        }
        else 
        {
            mt[i] = mt[i - 227] ^ (y >>1 );
        }
    }
}
 
/* Temper and return a random number */
float gen_random_mersenne_twister() 
{
    unsigned int y;
    
    /* When the array has all been used generate 624 new numbers */
    if(mt_index >= 624)
    {
        if (mt_index == 625)
        {
            init_mersenne_twister(5489);
        }
        generate_array_mersenne_twister();
        mt_index = 0;
    }

    y  = mt[mt_index];
    mt_index++;
    y ^=  (y >> 11);
    y ^= ((y << 7 ) & 0x9d2c5680);
    y ^= ((y << 15) & 0xefc60000);
    y ^=  (y >> 18);
    return static_cast<float>(y) * (1.0f / 4294967296.0f);
}
}; /* namespace raptor_raytracer */
