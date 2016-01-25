#include "picture_functions.h"
#include "simd.h"

namespace raptor_raytracer
{
void brightness_adjust(float *const data, const int size, const int brit)
{
    const vfp_t vbrit(brit);

    /* Vector process the data we can */
    for (int i = 0; i <= (size - SIMD_WIDTH); i += SIMD_WIDTH)
    {
        vfp_t pixel(&data[i]);
        pixel += vbrit;
        pixel.store(&data[i]);
    }

    /* Process tail data */
    for (int i = (size & ~(SIMD_WIDTH - 1)); i < size; ++i)
    {
        data[i] += brit;
    }
}

void brightness_scale(float *const data, const int size, const float brit)
{
    const vfp_t vbrit(brit);

    /* Vector process the data we can */
    for (int i = 0; i <= (size - SIMD_WIDTH); i += SIMD_WIDTH)
    {
        vfp_t pixel(&data[i]);
        pixel *= vbrit;
        pixel.store(&data[i]);
    }

    /* Process tail data */
    for (int i = (size & ~(SIMD_WIDTH - 1)); i < size; ++i)
    {
        data[i] *= brit;
    }
}

void contrast_adjust(float *const data, const int size, const int cont)
{
    const vfp_t vcont(cont);
    const vfp_t middle(0.5f);

    /* Vector process the data we can */
     for (int i = 0; i <= (size - SIMD_WIDTH); i += SIMD_WIDTH)
    {
        vfp_t pixel(&data[i]);
        const vfp_t dist(pixel - middle);
        pixel += (dist * vcont);
        pixel.store(&data[i]);
    }

    /* Process tail data */
    for (int i = (size & ~(SIMD_WIDTH - 1)); i < size; ++i)
    {
        const float dist = data[i] - 0.5f;
        data[i] += (dist * cont);
    }
}

const float pr = 0.299f;
const float pg = 0.587f;
const float pb = 0.114f;
//  The "change" parameter works like this:
//    0.0 creates a black-and-white image.
//    0.5 reduces the color saturation by half.
//    1.0 causes no change.
//    2.0 doubles the color saturation.
void saturation_adjust(float *const data, const int size, const int satr)
{
    for (int i = 0; i < size; i += 3)
    {
        const float r = data[i    ];
        const float g = data[i + 1];
        const float b = data[i + 2];
        const float p = sqrt((r * r * pr) + (g * g * pg) + (b * b * pb));

        data[i    ] = p + (r - p) * satr;
        data[i + 1] = p + (g - p) * satr;
        data[i + 2] = p + (b - p) * satr;
    }
}

void gamma_adjust(float *const data, const int size, const int gamm)
{
    for (int i = 0; i < size; ++i)
    {
        data[i] = pow(data[i], gamm);
    }
}

void negative(float *const data, const int size)
{
    const vfp_t white(1.0f);

    /* Vector process the data we can */
    for (int i = 0; i <= (size - SIMD_WIDTH); i += SIMD_WIDTH)
    {
        vfp_t pixel(&data[i]);
        const vfp_t nega(white - pixel);
        nega.store(&data[i]);
    }

    /* Process tail data */
    for (int i = (size & ~(SIMD_WIDTH - 1)); i < size; ++i)
    {
        data[i] = 1.0f - data[i];
    }
}
}; /* namespace raptor_raytracer */
