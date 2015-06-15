#pragma once

namespace raptor_raytracer
{
void brightness_adjust(float *const data, const int size, const int brit);
void brightness_scale(float *const data, const int size, const float brit);
void contrast_adjust(float *const data, const int size, const int cont);
void saturation_adjust(float *const data, const int size, const int satr);
void gamma_adjust(float *const data, const int size, const int gamm);
void negative(float *const data, const int size);
}; /* namespace raptor_raytracer */
