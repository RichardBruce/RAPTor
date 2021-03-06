#include "cylindrical_mapper.h"

#include "ext_colour_t.h"


namespace raptor_raytracer
{
void cylindrical_mapper::texture_coordinates(float *const u_co, float *const v_co, const point_t<> &dst, const point_t<> &n) const
{
    const point_t<> dist(dst - _c);
    const float h_off = dot_product(dist, _v);
    const float v_len = dot_product(_s, _v);
    *v_co = (h_off + (0.5f * v_len)) * (static_cast<float>(_h) / v_len);

    const point_t<> height_align(_c + (_v * h_off));
    const float theta = acos(std::max(-1.0f, std::min(1.0f, dot_product(normalise(dst - height_align), _u))));
    const float phase = theta * (0.5f / PI);
    *u_co = ((_cycles * phase) * static_cast<float>(_w)) / dot_product(_s, _u);
}
}; /* namespace raptor_raytracer */
