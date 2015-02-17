//#ifndef __LIGHT_SHADER_H__
//#define __LIGHT_SHADER_H__
//
//#include "common.h"
//#include "material.h"
//#include "line.h"
//#include "ray.h"
//
//
//namespace raptor_raytracer
//{
///* Pure virtual class for material data and shading */
//class light_shader : public material
//{
//    public :
//        /* Constructor for light emmitting object with rgb value */
//        light_shader(const ext_colour_t rgb) : 
//            material(true, ext_colour_t((((fp_t)1.0/(fp_t)255.0) * rgb.r), (((fp_t)1.0/(fp_t)255.0) * rgb.g), (((fp_t)1.0/(fp_t)255.0) * rgb.b))), 
//                 rgb(rgb)
//        {
//            /* Check the resulting rgb value is in range */
//            assert((this->rgb.r >= (fp_t)0.0) && (this->rgb.r <= (fp_t)255.0));
//            assert((this->rgb.g >= (fp_t)0.0) && (this->rgb.g <= (fp_t)255.0));
//            assert((this->rgb.b >= (fp_t)0.0) && (this->rgb.b <= (fp_t)255.0));
//        };
//        
//        light_shader(const fp_t i, const fp_t x, const fp_t y) : 
//            material(true, ext_colour_t((fp_t)  1.0e-5 * i, (fp_t)  1.0e-5 * i, (fp_t)  1.0e-5 * i)), 
//                 rgb(ext_colour_t((fp_t)255.0e-5 * i, (fp_t)255.0e-5 * i, (fp_t)255.0e-5 * i))
//        {
//            /* Check the resulting rgb value is in range */
//            assert((this->rgb.r >= (fp_t)0.0) && (this->rgb.r <= (fp_t)255.0));
//            assert((this->rgb.g >= (fp_t)0.0) && (this->rgb.g <= (fp_t)255.0));
//            assert((this->rgb.b >= (fp_t)0.0) && (this->rgb.b <= (fp_t)255.0));
//        };
//        
//                /* Constructor for light emmitting object with intensity and colour given by black body temperature */
//        light_shader(const fp_t i, const fp_t t) : material(true)
//        {
//            fp_t c_x, c_y;
//            
//            /* Convert the rgb value to xy */
//            black_body_temperature_to_cxy(t, &c_x, &c_y);
//            
//            /* Convert the Cxy to rgbs */
//            cxy_to_rgb(c_x, c_y, i * (fp_t)1.0e-5, &this->rgb);
//            
//            /* Set the light intensity to the scaled rgb value */
//            this->set_light_intensity(ext_colour_t((this->rgb.r * ((fp_t)1.0/(fp_t)255.0)), (this->rgb.g * ((fp_t)1.0/(fp_t)255.0)), (this->rgb.b * ((fp_t)1.0/(fp_t)255.0))));
//            
//            /* Check the resulting rgb value is in range */
//            assert((this->rgb.r >= (fp_t)0.0) && (this->rgb.r <= (fp_t)255.0));
//            assert((this->rgb.g >= (fp_t)0.0) && (this->rgb.g <= (fp_t)255.0));
//            assert((this->rgb.b >= (fp_t)0.0) && (this->rgb.b <= (fp_t)255.0));
//        };
//        
//        /* Constructor for light emmitting object with intensity, colour is assumed white */
//        light_shader(const fp_t i) : 
//            material(true, ext_colour_t((fp_t)  1.0e-5 * i, (fp_t)  1.0e-5 * i, (fp_t)  1.0e-5 * i)), 
//                 rgb(ext_colour_t((fp_t)255.0e-5 * i, (fp_t)255.0e-5 * i, (fp_t)255.0e-5 * i))
//        {
//            /* Check the resulting rgb value is in range */
//            assert((this->rgb.r >= (fp_t)0.0) && (this->rgb.r <= (fp_t)255.0));
//            assert((this->rgb.g >= (fp_t)0.0) && (this->rgb.g <= (fp_t)255.0));
//            assert((this->rgb.b >= (fp_t)0.0) && (this->rgb.b <= (fp_t)255.0));
//        };
//        
//        virtual ~light_shader() { };
//
//        /* Shading function. Takes the incident ray, the light 
//           ray and the normal of the intersect object */
//        void shade(const ray_trace_engine<> &r, ray &i, const line &n, const hit_t h, ext_colour_t *const c) const;
//
//    private :
//        ext_colour_t    rgb;    /* The colour of the light for shading */
//};
//
//#endif /* #ifndef __LIGHT_SHADER_H__ */
//
//}; /* namespace raptor_raytracer */
