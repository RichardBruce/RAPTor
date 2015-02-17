#include "camera.h"
#include "fftw3.h"


namespace raptor_raytracer
{
/* C style headers */
extern "C"
{
#include <stdio.h>
#include <jpeglib.h>
#include <png.h>
}


/* Componant glare functions as defined by Spencer */
inline float f0(const float t)
{
    const float tmp = t * (1.0f / 0.02f);
    return 2.61e6f * exp(-(tmp * tmp));
}


inline float f1(const float t)
{
    const float tmp = t + 0.02f;
    return 20.91f / (tmp * tmp * tmp);
}

inline float f2(const float t)
{
    const float tmp = t + 0.02f;
    return 72.37f / (tmp * tmp);
}

inline float f3(const float t, const float l)
{
    const float tmp = t - 3.0f * l * (1.0f / 568.0f);
    return 436.9f * (568.0f / l) * exp(-19.75f * (tmp * tmp));
}

struct scotopic_corona_t
{
    inline ext_colour_t operator()(const float x, const float y, const float p) const
    {
        const float dist = sqrt((x * x) + (y * y));
        const float deg  = dist * p;
        const float a    = (0.478f * f1(deg)) + (0.207f * f2(deg));
        return ext_colour_t(a, a, a);
    }
};


struct scotopic_halo_t
{
    inline ext_colour_t operator()(const float x, const float y, const float p) const
    {
        const float dist = sqrt((x * x) + (y * y));
        const float deg  = dist * p;
        return 0.033f * ext_colour_t(f3(deg, 611.0f), f3(deg, 549.0f), f3(deg, 467.0f));
    }
};


struct scotopic_bloom_t
{
    inline ext_colour_t operator()(const float x, const float y, const float p) const
    {
        const float dist = sqrt((x * x) + (y * y));
        const float deg  = dist * p;
        const float a    = 0.282f * f0(deg);
        return ext_colour_t(a,a,a);
    }
};


struct mesopic_corona_t
{
    inline ext_colour_t operator()(const float x, const float y, const float p) const
    {
        const float dist = sqrt((x * x) + (y * y));
        const float deg  = dist * p;
        const float a    = (0.478f * f1(deg)) + (0.138f * f2(deg));
        return ext_colour_t(a, a, a);
    }
};


struct mesopic_halo_t
{
    inline ext_colour_t operator()(const float x, const float y, const float p) const
    {
        const float dist = sqrt((x * x) + (y * y));
        const float deg  = dist * p;
        return 0.016f * ext_colour_t(f3(deg, 611.0f), f3(deg, 549.0f), f3(deg, 467.0f)); 
    }
};


struct mesopic_bloom_t
{
    inline ext_colour_t operator()(const float x, const float y, const float p) const
    {
        const float dist = sqrt((x * x) + (y * y));
        const float deg  = dist * p;
        const float a    = 0.368f * f0(deg);
        return ext_colour_t(a, a, a);
    }
};


struct photopic_corona_t
{
    inline ext_colour_t operator()(const float x, const float y, const float p) const
    {
        const float dist = sqrt((x * x) + (y * y));
        const float deg  = dist * p;
        const float a    = (0.478f * f1(deg)) + (0.138f * f2(deg));
        return ext_colour_t(a, a, a);
    }
};


struct photopic_halo_t
{
    inline ext_colour_t operator()(const float x, const float y, const float p) const
    {
        return ext_colour_t(0.0f, 0.0f, 0.0f);
    }
};


struct photopic_bloom_t
{
    inline ext_colour_t operator()(const float x, const float y, const float p) const
    {
        const float dist = sqrt((x * x) + (y * y));
        const float deg  = dist * p;
        const float a    = 0.384f * f0(deg);
        return ext_colour_t(a,a,a);
    }
};


float camera::generate_flare_lines(ext_colour_t *const f)
{
    float sum = 0.0f;
    int nr_of_lines = 200;
    for (int i = 0; i < nr_of_lines; i++)
    {
        float lineIntens = gen_random_mersenne_twister();
        float lineRad = 1.0;//(gen_random_mersenne_twister() * 2.0);
    
        float theta = ((i + gen_random_mersenne_twister()) * (2.0f * PI)) / nr_of_lines;  // stratified
        float dir[2] = { std::cos(theta), std::sin(theta) };
                
        int dj = (theta <= PI) ? 1 : -1;
        int j = (int)(this->y_res / 2.0f - (lineRad - 0.5f) * dj);

        /* Write the intensity of the pixel for each row */
        while ((j >= 0) && (j < (int)this->y_res))
        {
            /* Calculate the x bounds of the line on this row */
            float jminf = (j + 0.0f - this->y_res / 2.0f);
            float jmaxf = (j + 1.0f - this->y_res / 2.0f);
            float iminf = jminf/tan(theta);
            float imaxf = jmaxf/tan(theta);

            // swap if iminf>imaxf
            if (iminf > imaxf)
            {
                float temp = iminf;
                iminf = imaxf;
                imaxf = temp;
            }

            float dif = fabs(lineRad/sin(theta));
            int imin = (int)(iminf - dif + 0.5f + this->x_res / 2.0f);
            int imax = (int)(imaxf + dif - 0.5f + this->x_res / 2.0f);

            /* Set the magnitude of each pixel on this row */
            for (int k = std::max(imin, 0); k < std::min((imax + 1), (int)this->x_res); k++)
            {
                // find pixel center in spatial coordinates
                float p[2];
                p[0] = k-this->x_res / 2.0f + 0.5f;
                p[1] = j-this->y_res / 2.0f + 0.5f;

                // find distance of p to line
                float dp = (p[0] * dir[0]) + (p[1] * dir[1]);
                float dist;
                if (dp > 0.0f)
                {
                    float vec[2] = { (p[0] - dp * dir[0]), (p[0] - dp * dir[0]) };
                    dist = sqrt((vec[0] * vec[0]) + (vec[1] * vec[1]));
                }
                else 
                {
                    dist = sqrt((p[0] * p[0]) + (p[1] * p[1]));
                }

                // tent filter
                if (dist>=lineRad)
                    continue;
                float filt = 1.0f / lineRad - 1.0f / (lineRad * lineRad) * dist;
                
                float len = sqrt((p[0] * p[0]) + (p[1] * p[1]));
                float flo = std::max((len * 2.0f * PI) / nr_of_lines, 1.0f / nr_of_lines);
                float val = filt*flo*lineIntens;
                f[k + (j * this->x_res)] += ext_colour_t(val, val, val);
                sum += val;
                assert((k + (j * this->x_res)) < (this->x_res * this->y_res));
            }

            // next row
            j += dj;
        }
    }
    
    return sum;
}


template<typename T>
void camera::apply_point_spreading_function(T p, ext_colour_t *const f)
{
    const float pixdeg = (atan(-this->x_m / this->t) * 2.0f) / this->x_res;

    for (int j = 0; j < static_cast<int>(this->y_res); ++j)
    {
        for (int i = 0; i < static_cast<int>(this->x_res); ++i)
        {
            // perform 2D trapezoidal integration
            float w = static_cast<float>(i) - this->x_res / 2.0f, x = w + 1.0f;
            float y = static_cast<float>(j) - this->y_res / 2.0f, z = y + 1.0f;
      
            // determine four corners, find ratio
            float max, min;
            ext_colour_t tmp, sum1;

            // sum four corners and keep track of min and max
            tmp = p(w, y, pixdeg);
            sum1 = tmp;
            max = std::max(std::max(sum1.r, sum1.g), sum1.b);
            min = std::min(std::min(sum1.r, sum1.g), sum1.b);

            tmp = p(x, y, pixdeg);
            sum1+=tmp;
            max = std::max(std::max(tmp.r, tmp.g), std::max(tmp.b, max));
            min = std::min(std::min(tmp.r, tmp.g), std::min(tmp.b, min));

            tmp = p(w, z, pixdeg);
            sum1+=tmp;
            max = std::max(std::max(tmp.r, tmp.g), std::max(tmp.b, max));
            min = std::min(std::min(tmp.r, tmp.g), std::min(tmp.b, min));

            tmp = p(x, z, pixdeg);
            sum1+=tmp;
            max = std::max(std::max(tmp.r, tmp.g), std::max(tmp.b, max));
            min = std::min(std::min(tmp.r, tmp.g), std::min(tmp.b, min));

            // choose number of samples based on ratio of min and max
            int samps = 3;
//            int samps = (max-min)/min > 2.0 ? 100 : 10;  // FIXME
            //int samps = (fabs(w/(d*0.5)) < .1 && fabs(y/(d*0.5))<.1) ? 100 : 3;
            int n = samps-1;
        
            float h = (x - w) / n;
            float k = (z - y) / n;
        
            ext_colour_t sum2(0.0f, 0.0f, 0.0f);
            for (int k = 1; k < n; k++)
            {
                float x_k = (x - w) / static_cast<float>(n * k + w); /* This cast may be include too many terms */
                float y_k = (z - y) / static_cast<float>(n * k + y); /* This cast may be include too many terms */
                sum2 += p(x_k, y, pixdeg) + 
                        p(x_k, z, pixdeg) + 
                        p(w, y_k, pixdeg) + 
                        p(x, y_k, pixdeg);
            }
            sum1 += ext_colour_t(2.0f) * sum2;

            sum2 = ext_colour_t(0.0f, 0.0f, 0.0f);
            for (int k = 1; k < n; ++k)
            {
                for (int l = 1; l < n; ++l)
                {
                    float x_k = (x - w) / static_cast<float>(n * k + w); /* This cast may be include too many terms */
                    float y_l = (z - y) / static_cast<float>(n * l + y); /* This cast may be include too many terms */
                    sum2 += p(x_k, y_l, pixdeg);
                }
            }
            sum1 += ext_colour_t(4.0f) * sum2;
            sum1 *= ext_colour_t(0.25f * h * k);

            f[(i + (j * this->x_res))].r *= sum1.r;
            f[(i + (j * this->x_res))].g *= sum1.g;
            f[(i + (j * this->x_res))].b *= sum1.b;
        }
    }
    
    return;
}


void camera::generate_glare_filter(ext_colour_t **gf, const light_level_t ll)
{
//    float pixdeg = atan(-this->x_m / this->t) * 2.0f;
//    std::cout << "angle : " << pixdeg * (180.0/PI) << std::endl;

    (*gf) = new ext_colour_t[this->x_res * this->y_res * 3];


    /* Generate the flare lines for the lenticular halo and ciliary corona */
    const float flare_sum = this->generate_flare_lines(&(*gf)[0]);

    this->generate_flare_lines(&(*gf)[this->x_res * this->y_res]);

//    /* Normalise 32x32 pixel blocks */
//    const int mb_size = 8;
//    for (int i = 0; i < (int)this->x_res; i += mb_size)
//    {
//        for (int j = 0; j < (int)this->y_res * 2; j += mb_size)
//        {
//            ext_colour_t mb_sum(0.0, 0.0, 0.0);
//            const int mb_addr = i + (j * this->x_res);
//            for (int mb_i = 0; mb_i < mb_size; mb_i++)
//            {
//                for (int mb_j = 0; mb_j < mb_size; mb_j++)
//                {
//                    mb_sum += (*gf)[mb_addr + mb_i + (mb_j * this->x_res)];
//                }
//            }
//
//            ext_colour_t mb_mul;            
//            if (mb_sum.r > 0.0f)
//            {
//                mb_mul.r = (mb_size * mb_size) / mb_sum.r;
//            }
//
//            if (mb_sum.g > 0.0f)
//            {
//                mb_mul.g = (mb_size * mb_size) / mb_sum.g;
//            }
//
//            if (mb_sum.b > 0.0f)
//            {
//                mb_mul.b = (mb_size * mb_size) / mb_sum.b;
//            }
//
//            for (int mb_i = 0; mb_i < mb_size; mb_i++)
//            {
//                for (int mb_j = 0; mb_j < mb_size; mb_j++)
//                {
//                    (*gf)[mb_addr + mb_i + (mb_j * this->x_res)] *= mb_mul;
//                }
//            }
//        }
//    }
    
    /* Set white for the bloom with the same average intensity as the other images */
    const float bloom_lum = flare_sum / (this->x_res * this->y_res);
    for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
    {
        (*gf)[(this->x_res * this->y_res * 2) + i] = ext_colour_t(bloom_lum, bloom_lum, bloom_lum);
    }
    

    /* Apply the spreading function to each componant */
    scotopic_corona_t   sc;
    scotopic_halo_t     sh;
    scotopic_bloom_t    sb;
    mesopic_corona_t    mc;
    mesopic_halo_t      mh;
    mesopic_bloom_t     mb;
    photopic_corona_t   pc;
    photopic_halo_t     ph;
    photopic_bloom_t    pb;
    switch (ll)
    {
        case light_level_t::scotopic   : 
            this->apply_point_spreading_function(sc, &(*gf)[0]                            );
            this->apply_point_spreading_function(sh, &(*gf)[this->x_res * this->y_res    ]);
            this->apply_point_spreading_function(sb, &(*gf)[this->x_res * this->y_res * 2]);
            break;
        case light_level_t::mesopic    : 
            this->apply_point_spreading_function(mc, &(*gf)[0]                            );
            this->apply_point_spreading_function(mh, &(*gf)[this->x_res * this->y_res    ]);
            this->apply_point_spreading_function(mb, &(*gf)[this->x_res * this->y_res * 2]);
            break;
        case light_level_t::photopic   : 
            this->apply_point_spreading_function(pc, &(*gf)[0]                            );
            this->apply_point_spreading_function(ph, &(*gf)[this->x_res * this->y_res    ]);
            this->apply_point_spreading_function(pb, &(*gf)[this->x_res * this->y_res * 2]);
            break;
        default         :
            assert(false);
            break;
    }
    

    /* Combine the componants and normalise the image */
    ext_colour_t sum(0.0, 0.0, 0.0);
    for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
    {
        (*gf)[i] += (*gf)[(this->x_res * this->y_res    ) + i];
        (*gf)[i] += (*gf)[(this->x_res * this->y_res * 2) + i];
        sum += (*gf)[i];
    }
  

    // In section 3.1 on page 4 it mentions
    // that perceptually only 10% of light is scattered.
    float amountScattered = 0.0001f;
    ext_colour_t mul(1.0f, 1.0f, 1.0f);
    if (sum.r > 0.0f)
    {
        mul.r = (amountScattered * this->x_res * this->y_res)/sum.r;
    }

    if (sum.g > 0.0f)
    {
        mul.g = (amountScattered * this->x_res * this->y_res)/sum.g;
    }

    if (sum.b > 0.0f)
    {
        mul.b = (amountScattered * this->x_res * this->y_res)/sum.b;
    }
    std::cout << "Sum = " << sum.r << ", " << sum.g << ", " << sum.b << std::endl;
    std::cout << "Mul = " << mul.r << "," << mul.g << "," << mul.b << std::endl;

    for (int i = 0; i < static_cast<int>(this->x_res * this->y_res); ++i)
    {
        (*gf)[i].r *= mul.r;
        (*gf)[i].g *= mul.g;
        (*gf)[i].b *= mul.b;
    }



    
//    std::cout << "dumping glare filter" << std::endl;
//    char * tga_data = new char [this->x_res * this->y_res * 3];
//    for (int i = 0; i < (int)this->x_res; i++)
//    {
//        for (int j = 0; j < (int)this->y_res; j++)
//        {
//            tga_data[((i + (j * this->x_res)) * 3)    ] = (char)min(max((*gf)[(i + (j * this->x_res))].r * 2550.0f, 0.0f), 255.0f);
//            tga_data[((i + (j * this->x_res)) * 3) + 1] = (char)min(max((*gf)[(i + (j * this->x_res))].g * 2550.0f, 0.0f), 255.0f);
//            tga_data[((i + (j * this->x_res)) * 3) + 2] = (char)min(max((*gf)[(i + (j * this->x_res))].b * 2550.0f, 0.0f), 255.0f);
//        }
//    }
//
//    /* Open output file */
//    ostringstream file_name(ostringstream::out);
//    file_name <<"glare_filter" << ".tga";
//    std::ofstream tga_output(file_name.str().c_str(), ios::out);
//    
//    /* Write the header to the tga file */
//    tga_output << (char)0;
//    tga_output << (char)0;
//    tga_output << (char)2; /* Uncompressed RGB image */
//    tga_output << (char)0 << (char)0;
//    tga_output << (char)0 << (char)0;
//    tga_output << (char)0;
//    tga_output << (char)0 << (char)0;
//    tga_output << (char)0 << (char)0;
//    tga_output << (char)(this->x_res & 0xff) << (char)((this->x_res >> 8) & 0xff);
//    tga_output << (char)(this->y_res & 0xff) << (char)((this->y_res >> 8) & 0xff);
//    tga_output << (char)24; /* 24bpp */
//    tga_output << (char)0;
//    
//    /* Write the picture data to the tag file */
//    tga_output.write((char *)tga_data, (this->x_res * this->y_res * 3));
//
//    /* Write the footer to the tga file */
//    tga_output << (char)0; tga_output << (char)0;
//    tga_output << (char)0; tga_output << (char)0;
//    tga_output << (char)0; tga_output << (char)0;
//    tga_output << (char)0; tga_output << (char)0;
//    
//    /* Close output file */
//    tga_output.close();
    
    
//    generate_temporal_psf(0.03);
          
    return;
}


//void camera::draw_circle(ext_colour_t *o, const int x, const int y, const float s, const float v)
//{
//    const float s_sq = s * s;
//    for (int j = 0; j < (int)(this->y_res * 4); j++)
//    {
//        const float y_dist    = (static_cast<float>(j) / 4.0f) - static_cast<float>(y);
//        const float y_dist_sq = y_dist * y_dist;
//        
//        if (y_dist_sq < s_sq)
//        {
//            const float x_dist_sq    = s_sq - y_dist_sq;
//            const float x_dist       = sqrt(x_dist_sq);
//            const int  x_pixel_dist = (int)x_dist;
//            const float min_x        = max(static_cast<float>(x) - x_dist, 0.0f);
//            const float max_x        = min(static_cast<float>(x) + x_dist, static_cast<float>(this)->x_res);
//            
//            for (int i = (int)(min_x * 4.0); i < (int)(max_x * 4.0); i++)
//            {
//                this->temporal_glare_filter[i/4 + (j/4 * this->x_res)].r += v/16.0;
//                this->temporal_glare_filter[i/4 + (j/4 * this->x_res)].g += v/16.0;
//                this->temporal_glare_filter[i/4 + (j/4 * this->x_res)].b += v/16.0;
//            }
//        }
//    }
//    
//    return;
//}
void camera::draw_circle(float *o, const int x, const int y, const float s, const float v)
{
     /* Picture size and scale */
//    const float picture_dimension    = 9.0e-3f; // 9mm
//    const float pixel_resolution     = 2.0e-6f; // 1um per pixel
    const int picture_resolution    = 5000;
    const int picture_resolution_y  = 5000;

    const float s_sq = s * s;
    for (int j = 0; j < static_cast<int>(picture_resolution_y); ++j)
    {
        const float y_dist    = static_cast<float>(j - y);
        const float y_dist_sq = y_dist * y_dist;
        
        if (y_dist_sq < s_sq)
        {
            const float x_dist_sq    = s_sq - y_dist_sq;
            const float x_dist       = sqrt(x_dist_sq);
            const int  x_pixel_dist = static_cast<int>(x_dist);
            const int  min_x        = std::max(x - x_pixel_dist, 0);
            const int  max_x        = std::min(x + x_pixel_dist, picture_resolution);
            
            for (int i = min_x; i < max_x; i++)
            {
                o[i + (j * picture_resolution)] = v;
            }
        }
    }
    
    return;
}


// cppcheck-suppress unusedFunction
void camera::generate_temporal_psf(const float y)
{
    /* Picture size and scale */
//    const float picture_dimension    = 9.0e-3f; // 9mm
    const float pixel_resolution     = 2.0e-6f; // 1um per pixel
    const int picture_resolution    = 5000;
    const int picture_resolution_y  = 5000;
    
    /* Allocate an array for the pupil, lens flare lines, lens particles, eyebrows, 
      vitreous particles and the combination of the componants */
    if (temporal_glare_filter == nullptr)
    {
        this->temporal_glare_filter = new ext_colour_t[picture_resolution * picture_resolution_y];
    }
    
    float *image_componants = new float [picture_resolution * picture_resolution_y * 6];
    
    /* Calculate the pupil diameter */
    const float p        = 4.9f - (3.0f * tanh(0.4f * (log(y) + 1.0f)));
    const float p_max    = 9.0f;
    std::cout << "pupil: " << p << std::endl;
    
    const int noise_octaves = 3;
    float frequency  = 1.0f;
    float amplitude  = 0.25f;
    float noise      = 0.0f;
    float fall_off   = 0.5f;
    
    for (int i = 0; i < noise_octaves; i++)
    {
        float x = ((this->time_step / p) * frequency);
    	int wx  = (int)((float)((int)x) + 0.5f);
    	float px = x - wx;

        /* Generate some smoothed noise */
    	int n = wx * 57;
    	n = (n << 13) ^ n;
    	float v1a = (1.0f - ((n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0f);

    	n = (wx - 1) * 57;
    	n = (n << 13) ^ n;
    	float v1b = (1.0f - ((n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0f);
        float v1 = (v1a + v1b) * 0.5f;
            
    	n = (wx + 1) * 57;
    	n = (n << 13) ^ n;
    	float v2a = (1.0f - ((n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0f);

    	n = (wx + 2) * 57;
    	n = (n << 13) ^ n;
    	float v2b = (1.0f - ((n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0f);
        float v2 = (v2a + v2b) * 0.5f;

        /* Interpolate */
    	float ft = px * PI;
    	float f = (1.0f - cos(ft)) * 0.5f;
    	float i1 = v1 * (1.0f - f) + v2 * f;

    	noise += (i1 * amplitude);
    	frequency *= 2.0f;
    	amplitude *= fall_off;
    }
//    std::cout << "noise: " << noise << std::endl;

    const float pixel_scale  = pixel_resolution;//(p_max * 10.0e-3) / picture_resolution_y;
    std::cout << "pixel_scale: " << pixel_scale << std::endl;

    const float pupil_scale  = 0.5e-3f / pixel_resolution;//(min(-this->x_m, -this->y_m) / p_max) * (picture_resolution_y / (-2.0 * this->y_m));
    const float p_hippus     = (p + (noise * (p_max / p) * sqrt(1.0f - (p / p_max)))) * pupil_scale;
//    std::cout << "hippus: " << p_hippus << std::endl;
    
    /* Draw the pupil */
    memset(&image_componants[0],  0, (picture_resolution * picture_resolution_y * sizeof(float)));
    this->draw_circle(&image_componants[0], (picture_resolution >> 1), (picture_resolution_y >> 1), p_hippus, 255.0f);


    /* Randomly place particle in the vitreous humour */
    for (int i = (int)(picture_resolution * picture_resolution_y); i < (int)(2 * picture_resolution * picture_resolution_y); i++)
    {
        image_componants[i] = 255.0f;
    }

    const int vitreous_humour_offset    = (picture_resolution * picture_resolution_y);
    const int vitreous_humour_particles = 100;
    for (int p = 0; p < vitreous_humour_particles; p++)
    {
        int x   = (int)(gen_random_mersenne_twister() * static_cast<float>(picture_resolution));
        int y   = (int)(gen_random_mersenne_twister() * static_cast<float>(picture_resolution_y));
        int sz  = (int)(((gen_random_mersenne_twister() * 4.0e-6f) + 1.0e-6f) / pixel_resolution);
        this->draw_circle(&image_componants[vitreous_humour_offset], x, y, sz, 0.0f);
    }


    /* Draw the lens fibers */
    for (int i = (int)(2 * picture_resolution * picture_resolution_y); i < static_cast<int>(3 * picture_resolution * picture_resolution_y); i++)
    {
        image_componants[i] = 255.0f;
    }

    const int  lens_fiber_offset    = (2 * picture_resolution * picture_resolution_y);
    const int  lens_fiber_lines     = 200;
    const float inv_lens_fiber_lines = (1.0f / 200.0f);
    const float min_line_dist_sq     = (1.0f / 10.0f) * picture_resolution_y * picture_resolution_y;
    for (int l = 0; l < lens_fiber_lines; l++)
    {
        const float theta     = ((static_cast<float>(l) + gen_random_mersenne_twister()) * (2.0f * PI)) * inv_lens_fiber_lines;
        const float x_y_ratio = sin(theta) / cos(theta);
                
        const float dj = (theta <= PI) ? 1 : -1;
        for (float j = 0; j < static_cast<float>(picture_resolution_y / 2); j++)
        {
            const int y_addr = static_cast<int>((j * dj) + (static_cast<float>(picture_resolution_y) * 0.5f));

            float fmin_x = (j - 0.5f) * x_y_ratio;
            float fmax_x = (j + 0.5f) * x_y_ratio;
            if (fmax_x < fmin_x)
            {
                const float tmp = fmax_x;
                fmax_x = fmin_x;
                fmin_x = tmp;
            }
            int min_x   = (int)fmin_x;
            int max_x   = (int)fmax_x;

            for (int i = min_x; i <= max_x; i++)
            {
                const float dist_sq = (i * i) + (j * j);

                const int x_addr = (int)((i * dj) + (static_cast<float>(picture_resolution) * 0.5f));
                if ((dist_sq > min_line_dist_sq) && (x_addr < (int)picture_resolution) && (x_addr >= 0))
                {
                    const int addr = lens_fiber_offset + x_addr + (y_addr * picture_resolution);
                    image_componants[addr] = 0.0f;
                }
            }
        }
    }


    /* Draw the lens particles */
    for (int i = (int)(3 * picture_resolution * picture_resolution_y); i < (int)(4 * picture_resolution * picture_resolution_y); i++)
    {
        image_componants[i] = 255.0f;
    }

    const int lens_particle_offset  = (3 * picture_resolution * picture_resolution_y);
    const int lens_particles        = 750;
    for (int p = 0; p < lens_particles; p++)
    {
        int x   = static_cast<int>(gen_random_mersenne_twister() * static_cast<float>(picture_resolution));
        int y   = static_cast<int>(gen_random_mersenne_twister() * static_cast<float>(picture_resolution_y));
        int sz  = static_cast<int>(((gen_random_mersenne_twister() * 5.0e-6f) + 1.0e-6f) / pixel_resolution);
        this->draw_circle(&image_componants[lens_particle_offset], x, y, sz, 0.0f);
    }


    /* Draw the cornea particles */
    for (int i = static_cast<int>(4 * picture_resolution * picture_resolution_y); i < static_cast<int>(5 * picture_resolution * picture_resolution_y); ++i)
    {
        image_componants[i] = 255.0f;
    }

    const int cornea_particle_offset  = (4 * picture_resolution * picture_resolution_y);
    const int cornea_particles        = 250;
    for (int p = 0; p < cornea_particles; p++)
    {
        int x   = (int)(gen_random_mersenne_twister() * static_cast<float>(picture_resolution));
        int y   = (int)(gen_random_mersenne_twister() * static_cast<float>(picture_resolution_y));
        int sz  = 15.0e-6f / pixel_resolution;
        this->draw_circle(&image_componants[cornea_particle_offset], x, y, sz, 0.0f);
    }


    /* Sum the componants */
    const int complete_image_offset  = (5 * picture_resolution * picture_resolution_y);
    for (int i = 0; i < static_cast<int>(picture_resolution * picture_resolution_y); i++)
    {
        image_componants[complete_image_offset + i]  = image_componants[i];
        image_componants[complete_image_offset + i] += image_componants[i + vitreous_humour_offset];
        image_componants[complete_image_offset + i] += image_componants[i + lens_fiber_offset     ];
        image_componants[complete_image_offset + i] += image_componants[i + lens_particle_offset  ];
        image_componants[complete_image_offset + i] += image_componants[i + cornea_particle_offset];
       
        if (image_componants[complete_image_offset + i] < (255.0f * 5.0f))
        {
            image_componants[complete_image_offset + i] = 1.0f;
        }
        else
        {
            image_componants[complete_image_offset + i] = 0.0f;
        }
    }

    /* Down sample to 1K X 1K */
    const int downsampled_resolution = 1000;
    const int x_samples = (picture_resolution   / downsampled_resolution);
    const int y_samples = (picture_resolution_y / downsampled_resolution);
    std::cout << picture_resolution << ", " << picture_resolution_y << std::endl;
    std::cout << x_samples << ", " << y_samples << std::endl;
    
    int image_addr = 0;
    for (int j = 0; j < downsampled_resolution; j++)
    {
        for (int i = 0; i < downsampled_resolution; i++)
        {
            float sample = 0.0f;
            for (int i_aa = 0; i_aa < x_samples; i_aa++)
            {
                for (int j_aa = 0; j_aa < y_samples; j_aa++)
                {
                    int sample_addr = ((i * x_samples) + i_aa) + (((j * y_samples) + j_aa) * picture_resolution);
                    sample += image_componants[complete_image_offset + sample_addr];
                }
            }
            image_componants[complete_image_offset + image_addr++] = sample / (x_samples * y_samples);
        }
    }

#if 1
{
    std::cout << "dumping complete image" << std::endl;
    char * tga_data = new char [downsampled_resolution * downsampled_resolution * 3];
    for (int i = 0; i < static_cast<int>(downsampled_resolution); ++i)
    {
        for (int j = 0; j < static_cast<int>(downsampled_resolution); ++j)
        {
            tga_data[((i + (j * downsampled_resolution)) * 3)    ] = static_cast<char>(std::min(std::max(image_componants[complete_image_offset + (i + (j * downsampled_resolution))] * 255.0f, 0.0f), 255.0f));
            tga_data[((i + (j * downsampled_resolution)) * 3) + 1] = static_cast<char>(std::min(std::max(image_componants[complete_image_offset + (i + (j * downsampled_resolution))] * 255.0f, 0.0f), 255.0f));
            tga_data[((i + (j * downsampled_resolution)) * 3) + 2] = static_cast<char>(std::min(std::max(image_componants[complete_image_offset + (i + (j * downsampled_resolution))] * 255.0f, 0.0f), 255.0f));
        }
    }
    
    /* Open output file */
    std::ostringstream file_name(std::ostringstream::out);
    file_name <<"complete_image" << ".tga";
    std::ofstream tga_output(file_name.str().c_str(), std::ios::out);
    
    /* Write the header to the tga file */
    tga_output << (char)0;
    tga_output << (char)0;
    tga_output << (char)2; /* Uncompressed RGB image */
    tga_output << (char)0 << (char)0;
    tga_output << (char)0 << (char)0;
    tga_output << (char)0;
    tga_output << (char)0 << (char)0;
    tga_output << (char)0 << (char)0;
    tga_output << (char)(downsampled_resolution & 0xff) << (char)((downsampled_resolution >> 8) & 0xff);
    tga_output << (char)(downsampled_resolution & 0xff) << (char)((downsampled_resolution >> 8) & 0xff);
    tga_output << (char)24; /* 24bpp */
    tga_output << (char)0;
    
    /* Write the picture data to the tag file */
    tga_output.write((char *)tga_data, (downsampled_resolution * downsampled_resolution * 3));

    /* Write the footer to the tga file */
    tga_output << (char)0; tga_output << (char)0;
    tga_output << (char)0; tga_output << (char)0;
    tga_output << (char)0; tga_output << (char)0;
    tga_output << (char)0; tga_output << (char)0;
    
    /* Close output file */
    tga_output.close();
    
    delete [] tga_data;
}
#endif


    /* Multiply by the complex exponential e^(i * (PI / (lambda * d)) * (xp^2 + yq^2)) */
    /*                                    = cos((PI / (lambda * d)) * (x^2 + y^2))) + (i * sin((PI / (lambda * d)) * (xp^2 + yq^2))) */
    fftwf_complex *image_in;
    fftwf_complex *image_out;
    fftwf_plan image_p;
    image_in    = (fftwf_complex *)fftwf_malloc(downsampled_resolution * downsampled_resolution * sizeof(fftwf_complex));
    image_out   = (fftwf_complex *)fftwf_malloc(downsampled_resolution * downsampled_resolution * sizeof(fftwf_complex));
    image_p     = fftwf_plan_dft_2d(downsampled_resolution, downsampled_resolution, image_in, image_out, FFTW_FORWARD, FFTW_ESTIMATE);

    const float retina_pixel_scale   = 1.0e-9f;
    const float image_scale_factor   = retina_pixel_scale / pixel_scale;
//    const float pupil_to_retina_dist = 24.0e-3f;
//    const float lambda               = 575.0e-9f;
    const float lambda_d             = 5.0e-4f;//lambda * pupil_to_retina_dist;
    std::cout << "lambda_d          : " << lambda_d           << std::endl;
    std::cout << "image_scale_factor: " << image_scale_factor << std::endl;
    for (int j = 0; j < (int)downsampled_resolution; j++)
    {
        const float yq = (static_cast<float>(j) - (static_cast<float>(downsampled_resolution) * 0.5f)) * (5.0f * pixel_scale) * 0.5f;
        for (int i = 0; i < (int)downsampled_resolution; i++)
        {
            const float xp = (static_cast<float>(i) - (static_cast<float>(downsampled_resolution) * 0.5f)) * (5.0f * pixel_scale) * 0.5f;
            const float exponential = ((PI / lambda_d) * ((xp * xp) + (yq * yq)));
            
            const int image_addr = i + (downsampled_resolution * j);

            const int xp_addr    = std::max(std::min((int)((xp / (5.0f * pixel_scale)) + (static_cast<float>(downsampled_resolution) * 0.5f)), (int)downsampled_resolution - 1), 0);
            const int yq_addr    = std::max(std::min((int)((yq / (5.0f * pixel_scale)) + (static_cast<float>(downsampled_resolution) * 0.5f)), (int)downsampled_resolution - 1), 0);
            const int pupil_addr = complete_image_offset + xp_addr + (downsampled_resolution * yq_addr);

            image_in[image_addr][0] = cos(exponential) * image_componants[pupil_addr];
            image_in[image_addr][1] = sin(exponential) * image_componants[pupil_addr];
        }
    }

#if 1
{
    std::cout << "dumping pre fft image" << std::endl;
    char * tga_data = new char [downsampled_resolution * downsampled_resolution * 3];
    for (int i = 0; i < static_cast<int>(downsampled_resolution); ++i)
    {
        for (int j = 0; j < static_cast<int>(downsampled_resolution); ++j)
        {
            tga_data[((i + (j * downsampled_resolution)) * 3)    ] = static_cast<char>(std::min(std::max(image_in[i + (j * downsampled_resolution)][0] * 255.0f, 0.0f), 255.0f));
            tga_data[((i + (j * downsampled_resolution)) * 3) + 1] = static_cast<char>(std::min(std::max(image_in[i + (j * downsampled_resolution)][0] * 255.0f, 0.0f), 255.0f));
            tga_data[((i + (j * downsampled_resolution)) * 3) + 2] = static_cast<char>(std::min(std::max(image_in[i + (j * downsampled_resolution)][0] * 255.0f, 0.0f), 255.0f));
        }
    }
    
    /* Open output file */
    std::ostringstream file_name(std::ostringstream::out);
    file_name <<"pre_fft_image" << ".tga";
    std::ofstream tga_output(file_name.str().c_str(), std::ios::out);
    
    /* Write the header to the tga file */
    tga_output << (char)0;
    tga_output << (char)0;
    tga_output << (char)2; /* Uncompressed RGB image */
    tga_output << (char)0 << (char)0;
    tga_output << (char)0 << (char)0;
    tga_output << (char)0;
    tga_output << (char)0 << (char)0;
    tga_output << (char)0 << (char)0;
    tga_output << (char)(downsampled_resolution & 0xff) << (char)((downsampled_resolution >> 8) & 0xff);
    tga_output << (char)(downsampled_resolution & 0xff) << (char)((downsampled_resolution >> 8) & 0xff);
    tga_output << (char)24; /* 24bpp */
    tga_output << (char)0;
    
    /* Write the picture data to the tag file */
    tga_output.write((char *)tga_data, (downsampled_resolution * downsampled_resolution * 3));

    /* Write the footer to the tga file */
    tga_output << (char)0; tga_output << (char)0;
    tga_output << (char)0; tga_output << (char)0;
    tga_output << (char)0; tga_output << (char)0;
    tga_output << (char)0; tga_output << (char)0;
    
    /* Close output file */
    tga_output.close();
    
    delete [] tga_data;
}
#endif

    /* Move 0Hz to the centre of the output image */
    float offset = 1.0f;
    for (int j = 0; j < (int)downsampled_resolution; j++)
    {
        for (int i = 0; i < (int)downsampled_resolution; i++)
        {
            image_in[i + (j * downsampled_resolution)][0] *= offset;
            image_in[i + (j * downsampled_resolution)][1] *= offset;
            offset *= -1.0f;
        }
        offset *= -1.0f;
    }
    
    /* FFT, take square magnitude and scale */    
    fftwf_execute(image_p);

    const float fft_scale = 1.0f;// / (lambda_d * lambda_d);
    for (int i = 0; i < (int)(downsampled_resolution * downsampled_resolution); i++)
    {
        image_in[i][0] /= (downsampled_resolution * downsampled_resolution);
        image_in[i][1] /= (downsampled_resolution * downsampled_resolution);
        image_in[i][0] = ((image_out[i][0] * image_out[i][0]) + (image_out[i][1] * image_out[i][1])) * fft_scale;
    }

#if 1
{
    std::cout << "dumping fft image" << std::endl;
    char * tga_data = new char [downsampled_resolution * downsampled_resolution * 3];
    for (int i = 0; i < (int)downsampled_resolution; i++)
    {
        for (int j = 0; j < (int)downsampled_resolution; j++)
        {
            tga_data[((i + (j * downsampled_resolution)) * 3)    ] = static_cast<char>(std::min(std::max(image_in[i + (j * downsampled_resolution)][0] / 100.0f, 0.0f), 255.0f));
            tga_data[((i + (j * downsampled_resolution)) * 3) + 1] = static_cast<char>(std::min(std::max(image_in[i + (j * downsampled_resolution)][0] / 100.0f, 0.0f), 255.0f));
            tga_data[((i + (j * downsampled_resolution)) * 3) + 2] = static_cast<char>(std::min(std::max(image_in[i + (j * downsampled_resolution)][0] / 100.0f, 0.0f), 255.0f));
        }
    }
    
    /* Open output file */
    std::ostringstream file_name(std::ostringstream::out);
    file_name <<"fft_image" << ".tga";
    std::ofstream tga_output(file_name.str().c_str(), std::ios::out);
    
    /* Write the header to the tga file */
    tga_output << (char)0;
    tga_output << (char)0;
    tga_output << (char)2; /* Uncompressed RGB image */
    tga_output << (char)0 << (char)0;
    tga_output << (char)0 << (char)0;
    tga_output << (char)0;
    tga_output << (char)0 << (char)0;
    tga_output << (char)0 << (char)0;
    tga_output << (char)(downsampled_resolution & 0xff) << (char)((downsampled_resolution >> 8) & 0xff);
    tga_output << (char)(downsampled_resolution & 0xff) << (char)((downsampled_resolution >> 8) & 0xff);
    tga_output << (char)24; /* 24bpp */
    tga_output << (char)0;
    
    /* Write the picture data to the tag file */
    tga_output.write((char *)tga_data, (downsampled_resolution * downsampled_resolution * 3));

    /* Write the footer to the tga file */
    tga_output << (char)0; tga_output << (char)0;
    tga_output << (char)0; tga_output << (char)0;
    tga_output << (char)0; tga_output << (char)0;
    tga_output << (char)0; tga_output << (char)0;
    
    /* Close output file */
    tga_output.close();
    
    delete [] tga_data;
}
#endif

    /* Apply spectral blur */
    memset(&this->temporal_glare_filter[0],  0, (downsampled_resolution * downsampled_resolution * sizeof(ext_colour_t)));
    
    /* CIE conversion factors / 4 for bi-linear filtering */
    static const float cie_xf[] = {   14.0f / (4.0f * 106836.0f),     42.0f / (4.0f * 106836.0f),   143.0f / (4.0f * 106836.0f),   435.0f / (4.0f * 106836.0f), 1344.0f / (4.0f * 106836.0f),  
                                   2839.0f / (4.0f * 106836.0f),   3483.0f / (4.0f * 106836.0f),  3362.0f / (4.0f * 106836.0f),  2908.0f / (4.0f * 106836.0f), 1954.0f / (4.0f * 106836.0f), 
                                    956.0f / (4.0f * 106836.0f),    320.0f / (4.0f * 106836.0f),    49.0f / (4.0f * 106836.0f),    93.0f / (4.0f * 106836.0f),  633.0f / (4.0f * 106836.0f),  
                                   1655.0f / (4.0f * 106836.0f),   2904.0f / (4.0f * 106836.0f),  4334.0f / (4.0f * 106836.0f),  5945.0f / (4.0f * 106836.0f), 7621.0f / (4.0f * 106836.0f), 
                                   9163.0f / (4.0f * 106836.0f),  10263.0f / (4.0f * 106836.0f), 10622.0f / (4.0f * 106836.0f), 10026.0f / (4.0f * 106836.0f), 8544.0f / (4.0f * 106836.0f),  
                                   6424.0f / (4.0f * 106836.0f),   4479.0f / (4.0f * 106836.0f),  2835.0f / (4.0f * 106836.0f),  1649.0f / (4.0f * 106836.0f),  874.0f / (4.0f * 106836.0f),  
                                    468.0f / (4.0f * 106836.0f),    227.0f / (4.0f * 106836.0f),   114.0f / (4.0f * 106836.0f),    58.0f / (4.0f * 106836.0f),   29.0f / (4.0f * 106836.0f),  
                                     14.0f / (4.0f * 106836.0f),      7.0f / (4.0f * 106836.0f),     3.0f / (4.0f * 106836.0f),     2.0f / (4.0f * 106836.0f),    1.0f / (4.0f * 106836.0f), 0.0f / (4.0f * 106836.0f) };//106836L

    static const float cie_yf[] = {    0.0f / (4.0f * 106856.0f),     1.0f / (4.0f * 106856.0f),     4.0f / (4.0f * 106856.0f),    12.0f / (4.0f * 106856.0f),    40.0f / (4.0f * 106856.0f),
                                    116.0f / (4.0f * 106856.0f),   230.0f / (4.0f * 106856.0f),   380.0f / (4.0f * 106856.0f),   600.0f / (4.0f * 106856.0f),   910.0f / (4.0f * 106856.0f),
                                   1390.0f / (4.0f * 106856.0f),  2080.0f / (4.0f * 106856.0f),  3230.0f / (4.0f * 106856.0f),  5030.0f / (4.0f * 106856.0f),  7100.0f / (4.0f * 106856.0f),
                                   8620.0f / (4.0f * 106856.0f),  9540.0f / (4.0f * 106856.0f),  9950.0f / (4.0f * 106856.0f),  9950.0f / (4.0f * 106856.0f),  9520.0f / (4.0f * 106856.0f),
                                   8700.0f / (4.0f * 106856.0f),  7570.0f / (4.0f * 106856.0f),  6310.0f / (4.0f * 106856.0f),  5030.0f / (4.0f * 106856.0f),  3810.0f / (4.0f * 106856.0f),  
                                   2650.0f / (4.0f * 106856.0f),  1750.0f / (4.0f * 106856.0f),  1070.0f / (4.0f * 106856.0f),   610.0f / (4.0f * 106856.0f),   320.0f / (4.0f * 106856.0f),
                                    170.0f / (4.0f * 106856.0f),    82.0f / (4.0f * 106856.0f),    41.0f / (4.0f * 106856.0f),    21.0f / (4.0f * 106856.0f),    10.0f / (4.0f * 106856.0f),     
                                      5.0f / (4.0f * 106856.0f),     2.0f / (4.0f * 106856.0f),     1.0f / (4.0f * 106856.0f),     1.0f / (4.0f * 106856.0f),     0.0f / (4.0f * 106856.0f),  0.0f / (4.0f * 106856.0f) };//106856L

    static const float cie_zf[] = {    65.0f / (4.0f * 106836.0f),   201.0f / (4.0f * 106836.0f),   679.0f / (4.0f * 106836.0f),  2074.0f / (4.0f * 106836.0f),  6456.0f / (4.0f * 106836.0f), 
                                   13856.0f / (4.0f * 106836.0f), 17471.0f / (4.0f * 106836.0f), 17721.0f / (4.0f * 106836.0f), 16692.0f / (4.0f * 106836.0f), 12876.0f / (4.0f * 106836.0f),
                                    8130.0f / (4.0f * 106836.0f),  4652.0f / (4.0f * 106836.0f),  2720.0f / (4.0f * 106836.0f),  1582.0f / (4.0f * 106836.0f),   782.0f / (4.0f * 106836.0f), 
                                     422.0f / (4.0f * 106836.0f),   203.0f / (4.0f * 106836.0f),    87.0f / (4.0f * 106836.0f),    39.0f / (4.0f * 106836.0f),    21.0f / (4.0f * 106836.0f),
                                      17.0f / (4.0f * 106836.0f),    11.0f / (4.0f * 106836.0f),     8.0f / (4.0f * 106836.0f),     3.0f / (4.0f * 106836.0f),     2.0f / (4.0f * 106836.0f),     
                                       0.0f / (4.0f * 106836.0f),     0.0f / (4.0f * 106836.0f),     0.0f / (4.0f * 106836.0f),     0.0f / (4.0f * 106836.0f),     0.0f / (4.0f * 106836.0f),
                                       0.0f / (4.0f * 106836.0f),     0.0f / (4.0f * 106836.0f),     0.0f / (4.0f * 106836.0f),     0.0f / (4.0f * 106836.0f),     0.0f / (4.0f * 106836.0f),     
                                       0.0f / (4.0f * 106836.0f),     0.0f / (4.0f * 106836.0f),     0.0f / (4.0f * 106836.0f),     0.0f / (4.0f * 106836.0f),     0.0f / (4.0f * 106836.0f), 0.0f / (4.0f * 106836.0f) };//106770L

    for (int j = 0; j < (int)downsampled_resolution; j++)
    {
        for (int i = 0; i < (int)downsampled_resolution; i++)
        {
            const float x_o  = static_cast<float>(i) - (downsampled_resolution * 0.5f);
            const float y_o  = static_cast<float>(j) - (downsampled_resolution * 0.5f);
            float lambda_i   = 380.0f;

            for (int f = 0; f < 41; f++)
            {
                /* X, Y co-ordinate of the frequency */
                const float scale    = 575.0f / lambda_i;
                const float x_i      = std::min(std::max(static_cast<float>((x_o * scale) + (static_cast<float>(downsampled_resolution) * 0.5f)), 0.0f), (static_cast<float>(downsampled_resolution) - 1.0f));
                const float y_i      = std::min(std::max(static_cast<float>((y_o * scale) + (static_cast<float>(downsampled_resolution) * 0.5f)), 0.0f), (static_cast<float>(downsampled_resolution) - 1.0f));
                
                /* Bi-linear interpolation co-ordinates and weights */
                const int x_index   = std::max((int)0, std::min((int)x_i,    (int)(downsampled_resolution - 1)));
                const int xx_index  = std::max((int)0, std::min(x_index + 1, (int)(downsampled_resolution - 1)));
    
                const int y_index   = std::max((int)0, std::min((int)y_i,    (int)(downsampled_resolution - 1)));
                const int yy_index  = std::max((int)0, std::min(y_index + 1, (int)(downsampled_resolution - 1)));

                const float x_alpha      = x_i - static_cast<float>(x_index);
                const float y_alpha      = y_i - static_cast<float>(y_index);
                const float m_x_alpha    = 1.0f - x_alpha;
                const float m_y_alpha    = 1.0f - y_alpha;
                const float x_y_alpha    = m_x_alpha * m_y_alpha;
                const float xx_y_alpha   =   x_alpha * m_y_alpha;
                const float x_yy_alpha   = m_x_alpha *   y_alpha;
                const float xx_yy_alpha  =   x_alpha *   y_alpha;

                const int output_addr = i + (j * downsampled_resolution);
                this->temporal_glare_filter[output_addr].r += x_y_alpha   * cie_xf[f] * image_in[x_index  + (y_index  * downsampled_resolution)][0];
                this->temporal_glare_filter[output_addr].g += x_y_alpha   * cie_yf[f] * image_in[x_index  + (y_index  * downsampled_resolution)][0];
                this->temporal_glare_filter[output_addr].b += x_y_alpha   * cie_zf[f] * image_in[x_index  + (y_index  * downsampled_resolution)][0];

                this->temporal_glare_filter[output_addr].r += xx_y_alpha  * cie_xf[f] * image_in[xx_index + (y_index  * downsampled_resolution)][0];
                this->temporal_glare_filter[output_addr].g += xx_y_alpha  * cie_yf[f] * image_in[xx_index + (y_index  * downsampled_resolution)][0];
                this->temporal_glare_filter[output_addr].b += xx_y_alpha  * cie_zf[f] * image_in[xx_index + (y_index  * downsampled_resolution)][0];

                this->temporal_glare_filter[output_addr].r += x_yy_alpha  * cie_xf[f] * image_in[x_index  + (yy_index * downsampled_resolution)][0];
                this->temporal_glare_filter[output_addr].g += x_yy_alpha  * cie_yf[f] * image_in[x_index  + (yy_index * downsampled_resolution)][0];
                this->temporal_glare_filter[output_addr].b += x_yy_alpha  * cie_zf[f] * image_in[x_index  + (yy_index * downsampled_resolution)][0];

                this->temporal_glare_filter[output_addr].r += xx_yy_alpha * cie_xf[f] * image_in[xx_index + (yy_index * downsampled_resolution)][0];
                this->temporal_glare_filter[output_addr].g += xx_yy_alpha * cie_yf[f] * image_in[xx_index + (yy_index * downsampled_resolution)][0];
                this->temporal_glare_filter[output_addr].b += xx_yy_alpha * cie_zf[f] * image_in[xx_index + (yy_index * downsampled_resolution)][0];
                
                lambda_i += ((770.0f - 380.0f) / 40.0f);
            }
        }
    }

    this->convert_xyz_to_rgb(this->temporal_glare_filter, downsampled_resolution, downsampled_resolution);

#if 1
{
    std::cout << "dumping rgb spectral image" << std::endl;
    char * tga_data = new char [downsampled_resolution * downsampled_resolution * 3];
    for (int i = 0; i < (int)downsampled_resolution; i++)
    {
        for (int j = 0; j < (int)downsampled_resolution; j++)
        {
            tga_data[((i + (j * downsampled_resolution)) * 3)    ] = static_cast<char>(std::min(std::max(this->temporal_glare_filter[i + (j * downsampled_resolution)].b / 25500.0f, 0.0f), 255.0f));
            tga_data[((i + (j * downsampled_resolution)) * 3) + 1] = static_cast<char>(std::min(std::max(this->temporal_glare_filter[i + (j * downsampled_resolution)].g / 25500.0f, 0.0f), 255.0f));
            tga_data[((i + (j * downsampled_resolution)) * 3) + 2] = static_cast<char>(std::min(std::max(this->temporal_glare_filter[i + (j * downsampled_resolution)].r / 25500.0f, 0.0f), 255.0f));
        }
    }
    
    /* Open output file */
    std::ostringstream file_name(std::ostringstream::out);
    file_name <<"rgb_spectral_image" << ".tga";
    std::ofstream tga_output(file_name.str().c_str(), std::ios::out);
    
    /* Write the header to the tga file */
    tga_output << (char)0;
    tga_output << (char)0;
    tga_output << (char)2; /* Uncompressed RGB image */
    tga_output << (char)0 << (char)0;
    tga_output << (char)0 << (char)0;
    tga_output << (char)0;
    tga_output << (char)0 << (char)0;
    tga_output << (char)0 << (char)0;
    tga_output << (char)(downsampled_resolution & 0xff) << (char)((downsampled_resolution >> 8) & 0xff);
    tga_output << (char)(downsampled_resolution & 0xff) << (char)((downsampled_resolution >> 8) & 0xff);
    tga_output << (char)24; /* 24bpp */
    tga_output << (char)0;
    
    /* Write the picture data to the tag file */
    tga_output.write((char *)tga_data, (downsampled_resolution * downsampled_resolution * 3));

    /* Write the footer to the tga file */
    tga_output << (char)0; tga_output << (char)0;
    tga_output << (char)0; tga_output << (char)0;
    tga_output << (char)0; tga_output << (char)0;
    tga_output << (char)0; tga_output << (char)0;
    
    /* Close output file */
    tga_output.close();
    
    delete [] tga_data;
}
#endif   
    /* Clean up */
    delete [] image_componants;

    fftwf_destroy_plan(image_p);
    fftwf_free(image_in);
    fftwf_free(image_out);
    
    return;
}


camera & camera::tone_map(const tone_mapping_mode_t tone_map, const float key, const float xw, const float yw, const bool gf, const bool ta, const bool ds, const bool sc)
{
    /* RGB to CIE xyz conversion matrix */
    static const float xb[3] = { 0.4124f,  0.3576f, 0.1805f };
    static const float yb[3] = { 0.2126f,  0.7151f, 0.0721f };
    static const float zb[3] = { 0.0193f,  0.1192f, 0.9505f };
    static const float vb[3] = { 0.01477f, 0.497f,  0.631f  };
    
    /* Convert to CIE Yxy */
    float Yi     = 0.0f;
    float maxY   = 0.0f;
    float rgbAvg = 0.0f;
    int numpix  = 0;
    for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
    {
        this->image[i] /= 255.0f;

        /* Find average rgb value for average tone mapper */
        if (tone_map == tone_mapping_mode_t::global_exposure)
        {
            rgbAvg += (this->image[i].r + this->image[i].g + this->image[i].b) * (1.0f / 3.0f);
        }

        /* Convert to CIE xyz */
        ext_colour_t xyz;
        xyz.r = 683.0f * ((xb[0] * this->image[i].r) + (xb[1] * this->image[i].g) + (xb[2] * this->image[i].b));
        xyz.g = 683.0f * ((yb[0] * this->image[i].r) + (yb[1] * this->image[i].g) + (yb[2] * this->image[i].b));
        xyz.b = 683.0f * ((zb[0] * this->image[i].r) + (zb[1] * this->image[i].g) + (zb[2] * this->image[i].b));
        if (xyz.g < 3.18e-7f)
        {
            xyz.g = 3.18e-7f;
        }
        else
        {
            numpix++;
            maxY = std::max(maxY, xyz.g);
            Yi += log10(xyz.g);
        }

        /* Convert to CIE Yyz */
        float W = xyz.g + xyz.r + xyz.b;
        float x = xyz.r/W;
        float y = xyz.g/W;

        /* Desaturate colours */
        if (ds)
        {
            float s;
            const float log10_Y = log10(xyz.g);
            if (log10_Y < -2.0f)
            {
                s = 0.0f;
            }
            else if (log10_Y < 0.6f)
            {
                const float scaled_log10_y       = (log10_Y + 2.0f) / 2.6f;
                const float sq_scaled_log10_y    = scaled_log10_y * scaled_log10_y;
                s = (3.0f * sq_scaled_log10_y) -  (2.0f * scaled_log10_y * sq_scaled_log10_y);
            }
            else
            {
                s = 1.0f;
            }

            /* Scotopic luminance */
            const float V = 1700.0f * ((vb[0] * this->image[i].r) + (vb[1] * this->image[i].g) + (vb[2] * this->image[i].b));
    
            x     = ((1.0f - s) * xw) + (s * (x + xw - (1.0f / 3.0f)));
            y     = ((1.0f - s) * yw) + (s * (y + yw - (1.0f / 3.0f)));
            xyz.g = (0.4468f * (1.0f - s) * V) + (s * xyz.g);
        }


        this->image[i].r = x;
        this->image[i].g = xyz.g;
        this->image[i].b = y;
    }

    if (numpix)
    {
        Yi /= numpix;
    }

    rgbAvg /= (this->x_res * this->y_res);
    Yi = std::pow(10.0f, Yi);


    /* Correct Yi to the adaption level */
    if (ta && (this->time_step > 0.0f))
    {
        float k;
        const float log_la = log(Yi);
        if (log_la < -2.0f)
        {
            k = 1.0f;
        }
        else if (log_la < 1.0f)
        {
            k = 1.0f - ((log_la + 2.0f) * (1.0f / 3.0f));
        }
        else
        {
            k = 0.0f;
        }
        
        /* Dark adaption (light -> dark) */
        if (this->adatption_level > Yi)
        {
            /* Unbleached pigment proportion, luminances in Trolands (luminance * pupil area in cd/m^2 * mm^2) */
//            const yi_troland    = Yi * 75.0f;
//            const float p_cone   = 45000.0f / (45000.0f * yi_troland);
//            const float p_rod    = 40000.0f / (40000.0f * yi_troland);
//            
//            const float t_cone   = 0.02f * 8.0f;
//            const float t_rod    = 0.1f  * 0.015f;
//            
//            const float pt_cone  = t_cone * exp(3.0f  * (1.0f - p_cone));
//            const float pt_rod   = t_rod  * exp(19.0f * (1.0f - p_rod ));
            
            /* Cone adaption level */
            const float a_cone = Yi + ((this->adatption_level - Yi) * exp(-(this->time_step / 0.2f))); // 0.200s for no bleaching else 120s

            /* Rod adaption level */
            const float a_rod  = Yi + ((this->adatption_level - Yi) * exp(-(this->time_step / 6.0f))); // 6s     for no bleaching else 400s
            
            this->adatption_level = ((1.0f - k) * a_cone) + (k * a_rod);
        }
        /* Light adaption (dark to light) */
        else
        {
            const float r    = (Yi / this->adatption_level);
            const float c1   = this->adatption_level * (std::pow(r, 0.9f) - 1.0f);
            const float c2   = Yi * (1.0f - std::pow(r, -0.1f));

            /* Cone adaption level */
            const float a_cone = Yi - (c1 * exp(-(this->time_step / 16.0e-3f))) - (c2 * exp(-(this->time_step / 200.0e-3f)));

            /* Rod adaption level */
            const float a_rod  = Yi - (c1 * exp(-(this->time_step / 40.0e-3f))) - (c2 * exp(-(this->time_step / 6.0f)));

            this->adatption_level = ((1.0f - k) * a_cone) + (k * a_rod);
        }
        
        std::cout << Yi << " -> " << this->adatption_level << std::endl;
        Yi = this->adatption_level;
    }
    else
    {
        this->adatption_level = Yi;
    }


    /* Contrast based tone mapping (Ward) */
    if (tone_map == tone_mapping_mode_t::global_contrast)
    {
        const float Yw  = 300.0f;  /* Maximum display illumination */
        const float num = 1.219f + std::pow(Yw * 0.5f, 0.4f);
        const float den = 1.219f + std::pow(Yi, 0.4f);
        const float sf  = (1.0f / Yw) * std::pow((num / den), 2.5f);

        for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
        {
            this->image[i].g *= sf;
        }
    }
    /* Hostogram based local tone mapping function */
    else if (tone_map == tone_mapping_mode_t::local_histogram)
    {
        this->histogram_tone_map(false);
    }
    /* Hostogram based local tone mapping function with human based contrast limits */
    else if (tone_map == tone_mapping_mode_t::local_human_histogram)
    {
        this->histogram_tone_map(true);
    }
    /* Non-linear tone mapping */
    else if (tone_map == tone_mapping_mode_t::global_non_linear)
    {
        const float sf = key / Yi;
        maxY *= sf;
        maxY  = 1.0f / (maxY * maxY);

        for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
        {
            this->image[i].g *= sf;
            if (maxY)
            {
                this->image[i].g *= (1.0f + this->image[i].g * maxY) / (1.0f + this->image[i].g);
            }
        }
    }
    /* Exposure based tone mapping */
    else if (tone_map == tone_mapping_mode_t::global_exposure)
    {
        const float sf = key / Yi;
        maxY *= sf;

        for (int i = 0; i < static_cast<int>(this->x_res * this->y_res); i++)
        {
            this->image[i].g *= sf;
            if (maxY)
            {
                this->image[i].g = 1.0f - exp(-this->image[i].g);
            }
        }
    }
    /* Average luminance based tone mapping */
    else if (tone_map == tone_mapping_mode_t::global_avg_luminance)
    {
        /* Map average luminance to 0.5 */
        float sf = 1.0f;
        if (rgbAvg)
        {
            sf *= key * (0.5f / (rgbAvg * 683.0f));
        }

        for (int i = 0; i < static_cast<int>(this->x_res * this->y_res); i++)
        {
            this->image[i].g *= sf;
        }
    }
    /* Maximum luminance based tone mapping */
    else if (tone_map == tone_mapping_mode_t::global_max_luminance)
    {
        const float sf = key / maxY;
        for (int i = 0; i < static_cast<int>(this->x_res * this->y_res); i++)
        {
            this->image[i].g *= sf;
        }
    }
    else if (tone_map == tone_mapping_mode_t::global_bilateral_filter)
    {
        const float s_s = 0.02f * std::min(this->x_res, this->y_res);
        this->bilateral_filter_tone_map(0.4f, s_s, 0.4f, s_s);
    }
    else if (tone_map == tone_mapping_mode_t::global_ferwerda)
    {
        const float lda      = 800.0f;
        const float log_lda  = log(lda * 0.5f);
        const float log_la   = log(Yi);

        /* Cone response */
        float tp;
        if (log_la <= -2.6f)
        {
            tp = -0.72f;
        }
        else if (log_la <= 1.9f)
        {
            tp = std::pow(((0.249f * log_la) + 0.65f), 2.7f) - 0.72f;
        }
        else
        {
            tp = log_la - 1.255f;
        }
        const float mp   = (log_lda - 1.255f) / tp;

        /* Rod response */
        float ts;
        if (log_la <= -3.94f)
        {
            ts = -2.86f;
        }
        else if (log_la < -1.44f)
        {
            ts = std::pow(((0.405f * log_la) + 1.6f), 2.18f) - 2.86f;
        }
        else
        {
            ts = log_la - 0.395f;
        }
        const float ms = (log_lda - 1.255f) / ts;
        
        float k;
        if (log_la < -2.0f)
        {
            k = 1.0f;
        }
        else if (log_la < 1.0f)
        {
            k = 1.0f - ((log_la + 2.0f) * (1.0f / 3.0f));
        }
        else
        {
            k = 0.0f;
        }

        for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
        {
            const float ldp  = mp * this->image[i].g;
            const float lds  = ms * this->image[i].g;
            this->image[i].g = (1.0f / lda) * (ldp + (k * lds));
        }
    }
    /* No tone mapping */
    else
    {
        /* Remove the k conversion factor */
        const float sf = 1.0f / 683.0f;
        for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
        {
            this->image[i].g *= sf;
        }
    }

    /* Stretch low contrast or over compressed images */
    if (sc)
    {
        /* Find max and min luminance value */
        float maxY2 = -MAX_DIST;
        float minY2 =  MAX_DIST;
        for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
        {
            maxY2 = std::max(maxY2, this->image[i].g);
            minY2 = std::min(minY2, this->image[i].g);
        }
      
        if (((minY2 > 0.0f) || (maxY2 < 0.0f)) && (minY2 < maxY2))
        {
            const float offset = std::min(-minY2, 0.0f);
            const float scale  = 1.0f / (std::min(1.0f, maxY2) - std::max(0.0f, minY2));
            for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
            {
                this->image[i].g = (this->image[i].g + offset) * scale;
            }
        }
    }

    /* convert Yxy back to rgb */
    this->convert_Yxy_to_rgb();

    /* Add glare filter */
    if (gf)
    { 
        this->perform_glare_filter(Yi, 1.0f);
    }

    return *this;
}


// cppcheck-suppress unusedFunction
camera & camera::gamma_correct(const float gamma)
{
    for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
    {
        float r = this->image[i].r / 255.0f;
        float g = this->image[i].g / 255.0f;
        float b = this->image[i].b / 255.0f;

        /* Saturate */
        r = std::min(std::max(r, 0.0f), 1.0f);
        g = std::min(std::max(g, 0.0f), 1.0f);
        b = std::min(std::max(b, 0.0f), 1.0f);

        /* Gamma correct */
        if (fabs(gamma - 2.2f) > 0.001)
        {
            /* regular gamma correction */
            r = std::pow(r, 1.0f / gamma);
            g = std::pow(g, 1.0f / gamma);
            b = std::pow(b, 1.0f / gamma);
        }
        else if (fabs(gamma - 1.0f) > 0.001)
        {
            /* sRGB standard gamma correction (similar to regular w/2.2) */
            r = (r <= 0.0031308f) ? (12.92f * r) : (1.055f * std::pow(r, (1.0f / 2.4f)) - 0.055f);
            g = (g <= 0.0031308f) ? (12.92f * g) : (1.055f * std::pow(g, (1.0f / 2.4f)) - 0.055f);
            b = (b <= 0.0031308f) ? (12.92f * b) : (1.055f * std::pow(b, (1.0f / 2.4f)) - 0.055f);
        }

        this->image[i].r = r * 255.0f;
        this->image[i].g = g * 255.0f;
        this->image[i].b = b * 255.0f;
    }

    return *this;
}


void camera::perform_glare_filter(const float Yi, const float Yw)
{
    /* Pick lighting condition */
    const float log10_yi = log10(Yi);
    light_level_t current_light_level;
    ext_colour_t *glare_filter;
    ext_colour_t **glare_filter_ptr;
    if (log10_yi < -2.0f)
    {
        std::cout << "lighting is scotopic" << std::endl;
        current_light_level = light_level_t::scotopic;
        glare_filter_ptr    = &this->scotopic_glare_filter;
    }
    else if (log10_yi < 0.6f)
    {
        std::cout << "lighting is mesopic" << std::endl;
        current_light_level = light_level_t::mesopic;
        glare_filter_ptr    = &this->mesopic_glare_filter;
    }
    else
    {
        std::cout << "lighting is photopic" << std::endl;
        current_light_level = light_level_t::photopic;
        glare_filter_ptr    = &this->photopic_glare_filter;
    }

    /* Generate filter if required */
    if ((*glare_filter_ptr) == nullptr)
    {
        this->generate_glare_filter(glare_filter_ptr, current_light_level);
    }
    glare_filter = (*glare_filter_ptr);

    const int picture_size   = (this->x_res * this->y_res);
    const int image_g_offset = picture_size;
    const int image_b_offset = picture_size << 1;

    /* Allocate data for FFT */
    float *image_in;
    fftwf_complex *image_out;
    fftwf_plan image_p;
    image_in    = (float *)fftwf_malloc(picture_size * (3 * sizeof(float)));
    image_out   = (fftwf_complex *)fftwf_malloc(picture_size * (3 * sizeof(fftwf_complex)));
    int n[2]    = { static_cast<int>(this->y_res), static_cast<int>(this->x_res) };
    image_p     = fftwf_plan_many_dft_r2c(2, n, 3, image_in, nullptr, 1, (this->x_res * this->y_res), image_out, nullptr, 1, (this->x_res * this->y_res), FFTW_ESTIMATE);
    
    float *glare_in;
    fftwf_complex *glare_out;
    fftwf_plan glare_p;
    glare_in    = (float *)fftwf_malloc(picture_size * (3 * sizeof(float)));
    glare_out   = (fftwf_complex *)fftwf_malloc(picture_size * (3 * sizeof(fftwf_complex)));
    glare_p     = fftwf_plan_many_dft_r2c(2, n, 3, glare_in, nullptr, 1, (this->x_res * this->y_res), glare_out, nullptr, 1, (this->x_res * this->y_res), FFTW_ESTIMATE);
     
    fftwf_plan inv_p;
    inv_p       = fftwf_plan_many_dft_c2r(2, n, 3, image_out, nullptr, 1, (this->x_res * this->y_res), image_in, nullptr, 1, (this->x_res * this->y_res), FFTW_ESTIMATE);

    /* Split input image into channels */
    for (int i = 0; i < picture_size; i++)
    {
        if ((this->image[i].r > 255.0f) || (this->image[i].g > 255.0f) || (this->image[i].b > 255.0f))
        {
            image_in[i                 ] = this->image[i].r;
            image_in[i + image_g_offset] = this->image[i].g;
            image_in[i + image_b_offset] = this->image[i].b;
        }
        else
        {
            image_in[i                 ] = 0.0f;
            image_in[i + image_g_offset] = 0.0f;
            image_in[i + image_b_offset] = 0.0f;
        }
    }

    /*  Split glare image into channels and offset centre to pixel 0 0 */
    const int gf_offset      = static_cast<int>((this->x_res >> 1) + (this->x_res * ((this->y_res - 1) >> 1)));
    for (int i = 0; i < gf_offset; i++)
    {
        glare_in[i                 ] = glare_filter[i + gf_offset].r;
        glare_in[i + image_g_offset] = glare_filter[i + gf_offset].g;
        glare_in[i + image_b_offset] = glare_filter[i + gf_offset].b;
    }

    for (int i = gf_offset; i < picture_size; i++)
    {
       glare_in[i                 ] = glare_filter[i - gf_offset].r;
       glare_in[i + image_g_offset] = glare_filter[i - gf_offset].g;
       glare_in[i + image_b_offset] = glare_filter[i - gf_offset].b;
    }

    /* FFT */
    fftwf_execute(glare_p);
    fftwf_execute(image_p);
        
    /* Complex multiply and inverse transfrom */
    for (int i = 0; i < (picture_size * 3); i++)
    {
        const float tmp = image_out[i][0];
        image_out[i][0] = (image_out[i][0] * glare_out[i][0]) - (image_out[i][1] * glare_out[i][1]);
        image_out[i][1] = (tmp             * glare_out[i][1]) + (image_out[i][1] * glare_out[i][0]);
    }
    fftwf_execute(inv_p);


    /* Merge the convolution and original image */
    const float pixel_divisor = 1.0f / static_cast<float>(picture_size * 255);
    for (int i = 0; i < picture_size; i++)
    {
        this->image[i].r += image_in[i]                  * pixel_divisor;
        this->image[i].g += image_in[i + image_g_offset] * pixel_divisor;
        this->image[i].b += image_in[i + image_b_offset] * pixel_divisor;
    }


    /* Clean up */
    fftwf_destroy_plan(image_p);
    fftwf_free(image_in);
    fftwf_free(image_out);

    fftwf_destroy_plan(glare_p);
    fftwf_free(glare_in);
    fftwf_free(glare_out);

    fftwf_destroy_plan(inv_p);

    return;
}


void camera::bilateral_filter_tone_map(const float r_s, const float s_s, const float r_sa, const float s_sa)
{
    /* Build log image */
    float *log_intensity = new float [ this->x_res * this->y_res ];
    float input_max = -MAX_DIST;
    float input_min =  MAX_DIST;
    for (int i = 0; i < static_cast<int>(this->x_res * this->y_res); i++)
    {
        if (this->image[i].g > 3.18e-7f)
        {
            log_intensity[i] = log10(this->image[i].g);
            input_max = std::max(input_max, log_intensity[i]);
            input_min = std::min(input_min, log_intensity[i]);
        }
    }


    /* Down sample */    
    const float input_delta  = input_max - input_min;
    const float sigma_r      = r_s / r_sa; 
    const float sigma_s      = s_s / s_sa;

    const int padding_xy    = (int)(2.0f * sigma_s) + 1;
    const int padding_z     = (int)(2.0f * sigma_r) + 1;

    const int small_width   = (int)((this->x_res - 1) / s_sa) + 1 + (padding_xy << 1);
    const int small_height  = (int)((this->y_res - 1) / s_sa) + 1 + (padding_xy << 1);
    const int small_depth   = (int)(input_delta / r_sa) + 1 + (padding_z << 1);

    float *wiw = (float *)fftwf_malloc(2 * small_width * small_height * small_depth * sizeof(float));
    memset(wiw,  0, (2 * small_width * small_height * small_depth * sizeof(float)));

    int iw_offset = (small_width * small_height * small_depth);
    for(int x = 0; x < (int)this->x_res; x++)
    {
        for(int y = 0; y < (int)this->y_res; y++)
        {
            const int small_x = (int)(static_cast<float>(x) / s_sa + 0.5f) + padding_xy;
            const int small_y = (int)(static_cast<float>(y) / s_sa + 0.5f) + padding_xy;
            const int small_z = (int)((log_intensity[x + (this->x_res * y)] - input_min) / r_sa + 0.5f) + padding_z;

            wiw[small_x + (small_width * (small_y + (small_height * small_z)))            ] += 1.0f;
            wiw[small_x + (small_width * (small_y + (small_height * small_z))) + iw_offset] += log_intensity[x + (this->x_res * y)];
        }
    }
    
    
    /* Build kernel */
    float *kernel = (float *)fftwf_malloc(small_width * small_height * small_depth * sizeof(float));

    const int half_width  = small_width  >> 1;
    const int half_height = small_height >> 1;
    const int half_depth  = small_depth  >> 1;
    for(int x = 0; x < small_width; x++)
    {
        const float X = x - ((x > half_width) ? small_width : 0.0f);

        for(int y = 0; y < small_height; y++)
        {
            const float Y = y - ((y > half_height) ? small_height : 0.0f);

            for(int z = 0; z < small_depth; z++)
            {
                const float Z = z - ((z > half_depth) ? small_depth : 0.0f);
                const float rr = (X * X + Y * Y) / (sigma_s * sigma_s) + Z * Z / (sigma_r * sigma_r);	
                kernel[x + (small_width * (y + (small_height * z)))] = exp(-rr * 0.5f);
            }
        }
    }


    /* Convolve */
    /* Plan FFT */
    int n[3]                = { small_width, small_height, small_depth };
    fftwf_complex *wiw_freq = (fftwf_complex *)fftwf_malloc(2 * small_width * small_height * small_depth * sizeof(fftwf_complex));
    fftwf_plan     fwd_p    = fftwf_plan_many_dft_r2c(3, n, 2, wiw,      nullptr, 1, (small_width * small_height * small_depth), wiw_freq, nullptr, 1, (small_width * small_height * small_depth), FFTW_ESTIMATE);
    fftwf_plan     inv_p    = fftwf_plan_many_dft_c2r(3, n, 2, wiw_freq, nullptr, 1, (small_width * small_height * small_depth), wiw,      nullptr, 1, (small_width * small_height * small_depth), FFTW_ESTIMATE);


    fftwf_complex *k_freq   = (fftwf_complex *)fftwf_malloc(small_width * small_height * small_depth * sizeof(fftwf_complex));
    fftwf_plan     k_plan   = fftwf_plan_dft_r2c_3d(small_width, small_height, small_depth, kernel, k_freq, FFTW_ESTIMATE);

    /* FFT kernel */
    fftwf_execute(k_plan);
    
    /* Convolve i and iw */
    fftwf_execute(fwd_p);
    const float s = 1.0f / (small_width * small_height * small_depth);
    for(int i = 0; i < (small_width * small_height * small_depth); i++)
    {
        float tmp = wiw_freq[i][0];
        wiw_freq[i][0] = ((k_freq[i][0] * tmp           ) - (k_freq[i][1] * wiw_freq[i][1])) * s;
        wiw_freq[i][1] = ((k_freq[i][0] * wiw_freq[i][1]) + (k_freq[i][1] * tmp           )) * s;

        tmp = wiw_freq[i + iw_offset][0];
        wiw_freq[i + iw_offset][0] = ((k_freq[i][0] * tmp                       ) - (k_freq[i][1] * wiw_freq[i + iw_offset][1])) * s;
        wiw_freq[i + iw_offset][1] = ((k_freq[i][0] * wiw_freq[i + iw_offset][1]) + (k_freq[i][1] * tmp                       )) * s;
    }
    fftwf_execute(inv_p);


    /* Apply non-linearities */
    float *result = new float [this->x_res * this->y_res];
    float max_value = -MAX_DIST;
    float min_value =  MAX_DIST;
    for(int x = 0; x < (int)this->x_res; x++)
    {
        for(int y = 0; y < (int)this->y_res; y++)
        {
            const float z = log_intensity[x + (this->x_res * y)] - input_min;

            const float x_addr   = static_cast<float>(x) / s_sa + padding_xy;
            const int x_index   = std::max(0, std::min(static_cast<int>(x_addr),    small_width - 1));
            const int xx_index  = std::max(0, std::min(x_index + 1,                 small_width - 1));
    
            const float y_addr   = static_cast<float>(y) / s_sa + padding_xy;
            const int y_index   = std::max(0, std::min(static_cast<int>(y_addr),    small_height - 1));
            const int yy_index  = std::max(0, std::min(y_index + 1,                 small_height - 1));

            const float z_addr   = static_cast<float>(z) / r_sa + padding_z;
            const int z_index   = std::max(0, std::min(static_cast<int>(z_addr),    small_depth - 1));
            const int zz_index  = std::max(0, std::min(z_index + 1,                 small_depth - 1));

            const float x_alpha  = x_addr - x_index;
            const float y_alpha  = y_addr - y_index;
            const float z_alpha  = z_addr - z_index;

            const float IW = 
                (1.0f - x_alpha) * (1.0f - y_alpha) * (1.0f - z_alpha) * wiw[x_index  + (small_width * (y_index  + (small_height * z_index ))) + iw_offset] +
                x_alpha          * (1.0f - y_alpha) * (1.0f - z_alpha) * wiw[xx_index + (small_width * (y_index  + (small_height * z_index ))) + iw_offset] +
                (1.0f - x_alpha) * y_alpha          * (1.0f - z_alpha) * wiw[x_index  + (small_width * (yy_index + (small_height * z_index ))) + iw_offset] +
                x_alpha          * y_alpha          * (1.0f - z_alpha) * wiw[xx_index + (small_width * (yy_index + (small_height * z_index ))) + iw_offset] +
                (1.0f - x_alpha) * (1.0f - y_alpha) * z_alpha          * wiw[x_index  + (small_width * (y_index  + (small_height * zz_index))) + iw_offset] +
                x_alpha          * (1.0f - y_alpha) * z_alpha          * wiw[xx_index + (small_width * (y_index  + (small_height * zz_index))) + iw_offset] +
                (1.0f - x_alpha) * y_alpha          * z_alpha          * wiw[x_index  + (small_width * (yy_index + (small_height * zz_index))) + iw_offset] +
                x_alpha          * y_alpha          * z_alpha          * wiw[xx_index + (small_width * (yy_index + (small_height * zz_index))) + iw_offset];


            const float W = 
                (1.0f - x_alpha) * (1.0f - y_alpha) * (1.0f - z_alpha) * wiw[x_index  + (small_width * (y_index  + (small_height * z_index )))] +
                x_alpha          * (1.0f - y_alpha) * (1.0f - z_alpha) * wiw[xx_index + (small_width * (y_index  + (small_height * z_index )))] +
                (1.0f - x_alpha) * y_alpha          * (1.0f - z_alpha) * wiw[x_index  + (small_width * (yy_index + (small_height * z_index )))] +
                x_alpha          * y_alpha          * (1.0f - z_alpha) * wiw[xx_index + (small_width * (yy_index + (small_height * z_index )))] +
                (1.0f - x_alpha) * (1.0f - y_alpha) * z_alpha          * wiw[x_index  + (small_width * (y_index  + (small_height * zz_index)))] +
                x_alpha          * (1.0f - y_alpha) * z_alpha          * wiw[xx_index + (small_width * (y_index  + (small_height * zz_index)))] +
                (1.0f - x_alpha) * y_alpha          * z_alpha          * wiw[x_index  + (small_width * (yy_index + (small_height * zz_index)))] +
                x_alpha          * y_alpha          * z_alpha          * wiw[xx_index + (small_width * (yy_index + (small_height * zz_index)))];

            result[x + (this->x_res * y)] = IW / W;
            max_value = std::max(max_value, result[x + (this->x_res * y)]);
            min_value = std::min(min_value, result[x + (this->x_res * y)]);
        }
    }


    /* Scale the image */
    const float contrast = 300.0f;
    const float gamma = log10(contrast) /  (max_value - min_value);
    for(int i = 0; i < (int)(this->x_res * this->y_res); i++)
    {
        const float scaled_value = std::pow(10.0f, (result[i] * gamma + (log_intensity[i] - result[i])));
        this->image[i].g = scaled_value * (1.0f / 255.0f);
    }
    
    
    /* Clean up */
    delete [] log_intensity;
    delete [] result;

    fftwf_free(wiw);
    fftwf_free(wiw_freq);
    fftwf_free(kernel);
    fftwf_free(k_freq);

    fftwf_destroy_plan(fwd_p);
    fftwf_destroy_plan(inv_p);
    fftwf_destroy_plan(k_plan);

    return;
}


float camera::just_noticable_difference(const float La) const
{
    const float lLa = log10(La);
    float lLt;
    if (lLa < -3.94f)
    {
        lLt = -2.86f;
    }
    else if (lLa < -1.44f)
    {
        lLt = std::pow(0.405f * lLa + 1.6f, 2.18f) - 2.86f;
    }
    else if (lLa < -0.0184f)
    {
        lLt = lLa - 0.395f;
    }
    else if (lLa < 1.9f)
    {
        lLt = std::pow(0.249f * lLa + 0.65f, 2.7f) - 0.72f;
    }
    else
    {
        lLt = lLa - 1.255f;
    }
    
    return std::pow(10.0f, lLt);
}


void camera::histogram_tone_map(const bool h)
{
    /* Scale the image to one pixel per view degree */
    const float x_angle = (-2.0f * this->x_m) / this->t;
    const float y_angle = (-2.0f * this->y_m) / this->t;
    const int scaled_x_res = static_cast<int>(x_angle * (360.0f / (2.0f * PI)));
    const int scaled_y_res = static_cast<int>(y_angle * (360.0f / (2.0f * PI)));

    float * scaled_luminance = new float [scaled_x_res * scaled_y_res];
    memset(scaled_luminance, 0, (scaled_x_res * scaled_y_res * sizeof(float)));
    
    const float scaled_x_inc = static_cast<float>(this->x_res) / static_cast<float>(scaled_x_res);
    const float scaled_y_inc = static_cast<float>(this->y_res) / static_cast<float>(scaled_y_res);
    float y0 = 0.0f;
    float y1 = scaled_y_inc;
    for (int j = 0; j < scaled_y_res; ++j)
    {
        float x0 = 0.0f;
        float x1 = scaled_x_inc;
        
        for (int i = 0; i < scaled_x_res; ++i)
        {
            /* Average the 1 degree block to one pixel */
            for (int y = static_cast<int>(y0); y <= static_cast<int>(y1); ++y)
            {
                for (int x = static_cast<int>(x0); x <= static_cast<int>(x1); ++x)
                {
                    scaled_luminance[(j * scaled_x_res) + i] += this->image[(y * this->x_res) + x].g;
                }
                
            }
            
            scaled_luminance[(j * scaled_x_res) + i] /= ((x1 - x0) + 1) * ((y1 - y0) + 1);
            
            x0 = x1;
            x1 = std::min((this->x_res - 1.0f), (x1 + scaled_x_inc));
        }

        y0 = y1;
        y1 = std::min((this->y_res - 1.0f), (y1 + scaled_y_inc));
    }


    /* Find the range for the data, above 0 */
    const int bins = 100;
    int pix_gt0 = 0;
    float lw_min = MAX_DIST;
    float lw_max = 0.0f;
    for (int i = 0; i< (scaled_x_res * scaled_y_res); i++)
    {
        if (scaled_luminance[i] > 0.0f)
        {
            lw_min = std::min(lw_min, scaled_luminance[i]);
            lw_max = std::max(lw_max, scaled_luminance[i]);

            ++pix_gt0;
        }
    }
    
    /* Black image, no further work required */
    if (lw_min <= 0.0f)
    {
        std::cout << "black image" << std::endl;
        delete [] scaled_luminance;
        return;
    }

    /* Convert to log scale */
    const float min_lum  = 1.0e-4f;
    lw_min              = std::max(lw_min, min_lum);
    float bw_min         = log(lw_min);
    float bw_max         = log(lw_max);
    if (bw_max == bw_min)
    {
        bw_max += 0.0001f * fabs(bw_max);
    }

    /* Check for image within displayable range */
    const float ld_max = 300.0f;
    const float ld_min = 1.0f;
    const float bd_max = log(ld_max);
    const float bd_min = log(ld_min);
    if ((bd_max - bd_min) > (bw_max - bw_min))
    {
        std::cout << "using linear scale factor" << std::endl;
        const float sf = (bd_max - bd_min) / (bw_max - bw_min);
        for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
        {
            this->image[i].g = this->image[i].g * sf + bd_min;
        }

        delete [] scaled_luminance;
        return;
    }

    /* Populate the histogram */
    int  his[(bins + 1)];
    float cdf[(bins + 1)];
    memset(his, 0, ((bins + 1) * sizeof(int )));
    memset(cdf, 0, ((bins + 1) * sizeof(float)));
    
    int scaled_pixel_gt0 = 0;
    for (int i = 0; i < (scaled_x_res * scaled_y_res); i++)
    {
        if (scaled_luminance[i] > 0.0)
        {
            int b = static_cast<int>((log(scaled_luminance[i]) - bw_min) / (bw_max - bw_min) * bins) + 1;
            b = std::min(std::max(b, 0), bins);
            his[b]++;

            scaled_pixel_gt0++;
        }
    }

    /* Take average luminance */
    const float lw_avg = this->adatption_level;//max(lw_avg, min_lum);

    /* Populate the cumalitive density function */
    float sum = 0.0f;
    for (int i = 1; i <= bins; i++)
    {
        sum += his[i];
        cdf[i] = sum / static_cast<float>(pix_gt0);
    }

    
    /* Adjust the histogram */
    const float tolerance = 0.025f * static_cast<float>(pix_gt0);
    const float bin_step_size = (bw_max - bw_min) / bins;
    const float lw_step = 1.0f / static_cast<float>(bins) * (bw_max - bw_min);
    int trimmings;
    do
    {
        trimmings = 0;

        /* Convergence will fail, but there is no need to compression anyway */
        /* Apply a linear scale */
        if (pix_gt0 < tolerance)
        {
            std::cout << "using contrast based scale factor" << std::endl;
            const float num = 1.219f + std::pow(ld_max * 0.5f, 0.4f);
            const float den = 1.219f + std::pow(lw_avg , 0.4f);

            const float sf  = (1.0f / ld_max) * std::pow((num / den), 2.5f);

            for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
            {
                this->image[i].g *= sf;
            }

            delete [] scaled_luminance;
            return;
        }

        for (int i = 1; i <= bins; i++)
        {
            /* Calculate the ceiling */
            float ceiling = (pix_gt0 * bin_step_size) / (bd_max - bd_min);

            /* Adjust based on human perception */
            if (h)
            {
                /* Displaybale brightness */
                const float bde = bd_min + (bd_max - bd_min) * cdf[i];
                const float ld = exp(bde);

                const float bw = i * lw_step + bw_min;
                const float lw = exp(bw);
                ceiling *= (this->just_noticable_difference(ld) * lw) / (this->just_noticable_difference(lw) * ld);
            }

            /* Clip out of range values */
            if (his[i] > ceiling)
            {
                trimmings += (his[i] - (int)ceiling);
                his[i] = (int)ceiling;
            }
        }

        /* Re-calculate the cumalitive density function */
        pix_gt0 = 0;
        for (int i = 1; i <= bins; i++)
        {
            pix_gt0 += his[i];
        }

        float sum = 0.0f;
        for (int i = 1; i <= bins; i++)
        {
            sum += his[i];
            cdf[i] = sum / static_cast<float>(pix_gt0);
        }

    } while (trimmings > tolerance);


    /* Tone map the original image */
    for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
    {
        if (this->image[i].g > 0.0f)
        {
            const float bw = log(this->image[i].g);

            float bf = (bw - bw_min) / (bw_max - bw_min) * bins;
            bf = std::min(std::max(bf, 0.0f), static_cast<float>(bins));
            const int b0 = (int)bf;
            const int b1 = std::min((b0 + 1), bins);
            const float s = bf - b0;
            const float pbw = (1 - s) * cdf[b0] + s * cdf[b1];
    
            const float bde = bd_min + (bd_max - bd_min) * pbw;
            this->image[i].g = exp(bde);
            this->image[i].g /= ld_max;
        }
    }

    delete [] scaled_luminance;
    return;
}


void camera::convert_xyz_to_rgb(ext_colour_t *const p, const int x, const int y) const
{
    for (int i = 0; i < (int)(x * y); i++)
    {
        const ext_colour_t xyz = p[i];

        /* Conversion matrix assumes Y is in the range 0-1.0 and scales up by 255 */
        p[i].r = ( 826.353f  * xyz.r) + (-391.986f * xyz.g) + (-127.143f  * xyz.b);
        p[i].g = (-247.0695f * xyz.r) + ( 478.329f * xyz.g) + (  10.5825f * xyz.b);
        p[i].b = ( 14.2035f  * xyz.r) + ( -52.02f  * xyz.g) + ( 269.535f  * xyz.b);
    }

    return;
}


void camera::convert_Yxy_to_rgb()
{
    for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
    {
        ext_colour_t yxy = this->image[i];

        /* Convert to CIE xyz */
        const float x = yxy.r;
        const float y = yxy.b;
        yxy.r = x * yxy.g / y;
        yxy.b = (1.0f - x - y) * yxy.g / y;

        /* Conversion matrix assumes Y is in the range 0-1.0 and scales up by 255 */
        this->image[i].r = ( 826.353f  * yxy.r) + (-391.986f * yxy.g) + (-127.143f  * yxy.b);
        this->image[i].g = (-247.0695f * yxy.r) + ( 478.329f * yxy.g) + (  10.5825f * yxy.b);
        this->image[i].b = ( 14.2035f  * yxy.r) + ( -52.02f  * yxy.g) + ( 269.535f  * yxy.b);
    }

    return;
}


const camera & camera::write_tga_file(const std::string &file_name, unsigned char *o) const
{
    /* Open output file */
    std::ofstream tga_output(file_name.c_str(), std::ios::out);

    /* Clip image */
    unsigned char * tga_data;
    if (o == nullptr)
    {
        tga_data = new unsigned char [this->out_x_res * this->out_y_res * 3];
        this->clip_image_to_bgr(tga_data);
    }
    else
    {
        tga_data = o;
    }

    std::cout << "Writing snapshot taken to " << file_name << std::endl;

    /* Write the header to the tga file */
    tga_output << (char)0;                                                          /* Length of the ID section                                 */
    tga_output << (char)0;                                                          /* 0 - colour map not needed, 1 - colour map needed         */
    tga_output << (char)2;                                                          /* Uncompressed RGB image                                   */
    tga_output << (char)0 << (char)0;                                               /* Colour map specification                                 */
    tga_output << (char)0 << (char)0;
    tga_output << (char)0;
    tga_output << (char)0 << (char)0;                                               /* X origin or the image (bottom left corner)               */
    tga_output << (char)0 << (char)0;                                               /* Y origin or the image (bottom left corner)               */
    tga_output << (char)(this->out_x_res & 0xff) << (char)((this->out_x_res >> 8) & 0xff);  /* Image width  */
    tga_output << (char)(this->out_y_res & 0xff) << (char)((this->out_y_res >> 8) & 0xff);  /* Image height                                             */
    tga_output << (char)24;                                                         /* Pixel depth                                              */
    tga_output << (char)0;                                                          /* Image descriptor [ 0 0 Origin[2] Alpha Channel Bits[4] ] */
                                                                                    /* Origin has the following format:                         */
                                                                                    /*  00 - Bottom Left                                        */
                                                                                    /*  01 - Bottom Right                                       */
                                                                                    /*  10 - Top Left                                           */
                                                                                    /*  11 - Top Right                                          */

    /* Write the picture data to the tag file */
    tga_output.write((char *)tga_data, (this->out_x_res * this->out_y_res * 3));

    /* Write the footer to the tga file */
    tga_output << (char)0; tga_output << (char)0;
    tga_output << (char)0; tga_output << (char)0;
    tga_output << (char)0; tga_output << (char)0;
    tga_output << (char)0; tga_output << (char)0;

    /* Clean up */
    tga_output.close();
    delete [] tga_data;
    
    return *this;
}


void write_png_file(const std::string &file_name, unsigned char *png_data, const unsigned int x, const unsigned int y)
{
    /* Open output file */
    std::cout << "Writing snapshot taken to " << file_name << std::endl;

    /* Initialise png objects */
    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    assert((png_ptr != nullptr) || !"Error creating png write struct");

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (info_ptr == nullptr)
    {
        png_destroy_write_struct(&png_ptr,  (png_infopp)nullptr);
        assert(!"Error creating png info struct");
    }

    if (setjmp(png_jmpbuf(png_ptr)))
    {
        /* If we get here, we had a problem reading the file */
        png_destroy_write_struct(&png_ptr, &info_ptr);
        assert(!"Error setting png error handler");
    }

    /* Open output file */
    FILE *fp;
    if ((fp = fopen(file_name.c_str(), "wb")) == nullptr)
    {
    	std::cout << "Error opening png file " << file_name << " for output" << std::endl;
    	assert(!"Cannot open file");
    }

    /* Set compression parameters */
    png_init_io(png_ptr, fp);
    png_set_IHDR(png_ptr, info_ptr, x, y, 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

    /* Compress */
    png_write_info(png_ptr, info_ptr);

    png_bytep row_pointers[y];
    const unsigned int row_stride = x * 3;
    unsigned int read_location = x * (y - 1) * 3;
    for (unsigned int i = 0; i < y; ++i)
    {
        row_pointers[i] = &png_data[read_location];
        read_location -= row_stride;
    }
    png_write_image(png_ptr, row_pointers);

    /* Clean up */
    png_write_end(png_ptr, info_ptr);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    fclose(fp);
   
    return;
}


const camera & camera::write_png_file(const std::string &file_name) const
{
    /* Convert data to rgb */
    unsigned char * png_data = new unsigned char [this->out_x_res * this->out_y_res * 3];
    this->clip_image_to_rgb(png_data);
        
    raptor_raytracer::write_png_file(file_name, &png_data[0], this->out_x_res, this->out_y_res);

    delete [] png_data;
    return *this;
}


void read_png_file(const std::string &file_name, unsigned char *png_data, unsigned int *const x, unsigned int *const y)
{
    /* Open output file */
    std::cout << "Reading png from " << file_name << std::endl;

    /* Open file and check it is a png file */
    FILE *fp;
    if ((fp = fopen(file_name.c_str(), "rb")) == nullptr)
    {
        std::cout << "Error opening png file " << file_name << " for input" << std::endl;
        assert(!"Cannot open file");
    }

    png_byte header[8];
    assert((fread(header, 1, 8, fp) == 8) || !"Not enough bytes read for png header");
    assert(!png_sig_cmp(header, 0, 8) || !"File is not a png file");

    /* Initialise png objects */
    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    assert((png_ptr != nullptr) || !"Error creating png read struct");

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (info_ptr == nullptr)
    {
        png_destroy_read_struct(&png_ptr, (png_infopp)nullptr, (png_infopp)nullptr);
        assert(!"Error creating png info struct");
    }

    if (setjmp(png_jmpbuf(png_ptr)))
    {
        /* If we get here, we had a problem reading the file */
        png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)nullptr);
        assert(!"Error setting png error handler");
    }

    /* Set compression parameters */
    png_init_io(png_ptr, fp);
    png_set_sig_bytes(png_ptr, 8);

    /* Decompress */
    png_read_info(png_ptr, info_ptr);

    /* Read image info */
    (*x) = png_get_image_width(png_ptr, info_ptr);
    (*y) = png_get_image_height(png_ptr, info_ptr);

    assert((png_get_color_type(png_ptr, info_ptr) == PNG_COLOR_TYPE_RGB) || !"Unexpected colour type");
    assert((png_get_bit_depth(png_ptr, info_ptr) == 8) || !"Unexpected bit depth");
    assert((png_set_interlace_handling(png_ptr) == 1) || !"Unexpected interlace handling");
    png_read_update_info(png_ptr, info_ptr);

    /* Read data */
    if (setjmp(png_jmpbuf(png_ptr)))
    {
        /* If we get here, we had a problem reading the file */
        png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)nullptr);
        assert(!"Error setting png error handler");
    }

    png_bytep row_pointers[(*y)];
    const unsigned int row_stride = (*x) * 3;
    unsigned int write_location = (*x) * ((*y) - 1) * 3;
    for (unsigned int i = 0; i < (*y); ++i)
    {
        row_pointers[i] = &png_data[write_location];
        write_location -= row_stride;
    }

    png_read_image(png_ptr, row_pointers);

    /* Clean up */
    fclose(fp);
    png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)nullptr);

    return;
}

        
const camera & camera::write_jpeg_file(const std::string &file_name, const int q, unsigned char *o) const
{
    /* q must be in the range 0-100 */
    assert(q >=   0);
    assert(q <= 100);

    /* Clip image */
    unsigned char * jpeg_data;
    if (o == nullptr)
    {
        jpeg_data = new unsigned char [this->out_x_res * this->out_y_res * 3];
        this->clip_image_to_rgb(jpeg_data);
    }
    else
    {
        jpeg_data = o;
    }

    std::cout << "Writing snapshot taken to " << file_name << std::endl;

    /* Initialise jpeg objects */
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    /* Open output file */
    FILE * outfile;
    if ((outfile = fopen(file_name.c_str(), "wb")) == nullptr)
    {
    	std::cout << "Error opening jpeg file " << file_name << " for output" << std::endl;
    	assert(!"Cannot open file");
    }
    jpeg_stdio_dest(&cinfo, outfile);

    /* Set compression parameters */
    cinfo.image_width       = this->out_x_res;
    cinfo.image_height      = this->out_y_res;
    cinfo.input_components  = 3;
    cinfo.in_color_space    = JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, q, TRUE);

    /* Compress */
    JSAMPROW row_pointer[1];
    jpeg_start_compress(&cinfo, TRUE);
    const int row_stride = this->out_x_res * 3;
    int read_location = this->out_x_res * (this->out_y_res - 1) * 3;
    while (cinfo.next_scanline < cinfo.image_height)
    {
        row_pointer[0] = &jpeg_data[read_location];
        read_location -= row_stride;
        (void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    /* Clean up */
    jpeg_finish_compress(&cinfo);
    fclose(outfile);
    jpeg_destroy_compress(&cinfo);

    delete [] jpeg_data;
    
    return *this;
}
}; /* namespace raptor_raytracer */
