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
inline fp_t f0(const fp_t t)
{
    const fp_t tmp = t * (fp_t)(1.0 / 0.02);
    return (fp_t)2.61e6 * exp(-(tmp * tmp));
}


inline fp_t f1(const fp_t t)
{
    const fp_t tmp = t + (fp_t)0.02;
    return (fp_t)20.91 / (tmp * tmp * tmp);
}

inline fp_t f2(const fp_t t)
{
    const fp_t tmp = t + (fp_t)0.02;
    return (fp_t)72.37 / (tmp * tmp);
}

inline fp_t f3(const fp_t t, const fp_t l)
{
    const fp_t tmp = t - (fp_t)3.0 * l * (fp_t)(1.0 / 568.0);
    return (fp_t)436.9 * ((fp_t)568.0 / l) * exp((fp_t)-19.75 * (tmp * tmp));
}

struct scotopic_corona_t
{
    inline ext_colour_t operator()(const fp_t x, const fp_t y, const fp_t p) const
    {
        const fp_t dist = sqrt((x * x) + (y * y));
        const fp_t deg  = dist * p;
        const fp_t a    = ((fp_t)0.478 * f1(deg)) + ((fp_t)0.207 * f2(deg));
        return ext_colour_t(a,a,a);
    }
};


struct scotopic_halo_t
{
    inline ext_colour_t operator()(const fp_t x, const fp_t y, const fp_t p) const
    {
        const fp_t dist = sqrt((x * x) + (y * y));
        const fp_t deg  = dist * p;
        return (fp_t)0.033 * ext_colour_t(f3(deg, (fp_t)611.0), f3(deg, (fp_t)549.0), f3(deg, (fp_t)467.0));
    }
};


struct scotopic_bloom_t
{
    inline ext_colour_t operator()(const fp_t x, const fp_t y, const fp_t p) const
    {
        const fp_t dist = sqrt((x * x) + (y * y));
        const fp_t deg  = dist * p;
        const fp_t a    = (fp_t)0.282 * f0(deg);
        return ext_colour_t(a,a,a);
    }
};


struct mesopic_corona_t
{
    inline ext_colour_t operator()(const fp_t x, const fp_t y, const fp_t p) const
    {
        const fp_t dist = sqrt((x * x) + (y * y));
        const fp_t deg  = dist * p;
        const fp_t a    = ((fp_t)0.478 * f1(deg)) + ((fp_t)0.138 * f2(deg));
        return ext_colour_t(a, a, a);
    }
};


struct mesopic_halo_t
{
    inline ext_colour_t operator()(const fp_t x, const fp_t y, const fp_t p) const
    {
        const fp_t dist = sqrt((x * x) + (y * y));
        const fp_t deg  = dist * p;
        return (fp_t)0.016 * ext_colour_t(f3(deg, (fp_t)611.0), f3(deg, (fp_t)549.0), f3(deg, (fp_t)467.0)); 
    }
};


struct mesopic_bloom_t
{
    inline ext_colour_t operator()(const fp_t x, const fp_t y, const fp_t p) const
    {
        const fp_t dist = sqrt((x * x) + (y * y));
        const fp_t deg  = dist * p;
        const fp_t a    = (fp_t)0.368 * f0(deg);
        return ext_colour_t(a,a,a);
    }
};


struct photopic_corona_t
{
    inline ext_colour_t operator()(const fp_t x, const fp_t y, const fp_t p) const
    {
        const fp_t dist = sqrt((x * x) + (y * y));
        const fp_t deg  = dist * p;
        const fp_t a    = ((fp_t)0.478 * f1(deg)) + ((fp_t)0.138 * f2(deg));
        return ext_colour_t(a,a,a);
    }
};


struct photopic_halo_t
{
    inline ext_colour_t operator()(const fp_t x, const fp_t y, const fp_t p) const
    {
        return ext_colour_t((fp_t)0.0, (fp_t)0.0, (fp_t)0.0);
    }
};


struct photopic_bloom_t
{
    inline ext_colour_t operator()(const fp_t x, const fp_t y, const fp_t p) const
    {
        const fp_t dist = sqrt((x * x) + (y * y));
        const fp_t deg  = dist * p;
        const fp_t a    = (fp_t)0.384 * f0(deg);
        return ext_colour_t(a,a,a);
    }
};


fp_t camera::generate_flare_lines(ext_colour_t *const f)
{
    fp_t sum = (fp_t)0.0;
    int nr_of_lines = 200;
    for (int i = 0; i < nr_of_lines; i++)
    {
        fp_t lineIntens = gen_random_mersenne_twister();
        fp_t lineRad = 1.0;//(gen_random_mersenne_twister() * 2.0);
    
        fp_t theta = ((i + gen_random_mersenne_twister()) * (2.0*PI)) / nr_of_lines;  // stratified
        fp_t dir[2] = { cos(theta), sin(theta) };
                
        int dj = (theta<=PI) ? 1 : -1;
        int j = (int)(this->y_res/2.0 - (lineRad-0.5)*dj);

        /* Write the intensity of the pixel for each row */
        while ((j >= 0) && (j < (int)this->y_res))
        {
            /* Calculate the x bounds of the line on this row */
            fp_t jminf = (j+0.0-this->y_res/2.0);
            fp_t jmaxf = (j+1.0-this->y_res/2.0);
            fp_t iminf = jminf/tan(theta);
            fp_t imaxf = jmaxf/tan(theta);

            // swap if iminf>imaxf
            if (iminf > imaxf)
            {
                fp_t temp = iminf;
                iminf = imaxf;
                imaxf = temp;
            }

            fp_t dif = fabs(lineRad/sin(theta));
            int imin = (int)(iminf-dif+0.5+this->x_res/2.0);
            int imax = (int)(imaxf+dif-0.5+this->x_res/2.0);

            /* Set the magnitude of each pixel on this row */
            for (int k = max(imin, 0); k < min((imax + 1), (int)this->x_res); k++)
            {
                // find pixel center in spatial coordinates
                fp_t p[2];
                p[0] = k-this->x_res/2.0+0.5;
                p[1] = j-this->y_res/2.0+0.5;

                // find distance of p to line
                fp_t dp = (p[0] * dir[0]) + (p[1] * dir[1]);
                fp_t dist;
                if (dp>0.0)
                {
                    fp_t vec[2] = { (p[0] - dp * dir[0]), (p[0] - dp * dir[0]) };
                    dist = sqrt((vec[0] * vec[0]) + (vec[1] * vec[1]));
                }
                else 
                {
                    dist = sqrt((p[0] * p[0]) + (p[1] * p[1]));
                }

                // tent filter
                if (dist>=lineRad)
                    continue;
                fp_t filt = 1.0/lineRad - 1.0/(lineRad*lineRad)*dist;
                
                fp_t len = sqrt((p[0] * p[0]) + (p[1] * p[1]));
                fp_t flo = max((fp_t)(len*2.0*PI)/nr_of_lines, (fp_t)1.0/nr_of_lines);
                fp_t val = filt*flo*lineIntens;
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
    const fp_t pixdeg = (atan(-this->x_m / this->t) * (fp_t)2.0) / this->x_res;

    for (int j = 0; j < (int)this->y_res; j++)
    {
        for (int i = 0; i < (int)this->x_res; i++)
        {
            // perform 2D trapezoidal integration
            fp_t w = (fp_t)i-this->x_res/2.0, x=w+1.0;
            fp_t y = (fp_t)j-this->y_res/2.0, z=y+1.0;
      
            // determine four corners, find ratio
            fp_t max, min;
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
        
            fp_t h = (x-w)/n;
            fp_t k = (z-y)/n;
        
            ext_colour_t sum2(0.0, 0.0, 0.0);
            for (int k = 1; k < n; k++)
            {
                fp_t x_k = (x-w)/(fp_t)n*k+w;
                fp_t y_k = (z-y)/(fp_t)n*k+y;
                sum2 += p(x_k, y, pixdeg) + 
                        p(x_k, z, pixdeg) + 
                        p(w, y_k, pixdeg) + 
                        p(x, y_k, pixdeg);
            }
            sum1 += ext_colour_t((fp_t)2.0) * sum2;

            sum2 = ext_colour_t(0.0, 0.0, 0.0);
            for (int k=1; k<n; k++)
            {
                for (int l=1; l<n; l++)
                {
                    fp_t x_k = (x-w)/(fp_t)n*k+w;
                    fp_t y_l = (z-y)/(fp_t)n*l+y;
                    sum2 += p(x_k, y_l, pixdeg);
                }
            }
            sum1 += ext_colour_t((fp_t)4.0) * sum2;
            sum1 *= ext_colour_t((fp_t)0.25 * h * k);

            f[(i + (j * this->x_res))].r *= sum1.r;
            f[(i + (j * this->x_res))].g *= sum1.g;
            f[(i + (j * this->x_res))].b *= sum1.b;
        }
    }
    
    return;
}


void camera::generate_glare_filter(ext_colour_t **gf, const light_level_t ll)
{
//    fp_t pixdeg = atan(-this->x_m / this->t) * (fp_t)2.0;
//    cout << "angle : " << pixdeg * (180.0/PI) << endl;

    (*gf) = new ext_colour_t[this->x_res * this->y_res * 3];


    /* Generate the flare lines for the lenticular halo and ciliary corona */
    const fp_t flare_sum = this->generate_flare_lines(&(*gf)[0]);

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
//            if (mb_sum.r > (fp_t)0.0)
//            {
//                mb_mul.r = (mb_size * mb_size) / mb_sum.r;
//            }
//
//            if (mb_sum.g > (fp_t)0.0)
//            {
//                mb_mul.g = (mb_size * mb_size) / mb_sum.g;
//            }
//
//            if (mb_sum.b > (fp_t)0.0)
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
    const fp_t bloom_lum = flare_sum / (this->x_res * this->y_res);
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
        case scotopic   : 
            this->apply_point_spreading_function(sc, &(*gf)[0]                            );
            this->apply_point_spreading_function(sh, &(*gf)[this->x_res * this->y_res    ]);
            this->apply_point_spreading_function(sb, &(*gf)[this->x_res * this->y_res * 2]);
            break;
        case mesopic    : 
            this->apply_point_spreading_function(mc, &(*gf)[0]                            );
            this->apply_point_spreading_function(mh, &(*gf)[this->x_res * this->y_res    ]);
            this->apply_point_spreading_function(mb, &(*gf)[this->x_res * this->y_res * 2]);
            break;
        case photopic   : 
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
    fp_t amountScattered = 0.0001;
    ext_colour_t mul((fp_t)1.0, (fp_t)1.0, (fp_t)1.0);
    if (sum.r > (fp_t)0.0)
    {
        mul.r = (amountScattered * this->x_res * this->y_res)/sum.r;
    }

    if (sum.g > (fp_t)0.0)
    {
        mul.g = (amountScattered * this->x_res * this->y_res)/sum.g;
    }

    if (sum.b > (fp_t)0.0)
    {
        mul.b = (amountScattered * this->x_res * this->y_res)/sum.b;
    }
    cout << "Sum = " << sum.r << ", " << sum.g << ", " << sum.b << endl;
    cout << "Mul = " << mul.r << "," << mul.g << "," << mul.b << endl;

    for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
    {
        (*gf)[i].r *= mul.r;
        (*gf)[i].g *= mul.g;
        (*gf)[i].b *= mul.b;
    }



    
//    cout << "dumping glare filter" << endl;
//    char * tga_data = new char [this->x_res * this->y_res * 3];
//    for (int i = 0; i < (int)this->x_res; i++)
//    {
//        for (int j = 0; j < (int)this->y_res; j++)
//        {
//            tga_data[((i + (j * this->x_res)) * 3)    ] = (char)min(max((*gf)[(i + (j * this->x_res))].r * (fp_t)2550.0, (fp_t)0.0), (fp_t)255.0);
//            tga_data[((i + (j * this->x_res)) * 3) + 1] = (char)min(max((*gf)[(i + (j * this->x_res))].g * (fp_t)2550.0, (fp_t)0.0), (fp_t)255.0);
//            tga_data[((i + (j * this->x_res)) * 3) + 2] = (char)min(max((*gf)[(i + (j * this->x_res))].b * (fp_t)2550.0, (fp_t)0.0), (fp_t)255.0);
//        }
//    }
//
//    /* Open output file */
//    ostringstream file_name(ostringstream::out);
//    file_name <<"glare_filter" << ".tga";
//    ofstream tga_output(file_name.str().c_str(), ios::out);
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


//void camera::draw_circle(ext_colour_t *o, const int x, const int y, const fp_t s, const fp_t v)
//{
//    const fp_t s_sq = s * s;
//    for (int j = 0; j < (int)(this->y_res * 4); j++)
//    {
//        const fp_t y_dist    = ((fp_t)j / (fp_t)4.0) - (fp_t)y;
//        const fp_t y_dist_sq = y_dist * y_dist;
//        
//        if (y_dist_sq < s_sq)
//        {
//            const fp_t x_dist_sq    = s_sq - y_dist_sq;
//            const fp_t x_dist       = sqrt(x_dist_sq);
//            const int  x_pixel_dist = (int)x_dist;
//            const fp_t min_x        = max((fp_t)x - x_dist, (fp_t)0.0);
//            const fp_t max_x        = min((fp_t)x + x_dist, (fp_t)this->x_res);
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
void camera::draw_circle(fp_t *o, const int x, const int y, const fp_t s, const fp_t v)
{
     /* Picture size and scale */
//    const fp_t picture_dimension    = (fp_t)9.0e-3; // 9mm
//    const fp_t pixel_resolution     = (fp_t)2.0e-6; // 1um per pixel
    const int picture_resolution    = 5000;
    const int picture_resolution_y  = 5000;

    const fp_t s_sq = s * s;
    for (int j = 0; j < (int)picture_resolution_y; j++)
    {
        const fp_t y_dist    = (fp_t)(j - y);
        const fp_t y_dist_sq = y_dist * y_dist;
        
        if (y_dist_sq < s_sq)
        {
            const fp_t x_dist_sq    = s_sq - y_dist_sq;
            const fp_t x_dist       = sqrt(x_dist_sq);
            const int  x_pixel_dist = (int)x_dist;
            const int  min_x        = max(x - x_pixel_dist, 0);
            const int  max_x        = min(x + x_pixel_dist, (int)picture_resolution);
            
            for (int i = min_x; i < max_x; i++)
            {
                o[i + (j * picture_resolution)] = v;
            }
        }
    }
    
    return;
}


// cppcheck-suppress unusedFunction
void camera::generate_temporal_psf(const fp_t y)
{
    /* Picture size and scale */
//    const fp_t picture_dimension    = (fp_t)9.0e-3; // 9mm
    const fp_t pixel_resolution     = (fp_t)2.0e-6; // 1um per pixel
    const int picture_resolution    = 5000;
    const int picture_resolution_y  = 5000;
    
    /* Allocate an array for the pupil, lens flare lines, lens particles, eyebrows, 
      vitreous particles and the combination of the componants */
    if (temporal_glare_filter == nullptr)
    {
        this->temporal_glare_filter = new ext_colour_t[picture_resolution * picture_resolution_y];
    }
    
    fp_t *image_componants = new fp_t [picture_resolution * picture_resolution_y * 6];
    
    /* Calculate the pupil diameter */
    const fp_t p        = (fp_t)4.9 - ((fp_t)3.0 * tanh((fp_t)0.4 * (log(y) + (fp_t)1.0)));
    const fp_t p_max    = 9.0;
    cout << "pupil: " << p << endl;
    
    const int noise_octaves = 3;
    fp_t frequency  = 1.0;
    fp_t amplitude  = 0.25;
    fp_t noise      = 0.0;
    fp_t fall_off   = 0.5;
    
    for (int i = 0; i < noise_octaves; i++)
    {
        fp_t x = ((this->time_step / p) * frequency);
    	int wx  = (int)((float)((int)x) + 0.5);
    	fp_t px = x - wx;

        /* Generate some smoothed noise */
    	int n = wx * 57;
    	n = (n << 13) ^ n;
    	fp_t v1a = (fp_t) (1.0 - ((n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0);

    	n = (wx - 1) * 57;
    	n = (n << 13) ^ n;
    	fp_t v1b = (fp_t) (1.0 - ((n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0);
        fp_t v1 = (v1a + v1b) * (fp_t)0.5;
            
    	n = (wx + 1) * 57;
    	n = (n << 13) ^ n;
    	fp_t v2a = (fp_t) (1.0 - ((n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0);

    	n = (wx + 2) * 57;
    	n = (n << 13) ^ n;
    	fp_t v2b = (fp_t) (1.0 - ((n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0);
        fp_t v2 = (v2a + v2b) * (fp_t)0.5;

        /* Interpolate */
    	fp_t ft = px * PI;
    	fp_t f = (1.0 - cos(ft)) * 0.5;
    	fp_t i1 = v1 * (1.0 - f) + v2 * f;

    	noise += (i1 * amplitude);
    	frequency *= 2.0;
    	amplitude *= fall_off;
    }
//    cout << "noise: " << noise << endl;

    const fp_t pixel_scale  = pixel_resolution;//(p_max * 10.0e-3) / picture_resolution_y;
    cout << "pixel_scale: " << pixel_scale << endl;

    const fp_t pupil_scale  = (fp_t)0.5e-3 / pixel_resolution;//(min(-this->x_m, -this->y_m) / p_max) * (picture_resolution_y / (-2.0 * this->y_m));
    const fp_t p_hippus     = (p + (noise * (p_max / p) * sqrt((fp_t)1.0 - (p / p_max)))) * pupil_scale;
//    cout << "hippus: " << p_hippus << endl;
    
    /* Draw the pupil */
    memset(&image_componants[0],  0, (picture_resolution * picture_resolution_y * sizeof(fp_t)));
    this->draw_circle(&image_componants[0], (picture_resolution >> 1), (picture_resolution_y >> 1), p_hippus, 255.0);


    /* Randomly place particle in the vitreous humour */
    for (int i = (int)(picture_resolution * picture_resolution_y); i < (int)(2 * picture_resolution * picture_resolution_y); i++)
    {
        image_componants[i] = (fp_t)255.0;
    }

    const int vitreous_humour_offset    = (picture_resolution * picture_resolution_y);
    const int vitreous_humour_particles = 100;
    for (int p = 0; p < vitreous_humour_particles; p++)
    {
        int x   = (int)(gen_random_mersenne_twister() * (fp_t)picture_resolution);
        int y   = (int)(gen_random_mersenne_twister() * (fp_t)picture_resolution_y);
        int sz  = (int)(((gen_random_mersenne_twister() * (fp_t)4.0e-6) + 1.0e-6) / pixel_resolution);
        this->draw_circle(&image_componants[vitreous_humour_offset], x, y, sz, 0.0);
    }


    /* Draw the lens fibers */
    for (int i = (int)(2 * picture_resolution * picture_resolution_y); i < (int)(3 * picture_resolution * picture_resolution_y); i++)
    {
        image_componants[i] = (fp_t)255.0;
    }

    const int  lens_fiber_offset    = (2 * picture_resolution * picture_resolution_y);
    const int  lens_fiber_lines     = 200;
    const fp_t inv_lens_fiber_lines = (fp_t)(1.0 / 200.0);
    const fp_t min_line_dist_sq     = (fp_t)(1.0 / 10.0) * picture_resolution_y * picture_resolution_y;
    for (int l = 0; l < lens_fiber_lines; l++)
    {
        const fp_t theta     = (((fp_t)l + gen_random_mersenne_twister()) * (fp_t)(2.0 * PI)) * inv_lens_fiber_lines;
        const fp_t x_y_ratio = sin(theta) / cos(theta);
                
        const fp_t dj = (theta <= PI) ? 1 : -1;
        for (fp_t j = 0; j < (fp_t)(picture_resolution_y / 2); j++)
        {
            const int y_addr = (int)((j * dj) + ((fp_t)picture_resolution_y * (fp_t)0.5));

            fp_t fmin_x = (j - (fp_t)0.5) * x_y_ratio;
            fp_t fmax_x = (j + (fp_t)0.5) * x_y_ratio;
            if (fmax_x < fmin_x)
            {
                const fp_t tmp = fmax_x;
                fmax_x = fmin_x;
                fmin_x = tmp;
            }
            int min_x   = (int)fmin_x;
            int max_x   = (int)fmax_x;

            for (int i = min_x; i <= max_x; i++)
            {
                const fp_t dist_sq = (i * i) + (j * j);

                const int x_addr = (int)((i * dj) + ((fp_t)picture_resolution * (fp_t)0.5));
                if ((dist_sq > min_line_dist_sq) && (x_addr < (int)picture_resolution) && (x_addr >= 0))
                {
                    const int addr = lens_fiber_offset + x_addr + (y_addr * picture_resolution);
                    image_componants[addr] = (fp_t)0.0;
                }
            }
        }
    }


    /* Draw the lens particles */
    for (int i = (int)(3 * picture_resolution * picture_resolution_y); i < (int)(4 * picture_resolution * picture_resolution_y); i++)
    {
        image_componants[i] = (fp_t)255.0;
    }

    const int lens_particle_offset  = (3 * picture_resolution * picture_resolution_y);
    const int lens_particles        = 750;
    for (int p = 0; p < lens_particles; p++)
    {
        int x   = (int)(gen_random_mersenne_twister() * (fp_t)picture_resolution);
        int y   = (int)(gen_random_mersenne_twister() * (fp_t)picture_resolution_y);
        int sz  = (int)(((gen_random_mersenne_twister() * (fp_t)5.0e-6) + 1.0e-6) / pixel_resolution);
        this->draw_circle(&image_componants[lens_particle_offset], x, y, sz, 0.0);
    }


    /* Draw the cornea particles */
    for (int i = (int)(4 * picture_resolution * picture_resolution_y); i < (int)(5 * picture_resolution * picture_resolution_y); i++)
    {
        image_componants[i] = (fp_t)255.0;
    }

    const int cornea_particle_offset  = (4 * picture_resolution * picture_resolution_y);
    const int cornea_particles        = 250;
    for (int p = 0; p < cornea_particles; p++)
    {
        int x   = (int)(gen_random_mersenne_twister() * (fp_t)picture_resolution);
        int y   = (int)(gen_random_mersenne_twister() * (fp_t)picture_resolution_y);
        int sz  = 15.0e-6 / pixel_resolution;
        this->draw_circle(&image_componants[cornea_particle_offset], x, y, sz, 0.0);
    }


    /* Sum the componants */
    const int complete_image_offset  = (5 * picture_resolution * picture_resolution_y);
    for (int i = 0; i < (int)(picture_resolution * picture_resolution_y); i++)
    {
        image_componants[complete_image_offset + i]  = image_componants[i];
        image_componants[complete_image_offset + i] += image_componants[i + vitreous_humour_offset];
        image_componants[complete_image_offset + i] += image_componants[i + lens_fiber_offset     ];
        image_componants[complete_image_offset + i] += image_componants[i + lens_particle_offset  ];
        image_componants[complete_image_offset + i] += image_componants[i + cornea_particle_offset];
       
        if (image_componants[complete_image_offset + i] < (fp_t)(255.0 * 5.0))
        {
            image_componants[complete_image_offset + i] = (fp_t)1.0;
        }
        else
        {
            image_componants[complete_image_offset + i] = (fp_t)0.0;
        }
    }

    /* Down sample to 1K X 1K */
    const int downsampled_resolution = 1000;
    const int x_samples = (picture_resolution   / downsampled_resolution);
    const int y_samples = (picture_resolution_y / downsampled_resolution);
    cout << picture_resolution << ", " << picture_resolution_y << endl;
    cout << x_samples << ", " << y_samples << endl;
    
    int image_addr = 0;
    for (int j = 0; j < downsampled_resolution; j++)
    {
        for (int i = 0; i < downsampled_resolution; i++)
        {
            fp_t sample = (fp_t)0.0;
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
    cout << "dumping complete image" << endl;
    char * tga_data = new char [downsampled_resolution * downsampled_resolution * 3];
    for (int i = 0; i < (int)downsampled_resolution; i++)
    {
        for (int j = 0; j < (int)downsampled_resolution; j++)
        {
            tga_data[((i + (j * downsampled_resolution)) * 3)    ] = (char)min(max(image_componants[complete_image_offset + (i + (j * downsampled_resolution))] * (fp_t)255.0, (fp_t)0.0), (fp_t)255.0);
            tga_data[((i + (j * downsampled_resolution)) * 3) + 1] = (char)min(max(image_componants[complete_image_offset + (i + (j * downsampled_resolution))] * (fp_t)255.0, (fp_t)0.0), (fp_t)255.0);
            tga_data[((i + (j * downsampled_resolution)) * 3) + 2] = (char)min(max(image_componants[complete_image_offset + (i + (j * downsampled_resolution))] * (fp_t)255.0, (fp_t)0.0), (fp_t)255.0);
        }
    }
    
    /* Open output file */
    ostringstream file_name(ostringstream::out);
    file_name <<"complete_image" << ".tga";
    ofstream tga_output(file_name.str().c_str(), ios::out);
    
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

    const fp_t retina_pixel_scale   = (fp_t)1.0e-9;
    const fp_t image_scale_factor   = retina_pixel_scale / pixel_scale;
//    const fp_t pupil_to_retina_dist = (fp_t)24.0e-3;
//    const fp_t lambda               = (fp_t)575.0e-9;
    const fp_t lambda_d             = 5.0e-4;//lambda * pupil_to_retina_dist;
    cout << "lambda_d          : " << lambda_d           << endl;
    cout << "image_scale_factor: " << image_scale_factor << endl;
    for (int j = 0; j < (int)downsampled_resolution; j++)
    {
        const fp_t yq = ((fp_t)j - ((fp_t)downsampled_resolution * (fp_t)0.5)) * (5.0 * pixel_scale) * (fp_t)0.5;
        for (int i = 0; i < (int)downsampled_resolution; i++)
        {
            const fp_t xp = ((fp_t)i - ((fp_t)downsampled_resolution * (fp_t)0.5)) * (5.0 * pixel_scale) * (fp_t)0.5;
            const fp_t exponential = ((PI / lambda_d) * ((xp * xp) + (yq * yq)));
            
            const int image_addr = i + (downsampled_resolution * j);

            const int xp_addr    = max(min((int)((xp / (5.0 * pixel_scale)) + ((fp_t)downsampled_resolution * (fp_t)0.5)), (int)downsampled_resolution - 1), 0);
            const int yq_addr    = max(min((int)((yq / (5.0 * pixel_scale)) + ((fp_t)downsampled_resolution * (fp_t)0.5)), (int)downsampled_resolution - 1), 0);
            const int pupil_addr = complete_image_offset + xp_addr + (downsampled_resolution * yq_addr);

            image_in[image_addr][0] = cos(exponential) * image_componants[pupil_addr];
            image_in[image_addr][1] = sin(exponential) * image_componants[pupil_addr];
        }
    }

#if 1
{
    cout << "dumping pre fft image" << endl;
    char * tga_data = new char [downsampled_resolution * downsampled_resolution * 3];
    for (int i = 0; i < (int)downsampled_resolution; i++)
    {
        for (int j = 0; j < (int)downsampled_resolution; j++)
        {
            tga_data[((i + (j * downsampled_resolution)) * 3)    ] = (char)min(max(image_in[i + (j * downsampled_resolution)][0] * (fp_t)255.0, (fp_t)0.0), (fp_t)255.0);
            tga_data[((i + (j * downsampled_resolution)) * 3) + 1] = (char)min(max(image_in[i + (j * downsampled_resolution)][0] * (fp_t)255.0, (fp_t)0.0), (fp_t)255.0);
            tga_data[((i + (j * downsampled_resolution)) * 3) + 2] = (char)min(max(image_in[i + (j * downsampled_resolution)][0] * (fp_t)255.0, (fp_t)0.0), (fp_t)255.0);
        }
    }
    
    /* Open output file */
    ostringstream file_name(ostringstream::out);
    file_name <<"pre_fft_image" << ".tga";
    ofstream tga_output(file_name.str().c_str(), ios::out);
    
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
    fp_t offset = (fp_t)1.0;
    for (int j = 0; j < (int)downsampled_resolution; j++)
    {
        for (int i = 0; i < (int)downsampled_resolution; i++)
        {
            image_in[i + (j * downsampled_resolution)][0] *= offset;
            image_in[i + (j * downsampled_resolution)][1] *= offset;
            offset *= (fp_t)-1.0;
        }
        offset *= (fp_t)-1.0;
    }
    
    /* FFT, take square magnitude and scale */    
    fftwf_execute(image_p);

    const fp_t fft_scale = (fp_t)1.0;// / (lambda_d * lambda_d);
    for (int i = 0; i < (int)(downsampled_resolution * downsampled_resolution); i++)
    {
        image_in[i][0] /= (downsampled_resolution * downsampled_resolution);
        image_in[i][1] /= (downsampled_resolution * downsampled_resolution);
        image_in[i][0] = ((image_out[i][0] * image_out[i][0]) + (image_out[i][1] * image_out[i][1])) * fft_scale;
    }

#if 1
{
    cout << "dumping fft image" << endl;
    char * tga_data = new char [downsampled_resolution * downsampled_resolution * 3];
    for (int i = 0; i < (int)downsampled_resolution; i++)
    {
        for (int j = 0; j < (int)downsampled_resolution; j++)
        {
            tga_data[((i + (j * downsampled_resolution)) * 3)    ] = (char)min(max(image_in[i + (j * downsampled_resolution)][0] / (fp_t)100.0, (fp_t)0.0), (fp_t)255.0);
            tga_data[((i + (j * downsampled_resolution)) * 3) + 1] = (char)min(max(image_in[i + (j * downsampled_resolution)][0] / (fp_t)100.0, (fp_t)0.0), (fp_t)255.0);
            tga_data[((i + (j * downsampled_resolution)) * 3) + 2] = (char)min(max(image_in[i + (j * downsampled_resolution)][0] / (fp_t)100.0, (fp_t)0.0), (fp_t)255.0);
        }
    }
    
    /* Open output file */
    ostringstream file_name(ostringstream::out);
    file_name <<"fft_image" << ".tga";
    ofstream tga_output(file_name.str().c_str(), ios::out);
    
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
    static const fp_t cie_xf[] = { (fp_t)(  14.0 / (4.0 * 106836.0)),  (fp_t)(   42.0 / (4.0 * 106836.0)), (fp_t)(  143.0 / (4.0 * 106836.0)), (fp_t)(  435.0 / (4.0 * 106836.0)), (fp_t)(1344.0 / (4.0 * 106836.0)),  
                                   (fp_t)(2839.0 / (4.0 * 106836.0)),  (fp_t)( 3483.0 / (4.0 * 106836.0)), (fp_t)( 3362.0 / (4.0 * 106836.0)), (fp_t)( 2908.0 / (4.0 * 106836.0)), (fp_t)(1954.0 / (4.0 * 106836.0)), 
                                   (fp_t)( 956.0 / (4.0 * 106836.0)),  (fp_t)(  320.0 / (4.0 * 106836.0)), (fp_t)(   49.0 / (4.0 * 106836.0)), (fp_t)(   93.0 / (4.0 * 106836.0)), (fp_t)( 633.0 / (4.0 * 106836.0)),  
                                   (fp_t)(1655.0 / (4.0 * 106836.0)),  (fp_t)( 2904.0 / (4.0 * 106836.0)), (fp_t)( 4334.0 / (4.0 * 106836.0)), (fp_t)( 5945.0 / (4.0 * 106836.0)), (fp_t)(7621.0 / (4.0 * 106836.0)), 
                                   (fp_t)(9163.0 / (4.0 * 106836.0)),  (fp_t)(10263.0 / (4.0 * 106836.0)), (fp_t)(10622.0 / (4.0 * 106836.0)), (fp_t)(10026.0 / (4.0 * 106836.0)), (fp_t)(8544.0 / (4.0 * 106836.0)),  
                                   (fp_t)(6424.0 / (4.0 * 106836.0)),  (fp_t)( 4479.0 / (4.0 * 106836.0)), (fp_t)( 2835.0 / (4.0 * 106836.0)), (fp_t)( 1649.0 / (4.0 * 106836.0)), (fp_t)( 874.0 / (4.0 * 106836.0)),  
                                   (fp_t)( 468.0 / (4.0 * 106836.0)),  (fp_t)(  227.0 / (4.0 * 106836.0)), (fp_t)(  114.0 / (4.0 * 106836.0)), (fp_t)(   58.0 / (4.0 * 106836.0)), (fp_t)(  29.0 / (4.0 * 106836.0)),  
                                   (fp_t)(  14.0 / (4.0 * 106836.0)),  (fp_t)(    7.0 / (4.0 * 106836.0)), (fp_t)(    3.0 / (4.0 * 106836.0)), (fp_t)(    2.0 / (4.0 * 106836.0)), (fp_t)(   1.0 / (4.0 * 106836.0)), (fp_t)(0.0 / (4.0 * 106836.0)) };//106836L

    static const fp_t cie_yf[] = { (fp_t)(   0.0 / (4.0 * 106856.0)),  (fp_t)(   1.0 / (4.0 * 106856.0)),  (fp_t)(   4.0 / (4.0 * 106856.0)),  (fp_t)(  12.0 / (4.0 * 106856.0)),  (fp_t)(  40.0 / (4.0 * 106856.0)),
                                   (fp_t)( 116.0 / (4.0 * 106856.0)),  (fp_t)( 230.0 / (4.0 * 106856.0)),  (fp_t)( 380.0 / (4.0 * 106856.0)),  (fp_t)( 600.0 / (4.0 * 106856.0)),  (fp_t)( 910.0 / (4.0 * 106856.0)),
                                   (fp_t)(1390.0 / (4.0 * 106856.0)),  (fp_t)(2080.0 / (4.0 * 106856.0)),  (fp_t)(3230.0 / (4.0 * 106856.0)),  (fp_t)(5030.0 / (4.0 * 106856.0)),  (fp_t)(7100.0 / (4.0 * 106856.0)),
                                   (fp_t)(8620.0 / (4.0 * 106856.0)),  (fp_t)(9540.0 / (4.0 * 106856.0)),  (fp_t)(9950.0 / (4.0 * 106856.0)),  (fp_t)(9950.0 / (4.0 * 106856.0)),  (fp_t)(9520.0 / (4.0 * 106856.0)),
                                   (fp_t)(8700.0 / (4.0 * 106856.0)),  (fp_t)(7570.0 / (4.0 * 106856.0)),  (fp_t)(6310.0 / (4.0 * 106856.0)),  (fp_t)(5030.0 / (4.0 * 106856.0)),  (fp_t)(3810.0 / (4.0 * 106856.0)),  
                                   (fp_t)(2650.0 / (4.0 * 106856.0)),  (fp_t)(1750.0 / (4.0 * 106856.0)),  (fp_t)(1070.0 / (4.0 * 106856.0)),  (fp_t)( 610.0 / (4.0 * 106856.0)),  (fp_t)( 320.0 / (4.0 * 106856.0)),
                                   (fp_t)( 170.0 / (4.0 * 106856.0)),  (fp_t)(  82.0 / (4.0 * 106856.0)),  (fp_t)(  41.0 / (4.0 * 106856.0)),  (fp_t)(  21.0 / (4.0 * 106856.0)),  (fp_t)(  10.0 / (4.0 * 106856.0)),     
                                   (fp_t)(   5.0 / (4.0 * 106856.0)),  (fp_t)(   2.0 / (4.0 * 106856.0)),  (fp_t)(   1.0 / (4.0 * 106856.0)),  (fp_t)(   1.0 / (4.0 * 106856.0)),  (fp_t)(   0.0 / (4.0 * 106856.0)),  (fp_t)(0.0 / (4.0 * 106856.0)) };//106856L

    static const fp_t cie_zf[] = { (fp_t)(   65.0 / (4.0 * 106836.0)), (fp_t)(  201.0 / (4.0 * 106836.0)), (fp_t)(  679.0 / (4.0 * 106836.0)), (fp_t)( 2074.0 / (4.0 * 106836.0)), (fp_t)( 6456.0 / (4.0 * 106836.0)), 
                                   (fp_t)(13856.0 / (4.0 * 106836.0)), (fp_t)(17471.0 / (4.0 * 106836.0)), (fp_t)(17721.0 / (4.0 * 106836.0)), (fp_t)(16692.0 / (4.0 * 106836.0)), (fp_t)(12876.0 / (4.0 * 106836.0)),
                                   (fp_t)( 8130.0 / (4.0 * 106836.0)), (fp_t)( 4652.0 / (4.0 * 106836.0)), (fp_t)( 2720.0 / (4.0 * 106836.0)), (fp_t)( 1582.0 / (4.0 * 106836.0)), (fp_t)(  782.0 / (4.0 * 106836.0)), 
                                   (fp_t)(  422.0 / (4.0 * 106836.0)), (fp_t)(  203.0 / (4.0 * 106836.0)), (fp_t)(   87.0 / (4.0 * 106836.0)), (fp_t)(   39.0 / (4.0 * 106836.0)), (fp_t)(   21.0 / (4.0 * 106836.0)),
                                   (fp_t)(   17.0 / (4.0 * 106836.0)), (fp_t)(   11.0 / (4.0 * 106836.0)), (fp_t)(    8.0 / (4.0 * 106836.0)), (fp_t)(    3.0 / (4.0 * 106836.0)), (fp_t)(    2.0 / (4.0 * 106836.0)),     
                                   (fp_t)(    0.0 / (4.0 * 106836.0)), (fp_t)(    0.0 / (4.0 * 106836.0)), (fp_t)(    0.0 / (4.0 * 106836.0)), (fp_t)(    0.0 / (4.0 * 106836.0)), (fp_t)(    0.0 / (4.0 * 106836.0)),
                                   (fp_t)(    0.0 / (4.0 * 106836.0)), (fp_t)(    0.0 / (4.0 * 106836.0)), (fp_t)(    0.0 / (4.0 * 106836.0)), (fp_t)(    0.0 / (4.0 * 106836.0)), (fp_t)(    0.0 / (4.0 * 106836.0)),     
                                   (fp_t)(    0.0 / (4.0 * 106836.0)), (fp_t)(    0.0 / (4.0 * 106836.0)), (fp_t)(    0.0 / (4.0 * 106836.0)), (fp_t)(    0.0 / (4.0 * 106836.0)), (fp_t)(    0.0 / (4.0 * 106836.0)), (fp_t)(0.0 / (4.0 * 106836.0)) };//106770L

    for (int j = 0; j < (int)downsampled_resolution; j++)
    {
        for (int i = 0; i < (int)downsampled_resolution; i++)
        {
            const fp_t x_o  = (fp_t)i - (downsampled_resolution * (fp_t)0.5);
            const fp_t y_o  = (fp_t)j - (downsampled_resolution * (fp_t)0.5);
            fp_t lambda_i   = (fp_t)380.0;

            for (int f = 0; f < 41; f++)
            {
                /* X, Y co-ordinate of the frequency */
                const fp_t scale    = (fp_t)575.0 / lambda_i;
                const fp_t x_i      = min(max((fp_t)((x_o * scale) + ((fp_t)downsampled_resolution * (fp_t)0.5)), (fp_t)0.0), (fp_t)(downsampled_resolution - 1.0));
                const fp_t y_i      = min(max((fp_t)((y_o * scale) + ((fp_t)downsampled_resolution * (fp_t)0.5)), (fp_t)0.0), (fp_t)(downsampled_resolution - 1.0));
                
                /* Bi-linear interpolation co-ordinates and weights */
                const int x_index   = max((int)0, min((int)x_i,    (int)(downsampled_resolution - 1)));
                const int xx_index  = max((int)0, min(x_index + 1, (int)(downsampled_resolution - 1)));
    
                const int y_index   = max((int)0, min((int)y_i,    (int)(downsampled_resolution - 1)));
                const int yy_index  = max((int)0, min(y_index + 1, (int)(downsampled_resolution - 1)));

                const fp_t x_alpha      = x_i - (fp_t)x_index;
                const fp_t y_alpha      = y_i - (fp_t)y_index;
                const fp_t m_x_alpha    = (fp_t)1.0 - x_alpha;
                const fp_t m_y_alpha    = (fp_t)1.0 - y_alpha;
                const fp_t x_y_alpha    = m_x_alpha * m_y_alpha;
                const fp_t xx_y_alpha   =   x_alpha * m_y_alpha;
                const fp_t x_yy_alpha   = m_x_alpha *   y_alpha;
                const fp_t xx_yy_alpha  =   x_alpha *   y_alpha;

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
                
                lambda_i += (fp_t)((770.0 - 380.0) / 40.0);
            }
        }
    }

    this->convert_xyz_to_rgb(this->temporal_glare_filter, downsampled_resolution, downsampled_resolution);

#if 1
{
    cout << "dumping rgb spectral image" << endl;
    char * tga_data = new char [downsampled_resolution * downsampled_resolution * 3];
    for (int i = 0; i < (int)downsampled_resolution; i++)
    {
        for (int j = 0; j < (int)downsampled_resolution; j++)
        {
            tga_data[((i + (j * downsampled_resolution)) * 3)    ] = (char)min(max(this->temporal_glare_filter[i + (j * downsampled_resolution)].b / (fp_t)25500.0, (fp_t)0.0), (fp_t)255.0);
            tga_data[((i + (j * downsampled_resolution)) * 3) + 1] = (char)min(max(this->temporal_glare_filter[i + (j * downsampled_resolution)].g / (fp_t)25500.0, (fp_t)0.0), (fp_t)255.0);
            tga_data[((i + (j * downsampled_resolution)) * 3) + 2] = (char)min(max(this->temporal_glare_filter[i + (j * downsampled_resolution)].r / (fp_t)25500.0, (fp_t)0.0), (fp_t)255.0);
        }
    }
    
    /* Open output file */
    ostringstream file_name(ostringstream::out);
    file_name <<"rgb_spectral_image" << ".tga";
    ofstream tga_output(file_name.str().c_str(), ios::out);
    
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


camera & camera::tone_map(const tone_mapping_mode_t tone_map, const fp_t key, const fp_t xw, const fp_t yw, const bool gf, const bool ta, const bool ds, const bool sc)
{
    /* RGB to CIE xyz conversion matrix */
    static const fp_t xb[3] = { 0.4124,  0.3576, 0.1805 };
    static const fp_t yb[3] = { 0.2126,  0.7151, 0.0721 };
    static const fp_t zb[3] = { 0.0193,  0.1192, 0.9505 };
    static const fp_t vb[3] = { 0.01477, 0.497,  0.631  };
    
    /* Convert to CIE Yxy */
    fp_t Yi     = (fp_t)0.0;
    fp_t maxY   = (fp_t)0.0;
    fp_t rgbAvg = (fp_t)0.0;
    int numpix  = 0;
    for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
    {
        this->image[i] /= (fp_t)255.0;

        /* Find average rgb value for average tone mapper */
        if (tone_map == global_exposure)
        {
            rgbAvg += (this->image[i].r + this->image[i].g + this->image[i].b) * (fp_t)(1.0 / 3.0);
        }

        /* Convert to CIE xyz */
        ext_colour_t xyz;
        xyz.r = (fp_t)683.0 * ((xb[0] * this->image[i].r) + (xb[1] * this->image[i].g) + (xb[2] * this->image[i].b));
        xyz.g = (fp_t)683.0 * ((yb[0] * this->image[i].r) + (yb[1] * this->image[i].g) + (yb[2] * this->image[i].b));
        xyz.b = (fp_t)683.0 * ((zb[0] * this->image[i].r) + (zb[1] * this->image[i].g) + (zb[2] * this->image[i].b));
        if (xyz.g < (fp_t)3.18e-7)
        {
            xyz.g = (fp_t)3.18e-7;
        }
        else
        {
            numpix++;
            maxY = max(maxY, xyz.g);
            Yi += log10(xyz.g);
        }

        /* Convert to CIE Yyz */
        fp_t W = xyz.g + xyz.r + xyz.b;
        fp_t x = xyz.r/W;
        fp_t y = xyz.g/W;

        /* Desaturate colours */
        if (ds)
        {
            fp_t s;
            const fp_t log10_Y = log10(xyz.g);
            if (log10_Y < (fp_t)-2.0)
            {
                s = (fp_t)0.0;
            }
            else if (log10_Y < (fp_t)0.6)
            {
                const fp_t scaled_log10_y       = (log10_Y + (fp_t)2.0) / (fp_t)2.6;
                const fp_t sq_scaled_log10_y    = scaled_log10_y * scaled_log10_y;
                s = ((fp_t)3.0 * sq_scaled_log10_y) -  ((fp_t)2.0 * scaled_log10_y * sq_scaled_log10_y);
            }
            else
            {
                s = (fp_t)1.0;
            }

            /* Scotopic luminance */
            const fp_t V = (fp_t)1700.0 * ((vb[0] * this->image[i].r) + (vb[1] * this->image[i].g) + (vb[2] * this->image[i].b));
    
            x     = (((fp_t)1.0 - s) * xw) + (s * (x + xw - (fp_t)(1.0/3.0)));
            y     = (((fp_t)1.0 - s) * yw) + (s * (y + yw - (fp_t)(1.0/3.0)));
            xyz.g = ((fp_t)0.4468 * ((fp_t)1.0 - s) * V) + (s * xyz.g);
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
    Yi = pow((fp_t)10.0, Yi);


    /* Correct Yi to the adaption level */
    if (ta && (this->time_step > (fp_t)0.0))
    {
        fp_t k;
        const fp_t log_la = log(Yi);
        if (log_la < (fp_t)-2.0)
        {
            k = (fp_t)1.0;
        }
        else if (log_la < (fp_t)1.0)
        {
            k = (fp_t)1.0 - ((log_la + (fp_t)2.0) * (fp_t)(1.0 / 3.0));
        }
        else
        {
            k = (fp_t)0.0;
        }
        
        /* Dark adaption (light -> dark) */
        if (this->adatption_level > Yi)
        {
            /* Unbleached pigment proportion, luminances in Trolands (luminance * pupil area in cd/m^2 * mm^2) */
//            const yi_troland    = Yi * (fp_t)75.0;
//            const fp_t p_cone   = (fp_t)45000.0 / ((fp_t)45000.0 * yi_troland);
//            const fp_t p_rod    = (fp_t)40000.0 / ((fp_t)40000.0 * yi_troland);
//            
//            const fp_t t_cone   = (fp_t)0.02 * (fp_t)8.0;
//            const fp_t t_rod    = (fp_t)0.1  * (fp_t)0.015;
//            
//            const fp_t pt_cone  = t_cone * exp((fp_t)3.0  * ((fp_t)1.0 - p_cone));
//            const fp_t pt_rod   = t_rod  * exp((fp_t)19.0 * ((fp_t)1.0 - p_rod ));
            
            /* Cone adaption level */
            const fp_t a_cone = Yi + ((this->adatption_level - Yi) * exp(-(this->time_step / (fp_t)0.2))); // 0.200s for no bleaching else 120s

            /* Rod adaption level */
            const fp_t a_rod  = Yi + ((this->adatption_level - Yi) * exp(-(this->time_step / (fp_t)6.0))); // 6s     for no bleaching else 400s
            
            this->adatption_level = (((fp_t)1.0 - k) * a_cone) + (k * a_rod);
        }
        /* Light adaption (dark to light) */
        else
        {
            const fp_t r    = (Yi / this->adatption_level);
            const fp_t c1   = this->adatption_level * (pow(r, (fp_t)0.9) - (fp_t)1.0);
            const fp_t c2   = Yi * ((fp_t)1.0 - pow(r, (fp_t)-0.1));

            /* Cone adaption level */
            const fp_t a_cone = Yi - (c1 * exp(-(this->time_step / (fp_t)16.0e-3))) - (c2 * exp(-(this->time_step / (fp_t)200.0e-3)));

            /* Rod adaption level */
            const fp_t a_rod  = Yi - (c1 * exp(-(this->time_step / (fp_t)40.0e-3))) - (c2 * exp(-(this->time_step / (fp_t)6.0)));

            this->adatption_level = (((fp_t)1.0 - k) * a_cone) + (k * a_rod);
        }
        
        cout << Yi << " -> " << this->adatption_level << endl;
        Yi = this->adatption_level;
    }
    else
    {
        this->adatption_level = Yi;
    }


    /* Contrast based tone mapping (Ward) */
    if (tone_map == global_contrast)
    {
        const fp_t Yw  = (fp_t)300.0;  /* Maximum display illumination */
        const fp_t num = (fp_t)1.219 + pow(Yw * (fp_t)0.5, (fp_t)0.4);
        const fp_t den = (fp_t)1.219 + pow(Yi, (fp_t)0.4);
        const fp_t sf  = ((fp_t)1.0 / Yw) * pow((num / den), (fp_t)2.5);

        for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
        {
            this->image[i].g *= sf;
        }
    }
    /* Hostogram based local tone mapping function */
    else if (tone_map == local_histogram)
    {
        this->histogram_tone_map(false);
    }
    /* Hostogram based local tone mapping function with human based contrast limits */
    else if (tone_map == local_human_histogram)
    {
        this->histogram_tone_map(true);
    }
    /* Non-linear tone mapping */
    else if (tone_map == global_non_linear)
    {
        const fp_t sf = key / Yi;
        maxY *= sf;
        maxY  = (fp_t)1.0 / (maxY * maxY);

        for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
        {
            this->image[i].g *= sf;
            if (maxY)
            {
                this->image[i].g *= ((fp_t)1.0 + this->image[i].g * maxY) / ((fp_t)1.0 + this->image[i].g);
            }
        }
    }
    /* Exposure based tone mapping */
    else if (tone_map == global_exposure)
    {
        const fp_t sf = key / Yi;
        maxY *= sf;

        for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
        {
            this->image[i].g *= sf;
            if (maxY)
            {
                this->image[i].g = (fp_t)1.0 - exp(-this->image[i].g);
            }
        }
    }
    /* Average luminance based tone mapping */
    else if (tone_map == global_avg_luminance)
    {
        /* Map average luminance to 0.5 */
        fp_t sf = (fp_t)1.0;
        if (rgbAvg)
        {
            sf *= key * ((fp_t)0.5 / (rgbAvg * 683.0));
        }

        for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
        {
            this->image[i].g *= sf;
        }
    }
    /* Maximum luminance based tone mapping */
    else if (tone_map == global_max_luminance)
    {
        const fp_t sf = key / maxY;
        for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
        {
            this->image[i].g *= sf;
        }
    }
    else if (tone_map == global_bilateral_filter)
    {
        const fp_t s_s = (fp_t)0.02 * min(this->x_res, this->y_res);
        this->bilateral_filter_tone_map((fp_t)0.4, s_s, (fp_t)0.4, s_s);
    }
    else if (tone_map == global_ferwerda)
    {
        const fp_t lda      = (fp_t)800.0;
        const fp_t log_lda  = log(lda * (fp_t)0.5);
        const fp_t log_la   = log(Yi);

        /* Cone response */
        fp_t tp;
        if (log_la <= (fp_t)-2.6)
        {
            tp = (fp_t)-0.72;
        }
        else if (log_la <= (fp_t)1.9)
        {
            tp = pow((((fp_t)0.249 * log_la) + (fp_t)0.65), (fp_t)2.7) - (fp_t)0.72;
        }
        else
        {
            tp = log_la - (fp_t)1.255;
        }
        const fp_t mp   = (log_lda - (fp_t)1.255) / tp;

        /* Rod response */
        fp_t ts;
        if (log_la <= (fp_t)-3.94)
        {
            ts = (fp_t)-2.86;
        }
        else if (log_la < (fp_t)-1.44)
        {
            ts = pow((((fp_t)0.405 * log_la) + (fp_t)1.6), (fp_t)2.18) - (fp_t)2.86;
        }
        else
        {
            ts = log_la - (fp_t)0.395;
        }
        const fp_t ms = (log_lda - (fp_t)1.255) / ts;
        
        fp_t k;
        if (log_la < (fp_t)-2.0)
        {
            k = (fp_t)1.0;
        }
        else if (log_la < (fp_t)1.0)
        {
            k = (fp_t)1.0 - ((log_la + (fp_t)2.0) * (fp_t)(1.0 / 3.0));
        }
        else
        {
            k = (fp_t)0.0;
        }

        for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
        {
            const fp_t ldp  = mp * this->image[i].g;
            const fp_t lds  = ms * this->image[i].g;
            this->image[i].g = ((fp_t)1.0 / lda) * (ldp + (k * lds));
        }
    }
    /* No tone mapping */
    else
    {
        /* Remove the k conversion factor */
        const fp_t sf = (fp_t)1.0 / 683.0;
        for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
        {
            this->image[i].g *= sf;
        }
    }

    /* Stretch low contrast or over compressed images */
    if (sc)
    {
        /* Find max and min luminance value */
        fp_t maxY2 = -MAX_DIST;
        fp_t minY2 =  MAX_DIST;
        for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
        {
            maxY2 = max(maxY2, this->image[i].g);
            minY2 = min(minY2, this->image[i].g);
        }
      
        if (((minY2 > (fp_t)0.0) || (maxY2 < (fp_t)0.0)) && (minY2 < maxY2))
        {
            const fp_t offset = min(-minY2, (fp_t)0.0);
            const fp_t scale  = (fp_t)1.0 / (min((fp_t)1.0, maxY2) - max((fp_t)0.0, minY2));
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
        this->perform_glare_filter(Yi, 1.0);
    }

    return *this;
}


// cppcheck-suppress unusedFunction
camera & camera::gamma_correct(const fp_t gamma)
{
    for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
    {
        fp_t r = this->image[i].r / (fp_t)255.0;
        fp_t g = this->image[i].g / (fp_t)255.0;
        fp_t b = this->image[i].b / (fp_t)255.0;

        /* Saturate */
        r = min(max(r, (fp_t)0.0), (fp_t)1.0);
        g = min(max(g, (fp_t)0.0), (fp_t)1.0);
        b = min(max(b, (fp_t)0.0), (fp_t)1.0);

        /* Gamma correct */
        if (fabs(gamma - (fp_t)2.2) > 0.001)
        {
            /* regular gamma correction */
            r = pow(r, (fp_t)1.0/gamma);
            g = pow(g, (fp_t)1.0/gamma);
            b = pow(b, (fp_t)1.0/gamma);
        }
        else if (fabs(gamma - (fp_t)1.0) > 0.001)
        {
            /* sRGB standard gamma correction (similar to regular w/2.2) */
            r = (r <= (fp_t)0.0031308) ? ((fp_t)12.92 * r) : ((fp_t)1.055 * pow(r, (fp_t)(1.0 / 2.4)) - (fp_t)0.055);
            g = (g <= (fp_t)0.0031308) ? ((fp_t)12.92 * g) : ((fp_t)1.055 * pow(g, (fp_t)(1.0 / 2.4)) - (fp_t)0.055);
            b = (b <= (fp_t)0.0031308) ? ((fp_t)12.92 * b) : ((fp_t)1.055 * pow(b, (fp_t)(1.0 / 2.4)) - (fp_t)0.055);
        }

        this->image[i].r = r * (fp_t)255.0;
        this->image[i].g = g * (fp_t)255.0;
        this->image[i].b = b * (fp_t)255.0;
    }

    return *this;
}


void camera::perform_glare_filter(const fp_t Yi, const fp_t Yw)
{
    /* Pick lighting condition */
    const fp_t log10_yi = log10(Yi);
    light_level_t current_light_level;
    ext_colour_t *glare_filter;
    ext_colour_t **glare_filter_ptr;
    if (log10_yi < (fp_t)-2.0)
    {
        cout << "lighting is scotopic" << endl;
        current_light_level = scotopic;
        glare_filter_ptr    = &this->scotopic_glare_filter;
    }
    else if (log10_yi < (fp_t)0.6)
    {
        cout << "lighting is mesopic" << endl;
        current_light_level = mesopic;
        glare_filter_ptr    = &this->mesopic_glare_filter;
    }
    else
    {
        cout << "lighting is photopic" << endl;
        current_light_level = photopic;
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
        if ((this->image[i].r > (fp_t)255.0) || (this->image[i].g > (fp_t)255.0) || (this->image[i].b > (fp_t)255.0))
        {
            image_in[i                 ] = this->image[i].r;
            image_in[i + image_g_offset] = this->image[i].g;
            image_in[i + image_b_offset] = this->image[i].b;
        }
        else
        {
            image_in[i                 ] = (fp_t)0.0;
            image_in[i + image_g_offset] = (fp_t)0.0;
            image_in[i + image_b_offset] = (fp_t)0.0;
        }
    }

    /*  Split glare image into channels and offset centre to pixel 0 0 */
    const int gf_offset      = (int)((this->x_res >> 1) + (this->x_res * ((this->y_res - 1) >> 1)));
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
        const fp_t tmp = image_out[i][0];
        image_out[i][0] = (image_out[i][0] * glare_out[i][0]) - (image_out[i][1] * glare_out[i][1]);
        image_out[i][1] = (tmp             * glare_out[i][1]) + (image_out[i][1] * glare_out[i][0]);
    }
    fftwf_execute(inv_p);


    /* Merge the convolution and original image */
    const fp_t pixel_divisor = (fp_t)1.0 / (fp_t)(picture_size * 255);
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


void camera::bilateral_filter_tone_map(const fp_t r_s, const fp_t s_s, const fp_t r_sa, const fp_t s_sa)
{
    /* Build log image */
    fp_t *log_intensity = new fp_t [ this->x_res * this->y_res ];
    fp_t input_max = -MAX_DIST;
    fp_t input_min =  MAX_DIST;
    for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
    {
        if (this->image[i].g > (fp_t)3.18e-7)
        {
            log_intensity[i] = log10(this->image[i].g);
            input_max = max(input_max, log_intensity[i]);
            input_min = min(input_min, log_intensity[i]);
        }
    }


    /* Down sample */    
    const fp_t input_delta  = input_max - input_min;
    const fp_t sigma_r      = r_s / r_sa; 
    const fp_t sigma_s      = s_s / s_sa;

    const int padding_xy    = (int)((fp_t)2.0 * sigma_s) + 1;
    const int padding_z     = (int)((fp_t)2.0 * sigma_r) + 1;

    const int small_width   = (int)((this->x_res - 1) / s_sa) + 1 + (padding_xy << 1);
    const int small_height  = (int)((this->y_res - 1) / s_sa) + 1 + (padding_xy << 1);
    const int small_depth   = (int)(input_delta / r_sa) + 1 + (padding_z << 1);

    fp_t *wiw = (float *)fftwf_malloc(2 * small_width * small_height * small_depth * sizeof(float));
    memset(wiw,  0, (2 * small_width * small_height * small_depth * sizeof(fp_t)));

    int iw_offset = (small_width * small_height * small_depth);
    for(int x = 0; x < (int)this->x_res; x++)
    {
        for(int y = 0; y < (int)this->y_res; y++)
        {
            const int small_x = (int)((fp_t)x / s_sa + (fp_t)0.5) + padding_xy;
            const int small_y = (int)((fp_t)y / s_sa + (fp_t)0.5) + padding_xy;
            const int small_z = (int)((log_intensity[x + (this->x_res * y)] - input_min) / r_sa + (fp_t)0.5) + padding_z;

            wiw[small_x + (small_width * (small_y + (small_height * small_z)))            ] += (fp_t)1.0;
            wiw[small_x + (small_width * (small_y + (small_height * small_z))) + iw_offset] += log_intensity[x + (this->x_res * y)];
        }
    }
    
    
    /* Build kernel */
    fp_t *kernel = (float *)fftwf_malloc(small_width * small_height * small_depth * sizeof(float));

    const int half_width  = small_width  >> 1;
    const int half_height = small_height >> 1;
    const int half_depth  = small_depth  >> 1;
    for(int x = 0; x < small_width; x++)
    {
        const fp_t X = x - ((x > half_width) ? small_width : (fp_t)0.0);

        for(int y = 0; y < small_height; y++)
        {
            const fp_t Y = y - ((y > half_height) ? small_height : (fp_t)0.0);

            for(int z = 0; z < small_depth; z++)
            {
                const fp_t Z = z - ((z > half_depth) ? small_depth : (fp_t)0.0);
                const fp_t rr = (X * X + Y * Y) / (sigma_s * sigma_s) + Z * Z / (sigma_r * sigma_r);	
                kernel[x + (small_width * (y + (small_height * z)))] = exp(-rr * (fp_t)0.5);
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
    const fp_t s = (fp_t)1.0 / (small_width * small_height * small_depth);
    for(int i = 0; i < (small_width * small_height * small_depth); i++)
    {
        fp_t tmp = wiw_freq[i][0];
        wiw_freq[i][0] = ((k_freq[i][0] * tmp           ) - (k_freq[i][1] * wiw_freq[i][1])) * s;
        wiw_freq[i][1] = ((k_freq[i][0] * wiw_freq[i][1]) + (k_freq[i][1] * tmp           )) * s;

        tmp = wiw_freq[i + iw_offset][0];
        wiw_freq[i + iw_offset][0] = ((k_freq[i][0] * tmp                       ) - (k_freq[i][1] * wiw_freq[i + iw_offset][1])) * s;
        wiw_freq[i + iw_offset][1] = ((k_freq[i][0] * wiw_freq[i + iw_offset][1]) + (k_freq[i][1] * tmp                       )) * s;
    }
    fftwf_execute(inv_p);


    /* Apply non-linearities */
    fp_t *result = new fp_t [this->x_res * this->y_res];
    fp_t max_value = -MAX_DIST;
    fp_t min_value =  MAX_DIST;
    for(int x = 0; x < (int)this->x_res; x++)
    {
        for(int y = 0; y < (int)this->y_res; y++)
        {
            const fp_t z = log_intensity[x + (this->x_res * y)] - input_min;

            const fp_t x_addr   = (fp_t)(x) / s_sa + padding_xy;
            const int x_index   = max(0, min((int)x_addr,   small_width-1));
            const int xx_index  = max(0, min(x_index+1,     small_width-1));
    
            const fp_t y_addr   = (fp_t)(y) / s_sa + padding_xy;
            const int y_index   = max(0, min((int)y_addr,   small_height-1));
            const int yy_index  = max(0, min(y_index+1,     small_height-1));

            const fp_t z_addr   = (fp_t)(z) / r_sa + padding_z;
            const int z_index   = max(0, min((int)z_addr,   small_depth-1));
            const int zz_index  = max(0, min(z_index+1,     small_depth-1));

            const fp_t x_alpha  = x_addr - x_index;
            const fp_t y_alpha  = y_addr - y_index;
            const fp_t z_alpha  = z_addr - z_index;

            const fp_t IW = 
                ((fp_t)1.0 - x_alpha) * ((fp_t)1.0 - y_alpha) * ((fp_t)1.0 - z_alpha) * wiw[x_index  + (small_width * (y_index  + (small_height * z_index ))) + iw_offset] +
                x_alpha               * ((fp_t)1.0 - y_alpha) * ((fp_t)1.0 - z_alpha) * wiw[xx_index + (small_width * (y_index  + (small_height * z_index ))) + iw_offset] +
                ((fp_t)1.0 - x_alpha) * y_alpha               * ((fp_t)1.0 - z_alpha) * wiw[x_index  + (small_width * (yy_index + (small_height * z_index ))) + iw_offset] +
                x_alpha               * y_alpha               * ((fp_t)1.0 - z_alpha) * wiw[xx_index + (small_width * (yy_index + (small_height * z_index ))) + iw_offset] +
                ((fp_t)1.0 - x_alpha) * ((fp_t)1.0 - y_alpha) * z_alpha               * wiw[x_index  + (small_width * (y_index  + (small_height * zz_index))) + iw_offset] +
                x_alpha               * ((fp_t)1.0 - y_alpha) * z_alpha               * wiw[xx_index + (small_width * (y_index  + (small_height * zz_index))) + iw_offset] +
                ((fp_t)1.0 - x_alpha) * y_alpha               * z_alpha               * wiw[x_index  + (small_width * (yy_index + (small_height * zz_index))) + iw_offset] +
                x_alpha               * y_alpha               * z_alpha               * wiw[xx_index + (small_width * (yy_index + (small_height * zz_index))) + iw_offset];


            const fp_t W = 
                ((fp_t)1.0 - x_alpha) * ((fp_t)1.0 - y_alpha) * ((fp_t)1.0 - z_alpha) * wiw[x_index  + (small_width * (y_index  + (small_height * z_index )))] +
                x_alpha               * ((fp_t)1.0 - y_alpha) * ((fp_t)1.0 - z_alpha) * wiw[xx_index + (small_width * (y_index  + (small_height * z_index )))] +
                ((fp_t)1.0 - x_alpha) * y_alpha               * ((fp_t)1.0 - z_alpha) * wiw[x_index  + (small_width * (yy_index + (small_height * z_index )))] +
                x_alpha               * y_alpha               * ((fp_t)1.0 - z_alpha) * wiw[xx_index + (small_width * (yy_index + (small_height * z_index )))] +
                ((fp_t)1.0 - x_alpha) * ((fp_t)1.0 - y_alpha) * z_alpha               * wiw[x_index  + (small_width * (y_index  + (small_height * zz_index)))] +
                x_alpha               * ((fp_t)1.0 - y_alpha) * z_alpha               * wiw[xx_index + (small_width * (y_index  + (small_height * zz_index)))] +
                ((fp_t)1.0 - x_alpha) * y_alpha               * z_alpha               * wiw[x_index  + (small_width * (yy_index + (small_height * zz_index)))] +
                x_alpha               * y_alpha               * z_alpha               * wiw[xx_index + (small_width * (yy_index + (small_height * zz_index)))];

            result[x + (this->x_res * y)] = IW / W;
            max_value = max(max_value, result[x + (this->x_res * y)]);
            min_value = min(min_value, result[x + (this->x_res * y)]);
        }
    }


    /* Scale the image */
    const fp_t contrast = (fp_t)300.0;
    const fp_t gamma = log10(contrast) /  (max_value - min_value);
    for(int i = 0; i < (int)(this->x_res * this->y_res); i++)
    {
        const fp_t scaled_value = pow((fp_t)10.0, (result[i] * gamma + (log_intensity[i] - result[i])));
        this->image[i].g = scaled_value * (fp_t)(1.0/255.0);
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


fp_t camera::just_noticable_difference(const fp_t La) const
{
    const fp_t lLa = log10(La);
    fp_t lLt;
    if (lLa < (fp_t)-3.94)
    {
        lLt = (fp_t)-2.86;
    }
    else if (lLa < (fp_t)-1.44)
    {
        lLt = pow((fp_t)0.405 * lLa + (fp_t)1.6, (fp_t)2.18) - (fp_t)2.86;
    }
    else if (lLa < (fp_t)-0.0184)
    {
        lLt = lLa - (fp_t)0.395;
    }
    else if (lLa < (fp_t)1.9)
    {
        lLt = pow((fp_t)0.249 * lLa + (fp_t)0.65, (fp_t)2.7) - (fp_t)0.72;
    }
    else
    {
        lLt = lLa - (fp_t)1.255;
    }
    
    return pow((fp_t)10.0, lLt);
}


void camera::histogram_tone_map(const bool h)
{
    /* Scale the image to one pixel per view degree */
    const fp_t x_angle = (-2.0 * this->x_m) / this->t;
    const fp_t y_angle = (-2.0 * this->y_m) / this->t;
    const int scaled_x_res = (int)(x_angle * (fp_t)(360.0 / (2.0 * PI)));
    const int scaled_y_res = (int)(y_angle * (fp_t)(360.0 / (2.0 * PI)));

    fp_t * scaled_luminance = new fp_t [scaled_x_res * scaled_y_res];
    memset(scaled_luminance, 0, (scaled_x_res * scaled_y_res * sizeof(fp_t)));
    
    const fp_t scaled_x_inc = (fp_t)this->x_res / (fp_t)scaled_x_res;
    const fp_t scaled_y_inc = (fp_t)this->y_res / (fp_t)scaled_y_res;
    fp_t y0 = (fp_t)0.0;
    fp_t y1 = scaled_y_inc;
    for (int j = 0; j < scaled_y_res; j++)
    {
        fp_t x0 = (fp_t)0.0;
        fp_t x1 = scaled_x_inc;
        
        for (int i = 0; i < scaled_x_res; i++)
        {
            /* Average the 1 degree block to one pixel */
            for (int y = (int)y0; y <= (int)y1; y++)
            {
                for (int x = (int)x0; x <= (int)x1; x++)
                {
                    scaled_luminance[(j * scaled_x_res) + i] += this->image[(y * this->x_res) + x].g;
                }
                
            }
            
            scaled_luminance[(j * scaled_x_res) + i] /= ((x1 - x0) + 1) * ((y1 - y0) + 1);
            
            x0 = x1;
            x1 = min((fp_t)(this->x_res - (fp_t)1.0), (x1 + scaled_x_inc));
        }

        y0 = y1;
        y1 = min((fp_t)(this->y_res - (fp_t)1.0), (y1 + scaled_y_inc));
    }


    /* Find the range for the data, above 0 */
    const int bins = 100;
    int pix_gt0 = 0;
    fp_t lw_min = MAX_DIST;
    fp_t lw_max = 0.0;
    for (int i = 0; i< (scaled_x_res * scaled_y_res); i++)
    {
        if (scaled_luminance[i] > 0.0)
        {
            lw_min = min(lw_min, scaled_luminance[i]);
            lw_max = max(lw_max, scaled_luminance[i]);

            pix_gt0++;
        }
    }
    
    /* Black image, no further work required */
    if (lw_min <= (fp_t)0.0)
    {
        cout << "black image" << endl;
        delete [] scaled_luminance;
        return;
    }

    /* Convert to log scale */
    const fp_t min_lum  = (fp_t)1.0e-4;
    lw_min              = max(lw_min, min_lum);
    fp_t bw_min         = log(lw_min);
    fp_t bw_max         = log(lw_max);
    if (bw_max == bw_min)
    {
        bw_max += 0.0001 * fabs(bw_max);
    }

    /* Check for image within displayable range */
    const fp_t ld_max = (fp_t)300.0;
    const fp_t ld_min = (fp_t)1.0;
    const fp_t bd_max = log(ld_max);
    const fp_t bd_min = log(ld_min);
    if ((bd_max - bd_min) > (bw_max - bw_min))
    {
        cout << "using linear scale factor" << endl;
        const fp_t sf = (bd_max - bd_min) / (bw_max - bw_min);
        for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
        {
            this->image[i].g = this->image[i].g * sf + bd_min;
        }

        delete [] scaled_luminance;
        return;
    }

    /* Populate the histogram */
    int  his[(bins + 1)];
    fp_t cdf[(bins + 1)];
    memset(his, 0, ((bins + 1) * sizeof(int )));
    memset(cdf, 0, ((bins + 1) * sizeof(fp_t)));
    
    int scaled_pixel_gt0 = 0;
    for (int i = 0; i < (scaled_x_res * scaled_y_res); i++)
    {
        if (scaled_luminance[i] > 0.0)
        {
            int b = (int)((log(scaled_luminance[i]) - bw_min) / (bw_max - bw_min) * bins) + 1;
            b = min(max(b, 0), bins);
            his[b]++;

            scaled_pixel_gt0++;
        }
    }

    /* Take average luminance */
    const fp_t lw_avg = this->adatption_level;//max(lw_avg, min_lum);

    /* Populate the cumalitive density function */
    fp_t sum = (fp_t)0.0;
    for (int i = 1; i <= bins; i++)
    {
        sum += his[i];
        cdf[i] = sum/(fp_t)pix_gt0;
    }

    
    /* Adjust the histogram */
    const fp_t tolerance = (fp_t)0.025 * (fp_t)pix_gt0;
    const fp_t bin_step_size = (bw_max - bw_min) / bins;
    const fp_t lw_step = (fp_t)1.0 / (fp_t)bins * (bw_max - bw_min);
    int trimmings;
    do
    {
        trimmings = 0;

        /* Convergence will fail, but there is no need to compression anyway */
        /* Apply a linear scale */
        if (pix_gt0 < tolerance)
        {
            cout << "using contrast based scale factor" << endl;
            const fp_t num = (fp_t)1.219 + pow(ld_max * (fp_t)0.5, (fp_t)0.4);
            const fp_t den = (fp_t)1.219 + pow(lw_avg , (fp_t)0.4);

            const fp_t sf  = ((fp_t)1.0 / ld_max) * pow((num / den), (fp_t)2.5);

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
            fp_t ceiling = (pix_gt0 * bin_step_size) / (bd_max - bd_min);

            /* Adjust based on human perception */
            if (h)
            {
                /* Displaybale brightness */
                const fp_t bde = bd_min + (bd_max - bd_min) * cdf[i];
                const fp_t ld = exp(bde);

                const fp_t bw = i * lw_step + bw_min;
                const fp_t lw = exp(bw);
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

        fp_t sum = (fp_t)0.0;
        for (int i = 1; i <= bins; i++)
        {
            sum += his[i];
            cdf[i] = sum/(fp_t)pix_gt0;
        }

    } while (trimmings > tolerance);


    /* Tone map the original image */
    for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
    {
        if (this->image[i].g > 0.0)
        {
            const fp_t bw = log(this->image[i].g);

            fp_t bf = (bw - bw_min) / (bw_max - bw_min) * bins;
            bf = min(max(bf, (fp_t)0.0), (fp_t)bins);
            const int b0 = (int)bf;
            const int b1 = min((b0 + 1), bins);
            const fp_t s = bf - b0;
            const fp_t pbw = (1 - s) * cdf[b0] + s * cdf[b1];
    
            const fp_t bde = bd_min + (bd_max - bd_min) * pbw;
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
        p[i].r = ( 826.353  * xyz.r) + (-391.986 * xyz.g) + (-127.143  * xyz.b);
        p[i].g = (-247.0695 * xyz.r) + ( 478.329 * xyz.g) + (  10.5825 * xyz.b);
        p[i].b = ( 14.2035  * xyz.r) + ( -52.02  * xyz.g) + ( 269.535  * xyz.b);
    }

    return;
}


void camera::convert_Yxy_to_rgb()
{
    for (int i = 0; i < (int)(this->x_res * this->y_res); i++)
    {
        ext_colour_t yxy = this->image[i];

        /* Convert to CIE xyz */
        const fp_t x = yxy.r;
        const fp_t y = yxy.b;
        yxy.r = x * yxy.g/y;
        yxy.b = (1.0-x-y)*yxy.g/y;

        /* Conversion matrix assumes Y is in the range 0-1.0 and scales up by 255 */
        this->image[i].r = ( 826.353  * yxy.r) + (-391.986 * yxy.g) + (-127.143  * yxy.b);
        this->image[i].g = (-247.0695 * yxy.r) + ( 478.329 * yxy.g) + (  10.5825 * yxy.b);
        this->image[i].b = ( 14.2035  * yxy.r) + ( -52.02  * yxy.g) + ( 269.535  * yxy.b);
    }

    return;
}


#ifdef LOG_DEPTH
camera & camera::write_depth_map(const string &file_name)
{
    /* Open output file */
    ofstream depth_output(file_name.c_str(), ios::out);
    
    /* Write pixel depths */
    for (unsigned int i = 0; i < (this->out_x_res * this->out_y_res); i++)
    {
        depth_output << this->depth_map[i] << ", ";
    }

    /* Clean up */
    depth_output.close();
    
    return *this;
}
#endif


camera & camera::write_tga_file(const string &file_name, unsigned char *o)
{
    /* Open output file */
    ofstream tga_output(file_name.c_str(), ios::out);

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

    cout << "Writing snapshot taken to " << file_name << endl;

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


void write_png_file(const string &file_name, unsigned char *png_data, const unsigned int x, const unsigned int y)
{
    /* Open output file */
    ofstream png_output(file_name.c_str(), ios::out);
    cout << "Writing snapshot taken to " << file_name << endl;

    /* Initialise png objects */
    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (png_ptr == nullptr)
    {
        assert(!"Error creating png write struct");
    }

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
    	cout << "Error opening png file " << file_name << " for output" << endl;
    	assert(!"Cannot open file");
    }

    /* Set compression parameters */
    png_init_io(png_ptr, fp);
    png_set_IHDR(png_ptr, info_ptr, x, y, 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

    /* Compress */
    png_write_info(png_ptr, info_ptr);

    png_bytep row_pointers[y];
    const int row_stride = x * 3;
    int read_location = x * (y - 1) * 3;
    for (int k = 0; k < (int)y; k++)
    {
        row_pointers[k] = &png_data[read_location];
        read_location -= row_stride;
    }
    png_write_image(png_ptr, row_pointers);

    /* Clean up */
    png_write_end(png_ptr, info_ptr);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    fclose(fp);
   
    png_output.close();
    
    return;
}


camera & camera::write_png_file(const string &file_name)
{
    /* Convert data to rgb */
    unsigned char * png_data = new unsigned char [this->out_x_res * this->out_y_res * 3];
    this->clip_image_to_rgb(png_data);
        
    raptor_raytracer::write_png_file(file_name, &png_data[0], this->out_x_res, this->out_y_res);

    delete [] png_data;
    return *this;
}

        
camera & camera::write_jpeg_file(const string &file_name, const int q, unsigned char *o)
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

    cout << "Writing snapshot taken to " << file_name << endl;

    /* Initialise jpeg objects */
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    /* Open output file */
    FILE * outfile;
    if ((outfile = fopen(file_name.c_str(), "wb")) == nullptr)
    {
    	cout << "Error opening jpeg file " << file_name << " for output" << endl;
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
