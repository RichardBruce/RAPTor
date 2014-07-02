#include "common.h"
#include "parser_common.h"

#include "camera.h"

#include "lwo_parser.h"


enum mapper_type_t { non = 0, f_noise = 1, planar = 2, cubic = 3, spherical = 4, cylindrical = 5 };
enum mapper_of_t   { map_btex = 0, map_ctex = 1, map_dtex = 2, map_ltex = 3, map_stex = 4, map_rtex = 5, map_ttex = 6 };

struct texture_info_t
{
    void reset(const mapper_of_t m, const mapper_type_t s)
    {
        trgb            = ext_colour_t(255.0,255.0,255.0);
        topc            = 1.0;
        shader          = s;
        map_of          = m;
        twrp_mode_x     = (texture_wrapping_mode_t)2;
        twrp_mode_y     = (texture_wrapping_mode_t)2;    
    }
    
    void add_shader_to(list<texture_mapper *>  *const t)
    {
        texture_mapper  *tm;
        fp_t *img;
        unsigned int img_width;
        unsigned int img_height;
        unsigned int cpp;
        switch (this->shader)
        {
            case f_noise :
                tm = new perlin_noise_3d_mapper(this->trgb, this->tfp[1], this->tfp[0], this->tip, 4);
                t->push_front(tm);
                break;
                
            case cylindrical  :
                tm = new cylindrical_mapper(this->filename.c_str(), this->tctr, this->tnorm, this->tsiz, this->tfp[0]);
                t->push_front(tm);
                break;
    
            case planar  :
                cpp = read_jpeg(&img, this->filename.c_str(), &img_height, &img_width);
                tm  = new planar_mapper(img, this->tctr, this->tnorm, this->tsiz, cpp, img_width, img_height, this->twrp_mode_x, this->twrp_mode_y);
                t->push_front(tm);
                break;
    
            default :
                assert(false);
                break;
        }
    }
    
    void add_shader()
    {
        switch (this->map_of)
        {
            case map_btex :
                this->add_shader_to(&this->btex);
                break;

            case map_ctex :
                this->add_shader_to(&this->ctex);
                break;
                
            case map_dtex :
                this->add_shader_to(&this->dtex);
                break;
                
            case map_rtex :
                this->add_shader_to(&this->rtex);
                break;

            case map_ttex :
                this->add_shader_to(&this->ttex);
                break;
                
            default :
                assert(false);
                break;
        }
    }

    list<texture_mapper *>  btex;
    list<texture_mapper *>  ctex;
    list<texture_mapper *>  dtex;
    list<texture_mapper *>  ttex;
    list<texture_mapper *>  rtex;
    string                  filename;
    ext_colour_t            trgb;
    point_t                 tctr;
    point_t                 tnorm;
    point_t                 tsiz;
    fp_t                    tfp[4];
    fp_t                    topc;
    mapper_type_t           shader;
    mapper_of_t             map_of;
    unsigned short          tip;
    texture_wrapping_mode_t twrp_mode_x;
    texture_wrapping_mode_t twrp_mode_y;
};


/***********************************************************
  check_for_chunk checks that the pointer a points at the 
  same chunk name as the pointer e with length l.
  
  If the comparison is correct a is updated past the chunk
  name otherwise the program is aborted through assert()
************************************************************/
inline void check_for_chunk(const char **const a, const char *const e, const int l)
{
    if (strncmp(*a, e, l) != 0)
    {
        cout << (*a) << " found where " << e << " chunk not found where expected" << endl;
        assert(false);
    }
    (*a) += l;
}


/***********************************************************
  find_surf_chunk will move s to point to the first byte of 
  the SURF chunk (the MSB of the size of the SURF chunk).
  The number of surfaces will also be returned.
  
  l is the number of bytes in the SRFS chunk. s is a pointer
  to move the the start of the SURF chunk and p is a pointer
  to the input file.
************************************************************/
inline unsigned find_surf_chunk(const char *p, const char **s, unsigned l)
{
    /* Parse through the SRFS chunk counting the number of SRFS */
    unsigned num_of_surfs = 0;
    for (unsigned i = 0; i < l; i++)
    {
        if ((*p == 0x00) && ((i & 0x1) == 1))
        {
            num_of_surfs++;
        }
        p++;
    }
    
    /* Check p points where it should */
    check_for_chunk(&p, "POLS", 4);
    
    /* Skip the POLS chunk */
    unsigned tmp = from_byte_stream<unsigned>(&p);
    cout << tmp << endl;
    (*s) = p + tmp;
    
    /* Check the SURF chunk has been found */
    check_for_chunk(s, "SURF", 4);
    
    return num_of_surfs;
}


inline mapper_type_t pick_shader(const char *const c)
{
    if (strcmp(c, "Fractal Noise") == 0)
    {
        return f_noise;
    }
    else if (strcmp(c, "Planar Image Map") == 0)
    {
        return planar;
    }
    else if (strcmp(c, "Cubic Image Map") == 0)
    {
        return cubic;
    }
    else if (strcmp(c, "Cylindrical Image Map") == 0)
    {
        return cylindrical;
    }
    else
    {
        cout << "Unknown texture: " << c << endl;
        assert(false);
    }
}


inline void parse_surf(material **m, const string &p, const char **ptr, const unsigned surf_len)
{
    /* Variable to hold all the parameters */
    texture_info_t          current_info;
    ext_colour_t            rgb(255.0,255.0,255.0);
    fp_t                    vkd             = 0.0;
    fp_t                    vks             = 0.0;
    fp_t                    s               = 0.0;
    fp_t                    vt              = 0.0;
    fp_t                    ri              = 0.0;
    fp_t                    vr              = 0.0;
//    fp_t                    sman            = 0.0;
    const char              *tmp_ptr;
    unsigned                i               = 0;
    unsigned short          short_tmp;
    
    current_info.shader = non;

    while (i < surf_len)
    {
        tmp_ptr = (*ptr) + 4;
        unsigned short sec_len = from_byte_stream<unsigned short>(&tmp_ptr);
//        cout << "processed: " << i << " of " << surf_len << endl;
//        cout << "Parseing: " << *ptr << " with length " << sec_len << endl;
        
        if ((current_info.shader != non) && (strncmp((*ptr + 1), "TEX", 3) == 0))
        {
            current_info.add_shader();
        }
        
        /* Main image colour */
        if (strncmp((*ptr), "COLR", 4) == 0)
        {
            rgb.r = static_cast<fp_t>(static_cast<unsigned char>((*ptr)[6]));
            rgb.g = static_cast<fp_t>(static_cast<unsigned char>((*ptr)[7]));
            rgb.b = static_cast<fp_t>(static_cast<unsigned char>((*ptr)[8]));
        }
        else if (strncmp((*ptr), "FLAG", 4) == 0)
        {
            /* Flags, not currently used */
        }
        /* Integer percentage diffuse co-efficient */
        else if (strncmp((*ptr), "DIFF", 4) == 0)
        {
            if (vkd == 0.0)
            {
                tmp_ptr = (*ptr) + 6;
                vkd = (fp_t)from_byte_stream<unsigned short>(&tmp_ptr)/255.0;
            }
        }
        /* Floating point percentage diffuse co-efficient */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VDIF", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            vkd = from_byte_stream<fp_t>(&tmp_ptr);
        }
        /* Integer percentage specular co-efficient */
        else if (strncmp((*ptr), "SPEC", 4) == 0)
        {
            if (vks == 0.0)
            {
                tmp_ptr = (*ptr) + 6;
                vks = from_byte_stream<fp_t>(&tmp_ptr)/255.0;
            }
        }
        /* Floating point percentage specular co-efficient. */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VSPC", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            vks = from_byte_stream<fp_t>(&tmp_ptr);
        }
        else if (strncmp((*ptr), "GLOS", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            s = (fp_t)from_byte_stream<unsigned short>(&tmp_ptr);
//            s = 360.0 * (s / 1024.0);
//            cout << "GLOS: " << s << endl;
        }
        /* Integer percentage transmittance co-efficient */
        else if (strncmp((*ptr), "TRAN", 4) == 0)
        {
            if (vt == 0.0)
            {
                tmp_ptr = (*ptr) + 6;
                vt = (fp_t)from_byte_stream<unsigned short>(&tmp_ptr)/255.0;
            }
        }
        /* Floating point percentage transmittance co-efficient. */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VTRN", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            vt = from_byte_stream<fp_t>(&tmp_ptr);
//            cout << "VTRN: " << vt << endl;
        }
        /* Integer percentage reflectance co-efficient */
        else if (strncmp((*ptr), "REFL", 4) == 0)
        {
            if (vr == 0.0)
            {
                tmp_ptr = (*ptr) + 6;
                vr = (fp_t)from_byte_stream<unsigned short>(&tmp_ptr)/255.0;
            }
        }
        /* Floating point percentage reflectance co-efficient. */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VRFL", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            vr = from_byte_stream<fp_t>(&tmp_ptr);
//            cout << "VRFL: " << vr << endl;
        }
        else if (strncmp((*ptr), "RFLT", 4) == 0)
        {
        
        }
        /* Refractive index */
        else if (strncmp((*ptr), "RIND", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            ri = from_byte_stream<fp_t>(&tmp_ptr);
        }
        /* Integer percentage luminance co-efficient */
        else if (strncmp((*ptr), "LUMI", 4) == 0)
        {
//            cout << "LUMI" << endl;
        }
        /* Floating point percentage luminance co-efficient. */
        /* Takes precidence over the interge version */
        else if (strncmp((*ptr), "VLUM", 4) == 0)
        {
//            cout << "VLUM" << endl;
        }
        else if (strncmp((*ptr), "TFLG", 4) == 0)
        {
            /* Texture flags, not currently used */
            tmp_ptr = (*ptr) + 6;
            short_tmp = from_byte_stream<unsigned short>(&tmp_ptr);
//            cout << "TFLG: " << short_tmp << endl;
            
            /* Get the normal */
            current_info.tnorm.x = (fp_t)((short_tmp     ) & 0x1);
            current_info.tnorm.y = (fp_t)((short_tmp >> 1) & 0x1);
            current_info.tnorm.z = (fp_t)((short_tmp >> 2) & 0x1);
        }
        else if (strncmp((*ptr), "TSIZ", 4) == 0)
        {
            /* Texture size, not currently used */
            tmp_ptr = (*ptr) + 6;
            current_info.tsiz.x = from_byte_stream<fp_t>(&tmp_ptr);
            current_info.tsiz.y = from_byte_stream<fp_t>(&tmp_ptr);
            current_info.tsiz.z = from_byte_stream<fp_t>(&tmp_ptr);
            
//            cout << "TSIZ: "<< current_info.tsiz.x << ", " << current_info.tsiz.y << ", " << current_info.tsiz.z << endl;
        }
        else if (strncmp((*ptr), "TAAS", 4) == 0)
        {
            /* Texture percentage anti aliasing strength, not currently used */
        }
        /* Texture colour */
        else if (strncmp((*ptr), "TCLR", 4) == 0)
        {
            current_info.trgb.r = (fp_t)(*ptr)[6];
            current_info.trgb.g = (fp_t)(*ptr)[7];
            current_info.trgb.b = (fp_t)(*ptr)[8];
//            cout << "TCLR: "<< current_info.trgb.r << ", " << current_info.trgb.g << ", " << current_info.trgb.b << endl;
        }
        /* Integer texture parameter 0 */
        else if (strncmp((*ptr), "TIP0", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            current_info.tip = from_byte_stream<unsigned short>(&tmp_ptr);
//            cout << "TIP0: " << current_info.tip << endl;
        }
        /* Floating point texture parameter 0 */
        else if (strncmp((*ptr), "TFP0", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            current_info.tfp[0] = from_byte_stream<fp_t>(&tmp_ptr);
//            cout << "TFP0: " << current_info.tfp[0] << endl;
        }
        /* Floating point texture parameter 1 */
        else if (strncmp((*ptr), "TFP1", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            current_info.tfp[1] = from_byte_stream<fp_t>(&tmp_ptr);
//            cout << "TFP1: " << current_info.tfp[1] << endl;
        }
        /* Texture image name, not currently used */
        else if (strncmp((*ptr), "TIMG", 4) == 0)
        {
//            cout << "TIMG: " << (*ptr) + 6 << endl;
            current_info.filename = p + (*ptr + 6);
        }
        else if (strncmp((*ptr), "TWRP", 4) == 0)
        {
            /* Texture wrapping options, not currently used */
            tmp_ptr = (*ptr) + 6;
            current_info.twrp_mode_x = (texture_wrapping_mode_t)from_byte_stream<unsigned short>(&tmp_ptr);
            current_info.twrp_mode_y = (texture_wrapping_mode_t)from_byte_stream<unsigned short>(&tmp_ptr);
//            cout << "TWRP: " << current_info.twrp_mode_x << ", " << current_info.twrp_mode_y << endl;
        }
        /* Texture center */
        else if (strncmp((*ptr), "TCTR", 4) == 0)
        {
            tmp_ptr = (*ptr) + 6;
            current_info.tctr.x = from_byte_stream<fp_t>(&tmp_ptr);
            current_info.tctr.y = from_byte_stream<fp_t>(&tmp_ptr);
            current_info.tctr.z = from_byte_stream<fp_t>(&tmp_ptr);
            
//            cout << "TCTR: "<< current_info.tctr.x << ", " << current_info.tctr.y << ", " << current_info.tctr.z << endl;
        }
        else if (strncmp((*ptr), "TOPC", 4) == 0)
        {
            /* Texture opaqueness, not currently used */
            tmp_ptr = (*ptr) + 6;
            current_info.topc = from_byte_stream<fp_t>(&tmp_ptr);
//            cout << "TOPC: " << current_info.topc << endl;
        }
        /* Algorithmic texture mappers name */
        else if (strncmp((*ptr), "CTEX", 4) == 0)
        {
//            cout << "CTEX: " << (*ptr) + 6 << endl;
            current_info.reset(map_ctex, pick_shader(((*ptr) + 6)));
        }
        else if (strncmp((*ptr), "BTEX", 4) == 0)
        {
            /* Bump map, not currently used */
//            cout << "BTEX: " << (*ptr) + 6 << endl;
            current_info.reset(map_btex, pick_shader(((*ptr) + 6)));
        }
        else if (strncmp((*ptr), "TTEX", 4) == 0)
        {
            /* Transparency texture, not currently used */
//            cout << "TTEX: " << (*ptr) + 6 << endl;
            current_info.reset(map_ttex, pick_shader(((*ptr) + 6)));
        }
        else if (strncmp((*ptr), "RTEX", 4) == 0)
        {
            /* Reflection texture, not currently used */
//            cout << "RTEX: " << (*ptr) + 6 << endl;
            current_info.reset(map_rtex, pick_shader(((*ptr) + 6)));
        }
        else if (strncmp((*ptr), "DTEX", 4) == 0)
        {
            /* Diffuse texture, not currently used */
//            cout << "DTEX: " << (*ptr) + 6 << endl;
            current_info.reset(map_dtex, pick_shader(((*ptr) + 6)));
        }
        else if (strncmp((*ptr), "TAMP", 4) == 0)
        {
            /* Bump texture amplitude, not currently used */
//            cout << "TAMP" << endl;
        }
        else if (strncmp((*ptr), "TVAL", 4) == 0)
        {
            /* Texture value (Percentage modifier for DTEX STEX RTEX TTEX LTEX), not currently used */
        }
        else if (strncmp((*ptr), "SMAN", 4) == 0)
        {
            /* Maximum smooting angle, not currently used */
        }
        /* Catch unknown entities */
        else
        {
            cout << "Unknown entity: "<< *ptr << " found in SURF chunk" << endl;
            assert(false);
        }
        (*ptr) += sec_len + 6;
        i      += sec_len + 6;
        
    }

    if (current_info.shader != non)
    {
        current_info.add_shader();
    }

    /* Create the new material and return */
    *m = new mapper_shader(current_info.ctex, current_info.dtex, current_info.rtex, current_info.ttex, rgb, vkd, vks, s, vt, ri, vr);
}


void lwo_parser(
    ifstream            &lwo_file,
    string              &p,
    light_list          &l, 
    primitive_list      &e,
    list<material *>    &m,
    camera              **c)
{
//    cout << "Parsing LWO file" << endl;
    
    assert(sizeof(unsigned short) == 2);
    assert(sizeof(unsigned      ) == 4);
    assert(sizeof(float         ) == 4);
    
    /* Find the size of the file */
    lwo_file.seekg(0, ios::end);
    size_t len = lwo_file.tellg();
    lwo_file.seekg(0, ios::beg);
    
    /* Read the whole file into a buffer */
    char *buffer = new char [len];
    lwo_file.read(buffer, len);
    const char *at = &buffer[0];

    /* Check that the first 4 characters are "FORM" */
    check_for_chunk(&at, "FORM", 4);
    
    /* Check the declared number of bytes matchs the actual */
    unsigned file_len = from_byte_stream<unsigned>(&at);
    if ((file_len + 8) != len)
    {
        cout << "File length is " << len << " where " << (file_len + 8) << " expected" << endl;
        assert(false);
    }

    /* Check the first chunk is PNTS */
    check_for_chunk(&at, "LWOBPNTS", 8);
    
    /* Gather all the points */
    unsigned nr_of_verts    = from_byte_stream<unsigned>(&at) / 12;
    point_t *all_points     = new point_t[nr_of_verts];
    for (unsigned i = 0; i < nr_of_verts; i++)
    {
        all_points[i].x = from_byte_stream<fp_t>(&at);
        all_points[i].y = from_byte_stream<fp_t>(&at);
        all_points[i].z = from_byte_stream<fp_t>(&at);
    }
    
    /* Check this is the SRFS chunk */
    check_for_chunk(&at, "SRFS", 4);
    
    /* Find the SURF chunk */
    const char *surfs_start;
    unsigned srfs_len       = from_byte_stream<unsigned>(&at);
    unsigned num_of_surfs   = find_surf_chunk(at, &surfs_start, srfs_len);

    /* Parse the names of the surfaces (SRFS chunk) */
    const char **srfs = new const char *[num_of_surfs];
    for (unsigned i = 0; i < num_of_surfs; i++)
    {
        srfs[i] = at;
        at += strlen(srfs[i]) + 1;
        if (*at == 0x00)
        {
            at++;
        }
    }
    
    /* Parse the materials (SURF chunk) */
    material **surf_materials = new material *[num_of_surfs];
    for (unsigned i = 0; i < num_of_surfs; i ++)
    {
        unsigned surf_len = from_byte_stream<unsigned>(&surfs_start);
        if(strcmp(surfs_start, srfs[i]) != 0)
        {
            cout << "Expected: " << srfs[i]     << endl;
            cout << "Actual  : " << surfs_start << endl;
            assert(!"Expected and current surface dont match");
        }
        
        unsigned srf_len = strlen(srfs[i]) + 1;
        srf_len     += srf_len & 0x1;
        surfs_start += srf_len;
        surf_len    -= srf_len;
        
        cout << "Parsing material: " << srfs[i] << endl;
        parse_surf(&surf_materials[i], p, &surfs_start, surf_len);
        m.push_back(surf_materials[i]);
        
        surfs_start += 4;
    }
    
    /* Check this is the POLS chunk */
    check_for_chunk(&at, "POLS", 4);
    
    /* Gather all the polygons */
    vector<point_t> pol_vert;
    unsigned short vert_this_pol    = 0;
    unsigned pols_bytes             = from_byte_stream<unsigned>(&at);
    for(unsigned i = 0; i < pols_bytes; i += (4 + (vert_this_pol << 1)))
    {
        vert_this_pol = from_byte_stream<unsigned short>(&at);
        for (unsigned j = 0; j < vert_this_pol; j++)
        {
            unsigned short vert_num = from_byte_stream<unsigned short>(&at);
            assert(vert_num < nr_of_verts);
            
            pol_vert.push_back(all_points[vert_num]);
        }
        
        /* Parse the material to use */
        short mat_num = from_byte_stream<short>(&at);

        /* Check for detail polygons, but then parse them as normal polygons */
        if (mat_num < 0)
        {
//            cout << "found detail polygons at: 0x" << hex << (unsigned)(at - (&buffer[0])) << dec << endl;
            from_byte_stream<short>(&at);
            mat_num = abs(mat_num);
            i += 2;
        }

        /* Range check the material */
        if((unsigned)mat_num > num_of_surfs)
        {
            cout << "Material " << hex << mat_num << " out of range at " << (unsigned)(at - (&buffer[0])) << dec << endl;
            assert(false);
        }

        /* Create the polygon */
        face_to_triangles(&e, &l, pol_vert, surf_materials[mat_num - 1], false);
        
        /* Clean up */
        pol_vert.clear();
    }

    /* Check this is the SURF chunk, which has already been parsed */
    check_for_chunk(&at, "SURF", 4);
    
    /* Tidy up */
    delete [] buffer;
    delete [] all_points;
    delete [] srfs;
    delete [] surf_materials;
//    cout << "Parsing LWO file complete" << endl;
}
