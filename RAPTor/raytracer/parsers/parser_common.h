#ifndef __PARSER_COMMON_H__
#define __PARSER_COMMON_H__

#include "common.h"

#include "light.h"

#include "triangle.h"

/* Shaders */
#include "phong_shader.h"

/* Texture mappers */
#include "mapper_shader.h"
#include "coloured_mapper_shader.h"
#include "perlin_noise_3d_mapper.h"
#include "checker_board_mapper.h"
#include "cubic_mapper.h"
#include "cylindrical_mapper.h"
#include "planar_mapper.h"


namespace raptor_raytracer
{
/* Parsing functions */
/* Byte to... conversion */
template <class T>
inline T from_byte_stream(const unsigned char **s)
{
    /* Shift and or into t */
    T t = 0;
    for (unsigned i = 0; i < sizeof(T); i++)
    {
        t |= ((T)((*s)[i]) << (8 * (sizeof(T) - (i + 1))));
    }
    
    /* Increment the pointer */
    (*s) += sizeof(T);
    
    return t;
}

template <class T>
inline T from_byte_stream(const char **s)
{
    /* Shift and or into t */
    T t = 0;
    for (unsigned i = 0; i < sizeof(T); i++)
    {
        t |= ((T)((unsigned char)(*s)[i]) << (8 * (sizeof(T) - (i + 1))));
    }
    
    /* Increment the pointer */
    (*s) += sizeof(T);
    
    return t;
}

/* float specialisation */
template <>
inline float from_byte_stream<float>(const unsigned char **s)
{
    /* Union to bit cast char to float */
    union 
    {
        unsigned char   c[4];
        float           f;
    } u;
    
    /* Fill in reverse order */
    u.c[0] = (*s)[3];
    u.c[1] = (*s)[2];
    u.c[2] = (*s)[1];
    u.c[3] = (*s)[0];
    
    /* Increment the pointer */
    (*s) += 4;
    
    /* Return the float */
    return u.f;
}

template <>
inline float from_byte_stream<float>(const char **s)
{
    /* Union to bit cast char to float */
    union 
    {
        unsigned char   c[4];
        float           f;
    } u;
    
    /* Fill in reverse order */
    u.c[0] = (unsigned char)(*s)[3];
    u.c[1] = (unsigned char)(*s)[2];
    u.c[2] = (unsigned char)(*s)[1];
    u.c[3] = (unsigned char)(*s)[0];
    
    /* Increment the pointer */
    (*s) += 4;
    
    /* Return the float */
    return u.f;
}


/* Big endian byte to... conversion */
template <class T>
inline T from_big_endian_byte_stream(const unsigned char **s)
{
    /* Shift and or into t */
    T t = 0;
    for (unsigned i=0; i < sizeof(T); i++)
    {
        t |= ((T)((*s)[i]) << (8 * i));
    }
    
    /* Increment the pointer */
    (*s) += sizeof(T);
    
    return t;
}

/* float specialisation */
template <>
inline float from_big_endian_byte_stream<float>(const unsigned char **s)
{
    /* Union to bit cast char to float */
    union 
    {
        unsigned char   c[4];
        float           f;
    } u;
    
    /* Fill in c */
    u.c[0] = (*s)[0];
    u.c[1] = (*s)[1];
    u.c[2] = (*s)[2];
    u.c[3] = (*s)[3];
    
    /* Increment the pointer */
    (*s) += 4;
    
    /* Return the float */
    return u.f;
}


/* Ignore white space */
inline void skip_white_space(const char **a)
{
    while ((**a) == ' ')
    {
        ++(*a);
    }
    
    return;
}


/* String to float conversion */
inline float get_next_float(const std::string &s, size_t *const fst_space)
{
    /* Eat the string until white space is found */
    *fst_space = s.find(' ', *fst_space) + 1;
    
    /* Assume the next thing is a float and convert */
    return atof(s.c_str() + *fst_space);
}

/* Char* to float conversion */
inline float get_next_float(const char **c)
{
    /* Eat the string until white space is found */
    while (((**c) != ' ') && ((**c) != '\t'))
    {
        ++(*c);
    }


    /* Eat the white space */
    while (((**c) == ' ') || ((**c) == '\t'))
    {
        ++(*c);
    }
    
    /* Assume the next thing is a float and convert */
    return atof(*c);
}

inline float get_this_float(const char **c)
{
    /* Eat the white space */
    while (((**c) == ' ') || ((**c) == '\t'))
    {
        ++(*c);
    }
    
    /* Assume the next thing is a float and convert */
    return atof(*c);
}


/* Char* to unsigned conversion */
inline unsigned get_next_unsigned(const char **c)
{
    /* Eat the string until white space is found */
    while (((**c) != ' ') && ((**c) != '/'))
    {
        (*c)++;
    }

    (*c)++;
    while ((**c) == ' ')
    {
        (*c)++;
    }
    
    /* Assume the next thing is an unsigned and convert */
    return (unsigned)atoi(*c);
}

inline unsigned get_this_unsigned(const char **c)
{
    /* Eat the white space */
    while ((**c) == ' ')
    {
        (*c)++;
    }
    
    /* Assume the next thing is an unsigned and convert */
    return (unsigned)atoi(*c);
}

/* Char* to string conversion where string is quoted */
inline std::string get_quoted_string(const char **c)
{
    /* Eat to a quote */
    while ((**c) != '"')
    {
        ++(*c);
    }
    ++(*c);

    /* Remember the start of the string */
    const char *start = *c;
    
    /* Find the end of the string */
    while ((**c) != '"')
    {
        (*c)++;
    }
    
    return std::string(start, ((*c) - start));
}


/* Char* to string conversion */
inline std::string get_next_string(const char **c)
{
    /* Eat the string until white space is found */
    while ((**c) != ' ')
    {
        (*c)++;
    }

    /* Eat the white space */
    while ((**c) == ' ')
    {
        (*c)++;
    }
    
    /* Remember the start of the string */
    const char *start = *c;
    
    /* Find the end of the string */
    while (((**c) != ' ') && ((**c) != '\n') && ((**c) != '\r'))
    {
        (*c)++;
    }
    (*c)--;
    
    return std::string(start, (((*c) + 1) - start));
}


inline std::string get_this_string(const char **c)
{
    /* Eat the white space */
    while ((**c) == ' ')
    {
        (*c)++;
    }
    
    /* Remember the start of the string */
    const char *start = *c;
    
    /* Find the end of the string */
    while (((**c) != ' ') && ((**c) != '\n'))
    {
        (*c)++;
    }
    
    return std::string(start, ((*c) - start));
}


/* Check if this is the end of the line */
inline bool end_of_line(const char *c)
{
    /* Eat the string until the newline or white space is found */
    c++;
    while (((*c) != ' ') && ((*c) != '\n'))
    {
        c++;
    }
    
    return ((*c) == '\n');
}


/* Check if there is a comma after this word */
//inline bool proceeding_comma(char *c)
//{
//    /* Eat the string until the comma or white space is found */
//    c++;
//    while (((*c) != ' ') && ((*c) != ','))
//    {
//        c++;
//    }
//    
//    return ((*c) == ',');
//}


/* Find next line */
inline void find_next_line(const char **c)
{
    /* Eat the string until the newline is found */
    (*c)++;
    while ((**c) != '\n')
    {
        (*c)++;
    }
    
    /* Move passed the new line */
    (*c)++;
}


/* Primitive and material declarations */
/* These functions are all grouped together incase they need to be changed */
/* These functions are all inlines so there is no overhead in using them */


/* Declare material */



/* Declare triangle */
inline void new_triangle(primitive_list *e, std::vector<triangle *> *t, material *m, const point_t &a, const point_t &b, const point_t &c, const bool li, const point_t *vn=nullptr, const point_t *vt=nullptr)
{
    triangle *tr = new triangle(m, a, b, c, li, vn, vt);
    e->push_back(tr);
    if (li)
    {
        t->push_back(tr);
    }
}


/* Declare light */
inline void new_light(light_list *const l, const ext_colour_t &rgb, const point_t &c, const float d, const float r)
{
    l->push_back(light(rgb, c, d, r));
}


inline void new_light(light_list *const l, const ext_colour_t &rgb, const point_t &c, const float d, const std::vector<triangle *> *const t)
{
    l->push_back(light(rgb, c, d, t));
}


inline void new_light(light_list *const l, const ext_colour_t &rgb, const point_t &c, const point_t &n, const float d, const float s_a, const float s_b, const float r)
{
    l->push_back(light(rgb, c, n, d, s_a, s_b, r));
}


inline void new_light(light_list *const l, const ext_colour_t &rgb, const point_t &n, const float d)
{
    l->push_back(light(rgb, n, d));
}

inline point_t find_centre(const std::vector<point_t> &p)
{
    /* Add all the points together */
    point_t total;
    for (const point_t &p_i : p)
    {
        total += p_i;
    }

    /* Average the points and return */
    return total / static_cast<float>(p.size());
}

inline int find_furthest(const std::vector<point_t> &p, const point_t &c)
{
    int max_point = 0;
    float max_dist = 0.0f;

    /* Compare the distance of the points from the centre point */
    unsigned s = p.size();
    for (unsigned i = 0; i < s; i++)
    {
        float dist = magnitude(c - p[i]);
        if (dist > max_dist)
        {
            max_dist  = dist;
            max_point = i;
        }
    }
    
    /* Return the most distant point */
    return max_point;
}


inline bool is_straight_line(const point_t &a, point_t b, point_t c)
{
    b -= a;
    c -= a;
    point_t n;
    cross_product(b, c, &n);
    return (n == 0.0f);
}


inline bool same_side(const point_t &p, const point_t &a, const point_t &ab, const point_t &fn)
{
    /* Normal of the triangle formed by the tested point */
    point_t cp;
    cross_product((p - a), ab, &cp);
    
    /* Check the normal is in the same direction as the face normal */
    if (dot_product(cp, fn) > 0.0f)
    {
        return true;
    }
    else
    {
        return false;
    }
}


inline bool is_in_triangle(const std::vector<point_t> &p, const point_t &a, const point_t &b, const point_t &c, const point_t &fn)
{
    /* Pre-calculate some useful vectors */
    const point_t ab(a - b);
    const point_t ca(c - a);
    const point_t bc(b - c);
    
    /* Check each point if it is inside the triangle */
    for (const point_t &p_i : p)
    {
        /* Check the point is on the same side of AB as C, BC as A and CA as B */
        if (same_side(p_i, a, ab, fn) && same_side(p_i, b, bc, fn) && same_side(p_i, c, ca, fn))
        {
            return true;
        }
    }

    return false;
}


inline void face_to_triangle_edges(std::vector<int> *const tris, std::vector<point_t> &p)
{
    /* Progress tracking */
    unsigned size = p.size();
    std::unique_ptr<char []> invalid(new char [p.size()]);
    memset(invalid.get(), 0, sizeof(char) * p.size());

    /* Find the furthest point from the center */
    point_t  com = find_centre(p);
    unsigned max = find_furthest(p, com);

    /* Use the points either side of this point to find the surface normal */
    unsigned max_m1, max_p1;
    if (max == 0)
    {
        max_m1 = p.size() - 1;
    }
    else
    {
        max_m1 = max - 1;
    }

    if (max == p.size() -1)
    {
        max_p1 = 0;
    }
    else
    {
        max_p1 = max + 1;
    }
        
    point_t face_normal;
    const point_t ab(p[max_m1] - p[max]);
    const point_t bc(p[max] - p[max_p1]);
    
    cross_product(ab, bc, &face_normal);
    normalise(&face_normal);
    
    unsigned no_success = 0;
    while ((p.size() - 2) > 0)
    {
        /* Assume the guess will be a valid triangle */
        bool success = 1;
        
        /* Take the first 3 points of the face to try and form a triangle */
        const point_t a(p[max_m1]);
        const point_t b(p[max   ]);
        const point_t c(p[max_p1]);
        
        /* If the points form a straight line the triangle is invalid */ 
        if (!is_straight_line(a, b, c))
        {
            /* If the triangle has a different normal to the face it is invalid */
            const point_t ab(a - b);
            const point_t bc(b - c);
    
            point_t tri_normal;
            cross_product(ab, bc, &tri_normal);
            normalise(&tri_normal);

            /* If any of the remaining points fall in the triangle it is illegal */ 
            if ((dot_product(tri_normal, face_normal) > 0.0f) && !is_in_triangle(p, a, b, c, face_normal))
            {
                /* Declare if the triangle is valid */
                tris->push_back(max_m1);
                tris->push_back(max   );
                tris->push_back(max_p1);
            }
            else
            {
                success = false;
            }
        }

        /* If the triangle was valid or a straight line */
        /* remove the middle point and move to the next set of points */
        if (success)
        {
            --size; 
            no_success = 0;
            invalid[max] = true;
        }
        else
        {
            max_m1 = max;
            ++no_success;
        }

        max = max_p1;
        do
        {
            ++max_p1;
            if (max_p1 == p.size())
            {
                max_p1 = 0;
            }
        } while (invalid[max_p1] == true);
        
        if ((size == 2) || (no_success == size))
        {
            break;
        }

        assert(no_success < p.size());
    }
}


inline void face_to_triangles(primitive_list *e, light_list *l, std::vector<point_t> &p, material *const m, const bool li, std::vector<point_t> *vn = nullptr, std::vector<point_t> *vt = nullptr, const float d = 0.0f)
{
    /* Progress tracking */
    unsigned size = p.size();
    std::unique_ptr<char []> invalid(new char [p.size()]);
    memset(invalid.get(), 0, sizeof(char) * p.size());

    /* Vertex normal and texture arrays */
    point_t  vn_a[3];
    point_t  vt_a[3];
    point_t *vn_p = nullptr;
    point_t *vt_p = nullptr;

    if ((vn != nullptr) && (!vn->empty()))
    {    
        vn_p = &vn_a[0];
    }
    
    if ((vt != nullptr) && (!vt->empty()))
    {    
        vt_p = &vt_a[0];
    }
    

    /* Triangle list for lights */
    std::vector<triangle *>  *t  = nullptr;
    
    if (li)
    {
        /* To be deleted by the light destructor */
        t   = new std::vector<triangle *>;
    }

    /* Find the furthest point from the center */
    point_t  com = find_centre(p);
    unsigned max = find_furthest(p, com);

    /* Use the points either side of this point to find the surface normal */
    unsigned max_m1, max_p1;
    if (max == 0)
    {
        max_m1 = p.size() - 1;
    }
    else
    {
        max_m1 = max - 1;
    }

    if (max == p.size() -1)
    {
        max_p1 = 0;
    }
    else
    {
        max_p1 = max + 1;
    }
        
    point_t face_normal;
    const point_t ab(p[max_m1] - p[max]);
    const point_t bc(p[max] - p[max_p1]);
    
    cross_product(ab, bc, &face_normal);
    normalise(&face_normal);
    
    unsigned no_success = 0;
    while ((p.size() - 2) > 0)
    {
        /* Assume the guess will be a valid triangle */
        bool success = 1;
        
        /* Take the first 3 points of the face to try and form a triangle */
        const point_t a(p[max_m1]);
        const point_t b(p[max   ]);
        const point_t c(p[max_p1]);
        
        /* If the points form a straight line the triangle is invalid */ 
        if (!is_straight_line(a, b, c))
        {
            /* If the triangle has a different normal to the face it is invalid */
            point_t tri_normal;
            const point_t ab(a - b);
            const point_t bc(b - c);
    
            cross_product(ab, bc, &tri_normal);
            normalise(&tri_normal);

            /* If any of the remaining points fall in the triangle it is illegal */ 
            if ((dot_product(tri_normal, face_normal) > 0.0f) && !is_in_triangle(p, a, b, c, face_normal))
            {
                /* Declare if the triangle is valid */
                if ((vn != nullptr) && (!vn->empty()))
                {
                    vn_a[0] = (*vn)[max_m1];
                    vn_a[1] = (*vn)[max   ];
                    vn_a[2] = (*vn)[max_p1];
                }

                if ((vt != nullptr) && (!vt->empty()))
                {
                    vt_a[0] = (*vt)[max_m1];
                    vt_a[1] = (*vt)[max   ];
                    vt_a[2] = (*vt)[max_p1];
                }

                new_triangle(e, t, m, a, b, c, li, vn_p, vt_p);
            }
            else
            {
                success = false;
            }
        }

        /* If the triangle was valid or a straight line */
        /* remove the middle point and move to the next set of points */
        if (success)
        {
            --size; 
            no_success = 0;
            invalid[max] = true;
        }
        else
        {
            max_m1 = max;
            ++no_success;
        }

        max = max_p1;
        do
        {
            ++max_p1;
            if (max_p1 == p.size())
            {
                max_p1 = 0;
            }
        } while (invalid[max_p1] == true);
        
        if ((size == 2) || (no_success == size))
        {
            break;
        }
        
        assert(no_success < size);
    }
    
    /* Add light */
    if (li)
    {
        new_light(l, ext_colour_t(255.0f, 255.0f, 255.0f), com, 0.0f, t);
    }
}
}; /* namespace raptor_raytracer */

#endif /* #ifndef __PARSER_COMMON_H__ */
