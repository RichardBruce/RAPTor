#pragma once

#include "common.h"


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
inline unsigned int get_next_unsigned(const char **c)
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
    return static_cast<unsigned int>(atoi(*c));
}

inline unsigned int get_this_unsigned(const char **c)
{
    /* Eat the white space */
    while ((**c) == ' ')
    {
        (*c)++;
    }
    
    /* Assume the next thing is an unsigned and convert */
    return static_cast<unsigned int>(atoi(*c));
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
}; /* namespace raptor_raytracer */
