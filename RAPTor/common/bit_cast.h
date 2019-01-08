#pragma once
/* Common headers */
#include <cassert>


/* Template for casting bits from one type to another */
template<class T, class S>
inline S bit_cast(const T t_in)
{
    assert(sizeof(T) == sizeof(S));

    /* Create union of the types */
    union
    {
        T t;
        S s;
    } cast_union;

    /* Input one type */
    cast_union.t = t_in;

    /* And return the other */
    return cast_union.s;
}
