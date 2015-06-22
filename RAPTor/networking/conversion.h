#pragma once

namespace raptor_networking
{
template<class IN, class OUT>
OUT from_big_endian_byte_array(const IN *bytes)
{
    /* Check the input and output sizes */
    static_assert(((sizeof(OUT) % sizeof(IN)) == 0), "Error: Converting between sizes that arent integer multiples of eachother");
    static_assert((sizeof(OUT) >= sizeof(IN)), "Error: Conversion results in a reduction in size");

    /* Union to bit cast */
    union 
    {
        IN  c[sizeof(OUT)];
        OUT f;
    } u;

    /* Fill the union */
    for (unsigned int i = 0; i < (sizeof(OUT) / sizeof(IN)); i++)
    {
         u.c[i] = bytes[i];
    }

    /* return the converted value */
    return u.f;
}

template<class IN, class OUT>
void to_big_endian_byte_array(const IN val, OUT *const bytes)
{
    /* Check the input and output sizes */
    static_assert(((sizeof(IN) % sizeof(OUT)) == 0), "Error: Converting between sizes that arent integer multiples of eachother");
    static_assert((sizeof(IN) >= sizeof(OUT)), "Error: Conversion results in an increase in size");

    /* Union to bit cast */
    union 
    {
        OUT c[sizeof(IN)];
        IN  f;
    } u;

    /* Fill the union */
    u.f = val;

    /* Copy out the converted values */
    for (unsigned int i = 0; i < (sizeof(IN) / sizeof(OUT)); i++)
    {
         bytes[i] = u.c[i];
    }

    return;
}
}; /* namespace raptor_networking */
