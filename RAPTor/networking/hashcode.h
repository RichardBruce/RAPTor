#pragma once

std::uint32_t murmur_hash_3(const std::uint8_t *const key, const std::uint32_t seed, const std::uint32_t len)
{
    /* Some useful constants */
    const std::uint32_t c1 = 0xcc9e2d51;
    const std::uint32_t c2 = 0x1b873593;
    const std::uint32_t r1 = 15;
    const std::uint32_t r2 = 13;
    const std::uint32_t m = 5;
    const std::uint32_t n = 0xe6546b64;

    /* Union for converting data */
    union
    {
        std::uint8_t bytes[4];
        std::uint32_t word;
    } cast_union;
 
    /* Combine the 4 byte word */
    std::uint32_t hash = seed;
    for (std::uint32_t i = 0; i < (len >> 2); ++i)
    {
        cast_union.bytes[0] = key[(i << 2)    ];
        cast_union.bytes[1] = key[(i << 2) + 1];
        cast_union.bytes[2] = key[(i << 2) + 2];
        cast_union.bytes[3] = key[(i << 2) + 3];

        std::uint32_t word = cast_union.word;
        word = word * c1;
        word = (word << r1) |  (word >> (32 - r1));
        word = word * c2;

        hash = hash ^ word;
        hash = (hash << r2) |  (hash >> (32 - r2));
        hash = hash * m + n;
    }

    /* Combine the remaining bytes */
    for (std::uint32_t i = 0; i < (len & 0x3); ++i)
    {
        cast_union.bytes[i] = key[len - 1 - i];
    }

    std::uint32_t remaining = cast_union.word;
    remaining = remaining * c1;
    remaining = (remaining << r1) |  (remaining >> (32 - r1));
    remaining = remaining * c2;
    hash = hash ^ remaining;

    /* Finalise */
    hash = hash ^ len;
    hash = hash ^ (hash >> 16);
    hash = hash * 0x85ebca6b;
    hash = hash ^ (hash >> 13);
    hash = hash * 0xc2b2ae35;
    hash = hash ^ (hash >> 16);

    return hash;
}
