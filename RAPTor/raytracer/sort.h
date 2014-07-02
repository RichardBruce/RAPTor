#ifndef __SORT_H__
#define __SORT_H__

#include "common.h"

struct bb_point_t
{
    fp_t p;
    bool hi;
    bb_point_t(fp_t p=0.0, bool h=false) : p(p), hi(h) { };
};


void quick_sort(fp_t       *a, int bottom, int top);
void quick_sort(bb_point_t *a, int bottom, int top);

void insertion_sort(fp_t       *a, int length);
void insertion_sort(bb_point_t *a, int length);

void radixsort(float *source, long n);

#endif /* #ifndef __SORT_H__ */
