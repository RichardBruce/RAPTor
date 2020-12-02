#pragma once

#include "common.h"

struct bb_point_t
{
    float   p;
    bool    hi;
    bb_point_t(float p = 0.0f, bool h = false) : p(p), hi(h) { };
};


void quick_sort(float      *const a, const int bottom, const int top);
void quick_sort(bb_point_t *const a, const int bottom, const int top);

void insertion_sort(float      *const a, const int length);
void insertion_sort(bb_point_t *const a, const int length);

bool merge_sort(float *a, float *b, const int length, const int insert_sort = 8);
bool vmerge_sort(float *a, float *b, const int length);

void radix_sort(float *const farray, float *const sorted, const int elements);
void radix_sort(double *const farray, double *const sorted, const int elements);
