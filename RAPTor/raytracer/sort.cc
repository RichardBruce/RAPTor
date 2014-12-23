/* Standard headers */
#include <algorithm>
#include <iomanip>

/* Boost headers */

/* Common headers */

/* Ray tracer headers */
#include "simd.h"
#include "sort.h"


namespace raptor_raytracer
{
/**********************************************************
    insertion_sort is an implementation of insertion sort.
    
    'a' is the array to be sorted and length is the length
    of this array.
**********************************************************/
void insertion_sort(float *const a, const int length)
{
    for(int j = 1; j <= length; j++)
    {
        float key = a[j];
        int i    = j-1;
        while((i >= 0) && (a[i] > key))
        {
            a[i+1] = a[i];
            --i;
        }
        a[i+1] = key;
    }
}


/**********************************************************
    partition splits the array 'a' about the pivot.
    
    'bottom' is the array index of the lowest unsorted element. 
    'top' is the array index of the highest unsorted element.
    'pivot' is the value to split the list either side of.
**********************************************************/
int partition(float *const a, float pivot, int bottom, int top)
{
    while(bottom < top)
    {
        while((top > bottom) && (pivot < a[top]))
        {
            --top;
        }
        std::swap(a[bottom], a[top]);

        while((bottom < top) && (pivot >= a[bottom]))
        {
            ++bottom;
        }
        std::swap(a[top], a[bottom]);
    }
    return bottom;
}


/**********************************************************
    quick_sort is an implementation of quick sort.
    
    This function takes the array 'a' and sorts it. 'bottom' 
    is the array index of the lowest unsorted element. 'top'
    is the array index of the highest unsorted element.
**********************************************************/
void quick_sort(float *const a, const int bottom, const int top)
{
    /* Check if the list is sorted */
    if (top > bottom)
    {
        float pivot = a[bottom];

        /* If the list is short call insertion sort instead */
        if ((top - bottom) < 20)
        {
            insertion_sort(&a[bottom], (top - bottom));
        }
        /* else quick sort some more */
        else
        {
            
            int split_point = partition(a, pivot, bottom, top);
            a[split_point] = pivot;
            quick_sort(a, bottom, split_point - 1);
            quick_sort(a, split_point + 1, top);
        }
    }
}


/**********************************************************
    insertion_sort is an implementation of insertion sort.
    
    'a' is the array to be sorted and length is the length
    of this array.
**********************************************************/
void insertion_sort(bb_point_t *const a, const int length)
{
    for(int j = 1; j <= length; ++j)
    {
        bb_point_t key = a[j];
        int i          = j-1;
        while((i >= 0) && (a[i].p > key.p))
        {
            a[i+1] = a[i];
            --i;
        }
        a[i+1] = key;
    }
}


/**********************************************************
    partition splits the array 'a' about the pivot.
    
    'bottom' is the array index of the lowest unsorted element. 
    'top' is the array index of the highest unsorted element.
    'pivot' is the value to split the list either side of.
**********************************************************/
int partition(bb_point_t *const a, float pivot, int bottom, int top)
{
    while(bottom < top)
    {
        while(pivot < a[top].p && top > bottom)
        {
            --top;
        }
        std::swap(a[bottom], a[top]);

        while(pivot >= a[bottom].p && bottom < top)
        {
            ++bottom;
        }
        std::swap(a[top], a[bottom]);
    }
    return bottom;
}


/**********************************************************
    quick_sort is an implementation of quick sort.
    
    This function takes the array 'a' and sorts it. 'bottom' 
    is the array index of the lowest unsorted element. 'top'
    is the array index of the highest unsorted element.
**********************************************************/
void quick_sort(bb_point_t *const a, const int bottom, const int top)
{
    /* Check if the list is sorted */
    if (top > bottom)
    {
        float pivot = a[bottom].p;

        /* If the list is short call insertion sort instead */
        if ((top - bottom) < 20)
        {
            insertion_sort(&a[bottom], (top - bottom));
        }
        /* else quick sort some more */
        else
        {
            
            int split_point = partition(a, pivot, bottom, top);
            a[split_point].p = pivot;
            quick_sort(a, bottom, split_point - 1);
            quick_sort(a, split_point + 1, top);
        }
    }
}

void merge(float *const a, float *const b, const int left_idx, const int right_idx, const int end_idx)
{
    int i0 = left_idx;
    int i1 = right_idx;
 
    /* While there are elements in the left or right lists */
    for (int i = left_idx; i < end_idx; ++i)
    {
        /* If left head exists and is <= existing right head */
        if ((i0 < right_idx) && ((i1 >= end_idx) || (a[i0] < a[i1])))
        {
            b[i] = a[i0++];
        }
        else
        {
            b[i] = a[i1++];
        }
    }
}

/* array a[] has the items to sort; array b[] is a work array */
bool merge_sort(float *a, float *b, const int length, const int insert_sort)
{
    for (int i = 0; i < length; i += 8)
    {
        insertion_sort(&a[i], std::min(7, length - i - 1));
    }

    /* Each 1-element run in a is already "sorted" */
    /* Make successively longer sorted runs of length 2, 4, 8, 16... until whole array is sorted */
    int widths = 0;
    for (int width = 8; width < length; width <<= 1)
    {
        /* Array a is full of runs of length width */
        const int double_width = width << 1;
        for (int i = 0; i < length; i += double_width)
        {
            /* Merge two runs: a[i:i+width-1] and a[i+width:i+2*width-1] to b[] */
            /* or copy a[i:length-1] to b[] ( if(i+width >= length) ) */
            merge(a, b, i, std::min(i + width, length), std::min(i + double_width, length));
        }
        /* Now work array b is full of runs of length 2*width */
        /* Copy array b to array a for next iteration */
        std::swap(a, b);
        ++widths;
        /* Now array a is full of runs of length 2*width */
    }
    
    return (widths & 0x1);
}

void vmerge_no_scalar(float *const a, float *const b, const int left_idx, const int right_idx, const int end_idx)
{
    int i0 = left_idx + SIMD_WIDTH;
    int i1 = right_idx + SIMD_WIDTH;
 
    /* Prime the merge pass */
    vfp_t left(&a[left_idx ]);
    vfp_t right(&a[right_idx]);
    merge(left, right);
    left.store(&b[left_idx]);
 
    /* While there are elements in the left or right runs */
    for (int i = i0; i < (end_idx - SIMD_WIDTH); i += SIMD_WIDTH)
    {
        int load_idx;
        if ((i0 < right_idx) && ((i1 > (end_idx - SIMD_WIDTH)) || (a[i0] < a[i1])))
        {
            load_idx = i0;
            i0 += SIMD_WIDTH;
        }
        else
        {
            load_idx = i1;
            i1 += SIMD_WIDTH;
        }

        vfp_t left(&a[load_idx]);
        merge(left, right);
        left.store(&b[i]);
    }

    right.store(&b[end_idx - SIMD_WIDTH]);
}

void vmerge_16x16_no_scalar(float *const a, float *const b, const int left_idx, const int right_idx, const int end_idx)
{
    int i0 = left_idx + (SIMD_WIDTH << 2);
    int i1 = right_idx + (SIMD_WIDTH << 2);
 
    /* Prime the merge pass */
    vfp_t left0(&a[left_idx ]);
    vfp_t left1(&a[left_idx + SIMD_WIDTH]);
    vfp_t left2(&a[left_idx + (SIMD_WIDTH  * 2)]);
    vfp_t left3(&a[left_idx + (SIMD_WIDTH  * 3)]);

    vfp_t right0(&a[right_idx]);
    vfp_t right1(&a[right_idx + SIMD_WIDTH]);
    vfp_t right2(&a[right_idx + (SIMD_WIDTH  * 2)]);
    vfp_t right3(&a[right_idx + (SIMD_WIDTH  * 3)]);
    merge(left0, left1, left2, left3, right0, right1, right2, right3);

    left0.store(&b[left_idx]);
    left1.store(&b[left_idx + SIMD_WIDTH]);
    left2.store(&b[left_idx + (SIMD_WIDTH  * 2)]);
    left3.store(&b[left_idx + (SIMD_WIDTH  * 3)]);
 
    /* While there are elements in the left or right runs */
    for (int i = i0; i < (end_idx - (SIMD_WIDTH << 2)); i += (SIMD_WIDTH << 2))
    {
        int load_idx;
        if ((i0 < right_idx) && ((i1 > (end_idx - (SIMD_WIDTH << 2))) || (a[i0] < a[i1])))
        {
            load_idx = i0;
            i0 += (SIMD_WIDTH << 2);
            _mm_prefetch(&a[i0 + 32], _MM_HINT_T0);
        }
        else
        {
            load_idx = i1;
            i1 += (SIMD_WIDTH << 2);
            _mm_prefetch(&a[i1 + 32], _MM_HINT_T0);
        }

        vfp_t left0(&a[load_idx]);
        vfp_t left1(&a[load_idx + SIMD_WIDTH]);
        vfp_t left2(&a[load_idx + (SIMD_WIDTH  * 2)]);
        vfp_t left3(&a[load_idx + (SIMD_WIDTH  * 3)]);

        merge(left0, left1, left2, left3, right0, right1, right2, right3);

        left0.store(&b[i]);
        left1.store(&b[i + SIMD_WIDTH]);
        left2.store(&b[i + (SIMD_WIDTH * 2)]);
        left3.store(&b[i + (SIMD_WIDTH * 3)]);
    }

    right0.store(&b[end_idx - (SIMD_WIDTH * 4)]);
    right1.store(&b[end_idx - (SIMD_WIDTH * 3)]);
    right2.store(&b[end_idx - (SIMD_WIDTH * 2)]);
    right3.store(&b[end_idx - SIMD_WIDTH]);
}


void vmerge_8x8_no_scalar(float *const a, float *const b, const int left_idx, const int right_idx, const int end_idx)
{
    int i0 = left_idx + (SIMD_WIDTH << 1);
    int i1 = right_idx + (SIMD_WIDTH << 1);
 
    /* Prime the merge pass */
    vfp_t left0(&a[left_idx ]);
    vfp_t left1(&a[left_idx + SIMD_WIDTH]);

    vfp_t right0(&a[right_idx]);
    vfp_t right1(&a[right_idx + SIMD_WIDTH]);
    merge(left0, left1, right0, right1);

    left0.store(&b[left_idx]);
    left1.store(&b[left_idx + SIMD_WIDTH]);
 
    /* While there are elements in the left or right runs */
    for (int i = i0; i < (end_idx - (SIMD_WIDTH << 1)); i += (SIMD_WIDTH << 1))
    {
        int load_idx;
        if ((i0 < right_idx) && ((i1 > (end_idx - (SIMD_WIDTH << 1))) || (a[i0] < a[i1])))
        {
            load_idx = i0;
            i0 += (SIMD_WIDTH << 1);
            _mm_prefetch(&a[i0 + 16], _MM_HINT_T0);
        }
        else
        {
            load_idx = i1;
            i1 += (SIMD_WIDTH << 1);
            _mm_prefetch(&a[i1 + 16], _MM_HINT_T0);
        }

        vfp_t left0(&a[load_idx]);
        vfp_t left1(&a[load_idx + SIMD_WIDTH]);

        merge(left0, left1, right0, right1);

        left0.store(&b[i]);
        left1.store(&b[i + SIMD_WIDTH]);
    }

    right0.store(&b[end_idx - (SIMD_WIDTH << 1)]);
    right1.store(&b[end_idx - SIMD_WIDTH]);
}


void vmerge(float *const a, float *const b, const int left_idx, const int right_idx, const int end_idx)
{
    /* Not enough data for vector merge so use scalar code */
    if ((end_idx - right_idx) < SIMD_WIDTH)
    {
        merge(a, b, left_idx, right_idx, end_idx);
        return;
    }


    int i0 = left_idx + SIMD_WIDTH;
    int i1 = right_idx + SIMD_WIDTH;
 
    /* Prime the merge pass */
    vfp_t left(&a[left_idx ]);
    vfp_t right(&a[right_idx]);
    merge(left, right);
    left.store(&b[left_idx]);

    /* Vector iteratations */
    int i = i0;
    while (i1 <= (end_idx - SIMD_WIDTH))
    {
        /* If left run head exists and is <= existing right run head */
        int load_idx;
        if ((i0 < right_idx) && (a[i0] < a[i1]))
        {
            load_idx = i0;
            i0 += SIMD_WIDTH;
        }
        else
        {
            load_idx = i1;
            i1 += SIMD_WIDTH;
        }

        vfp_t left(&a[load_idx]);
        merge(left, right);
        left.store(&b[i]);

        i += SIMD_WIDTH;
    }

    /* Scalar iterations if required */
    if (i != (end_idx - SIMD_WIDTH))
    {
        if (i0 == right_idx)
        {
            i0 -= SIMD_WIDTH;
            right.store(&a[i0]);
        }
        else
        {
            i1 -= SIMD_WIDTH;
            right.store(&a[i1]);
        }

        for (; i < end_idx; ++i)
        {
            /* If left run head exists and is <= existing right run head */
            if ((i0 < right_idx) && ((i1 >= end_idx) || (a[i0] < a[i1])))
            {
                b[i] = a[i0++];
            }
            else
            {
                b[i] = a[i1++];
            }
        }
    }
    /* Clear out the last element */
    else
    {
        right.store(&b[i]);
    }
}


bool vmerge_sort(float *a, float *b, const int length)
{
    /* Check if we have enough data to be worth vectorising */
    if (length < (SIMD_WIDTH * SIMD_WIDTH))
    {
        return merge_sort(a, b, length);
    }

    /* Sort within registers */
    int i;
    for (i = 0; i <= (length - (SIMD_WIDTH * SIMD_WIDTH)); i += (SIMD_WIDTH * SIMD_WIDTH))
    {
        /* Odd-even sort */
        const vfp_t in_reg0(&a[i                   ]);
        const vfp_t in_reg1(&a[i +      SIMD_WIDTH ]);
        const vfp_t in_reg2(&a[i + (2 * SIMD_WIDTH)]);
        const vfp_t in_reg3(&a[i + (3 * SIMD_WIDTH)]);

        const vfp_t min_l(min(in_reg0, in_reg1));
        const vfp_t min_h(min(in_reg2, in_reg3));
        const vfp_t max_l(max(in_reg0, in_reg1));
        const vfp_t max_h(max(in_reg2, in_reg3));

        const vfp_t min_max(max(min_l, min_h));
        const vfp_t max_min(min(max_l, max_h));

        vfp_t a0(min(min_l, min_h));
        vfp_t a1(min(min_max, max_min));
        vfp_t a2(max(min_max, max_min));
        vfp_t a3(max(max_l, max_h));

        /* Transpose to SIMD_WIDTH sorted lists */
        transpose(a0, a1, a2, a3);
        a0.store(&a[i                   ]);
        a1.store(&a[i +      SIMD_WIDTH ]);
        a2.store(&a[i + (2 * SIMD_WIDTH)]);
        a3.store(&a[i + (3 * SIMD_WIDTH)]);
    }

    /* Sort any remain data */
    if (i < length)
    {
        insertion_sort(&a[i], length - i - 1);
    }

    /* Bitonic merge arrays */
    /* 4x4 merge */
    for (i = 0; i <= (length - (SIMD_WIDTH << 1)); i += (SIMD_WIDTH << 1))
    {
        vmerge_no_scalar(a, b, i, i + SIMD_WIDTH, i + (SIMD_WIDTH << 1));
    }

    merge(a, b, i, std::min(i + SIMD_WIDTH, length), std::min(i + (SIMD_WIDTH << 1), length));
    std::swap(a, b);
       
    /* 8x8 merge */
#if 0
    int widths = 1;
    for (int width = (SIMD_WIDTH << 1); width < length; width <<= 1)
    {
        /* Array a is full of runs of length width. */
        const int double_width = width << 1;
        for (i = 0; i <= (length - double_width); i += double_width)
        {
            vmerge_8x8_no_scalar(a, b, i, std::min(i + width, length), std::min(i + double_width, length));
        }

        merge(a, b, i, std::min(i + width, length), std::min(i + double_width, length));
        std::swap(a, b);
        ++widths;
    }
#else
    for (i = 0; i <= (length - (SIMD_WIDTH << 2)); i += (SIMD_WIDTH << 2))
    {
        vmerge_8x8_no_scalar(a, b, i, i + (SIMD_WIDTH << 1), i + (SIMD_WIDTH << 2));
    }

    merge(a, b, i, std::min(i + (SIMD_WIDTH << 1), length), std::min(i + (SIMD_WIDTH << 2), length));
    std::swap(a, b);

    int widths = 2;
    for (int width = (SIMD_WIDTH << 2); width < length; width <<= 1)
    {
        /* Array a is full of runs of length width. */
        const int double_width = width << 1;
        for (i = 0; i <= (length - double_width); i += double_width)
        {
            vmerge_16x16_no_scalar(a, b, i, i + width, i + double_width);
        }

        merge(a, b, i, std::min(i + width, length), std::min(i + double_width, length));
        std::swap(a, b);
        ++widths;
    }
#endif

    return !(widths & 0x1);
}


/* Radix sort */
inline void float_flip_inplace(unsigned int *const f)
{
    unsigned int mask = -static_cast<int>((*f) >> 31) | 0x80000000;
    (*f) ^= mask;
}

inline unsigned int inverse_float_flip(unsigned int f)
{
    unsigned int mask = ((f >> 31) - 1) | 0x80000000;
    return f ^ mask;
}

__attribute__((optimize("unroll-loops")))
void radix_sort(float *const farray, float *const sorted, const int elements)
{
    unsigned int *sort = (unsigned int*)sorted;
    unsigned int *array = (unsigned int*)farray;

    const int histogram_size = 2048;
    unsigned int hist0[histogram_size * 3];
    unsigned int *hist1 = hist0 + histogram_size;
    unsigned int *hist2 = hist1 + histogram_size;
    memset(hist0, 0, histogram_size * 3 * sizeof(float));

    /* Build histograms */
    for (int i = 0; i < elements; ++i)
    {
        float_flip_inplace(&array[i]);
        ++hist0[ array[i]        & 0x7FF];
        ++hist1[(array[i] >> 11) & 0x7FF];
        ++hist2[ array[i] >> 22         ];
    }
    
    /* Sum the histograms */
    unsigned int sum0 = 0;
    unsigned int sum1 = 0;
    unsigned int sum2 = 0;
    for (int i = 0; i < histogram_size; ++i)
    {
        const unsigned int tmp0 = hist0[i] + sum0;
        hist0[i] = sum0 - 1;
        sum0 = tmp0;

        const unsigned int tmp1 = hist1[i] + sum1;
        hist1[i] = sum1 - 1;
        sum1 = tmp1;

        const unsigned int tmp2 = hist2[i] + sum2;
        hist2[i] = sum2 - 1;
        sum2 = tmp2;
    }

    /* Move based on histogram 0 */
    for (int i = 0; i < elements; ++i)
    {
        const unsigned int data = array[i];
        const unsigned int pos = data & 0x7FF;
        sort[++hist0[pos]] = data;
    }

    /* Move based on histogram 1 */
    for (int i = 0; i < elements; ++i)
    {
        const unsigned int data = sort[i];
        const unsigned int pos = (data >> 11) & 0x7FF;
        array[++hist1[pos]] = data;
    }

    /* Move based on histogram 2 and flip data back */
    for (int i = 0; i < elements; ++i)
    {
        const unsigned int data = array[i];
        const unsigned int pos = data >> 22;
        sort[++hist2[pos]] = inverse_float_flip(data);
    }
}
}; /* namespace raptor_raytracer */
