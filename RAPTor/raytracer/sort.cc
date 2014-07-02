#include "sort.h"


/**********************************************************
  insertion_sort is an implementation of insertion sort.
  
  'a' is the array to be sorted and length is the length
  of this array.
**********************************************************/
void insertion_sort(fp_t *a, int length)
{
    for(int j=1;j<=length;j++)
    {
        fp_t key = a[j];
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
  swap exchanges the values pointed to by 'a' and 'b'.
  
  'a' and 'b' are reference to two values to be swapped.
**********************************************************/
inline void swap(fp_t &a, fp_t &b)
{
    fp_t temp = a;
    a = b;
    b = temp;
}


/**********************************************************
  partition splits the array 'a' about the pivot.
  
  'bottom' is the array index of the lowest unsorted element. 
  'top' is the array index of the highest unsorted element.
  'pivot' is the value to split the list either side of.
**********************************************************/
int partition(fp_t* a, fp_t pivot, int bottom, int top)
{
    while(bottom < top)
    {
        while((top > bottom) && (pivot < a[top]))
        {
            --top;
        }
        swap(a[bottom], a[top]);

        while((bottom < top) && (pivot >= a[bottom]))
        {
            ++bottom;
        }
        swap(a[top], a[bottom]);
    }
    return bottom;
}


/**********************************************************
  quick_sort is an implementation of quick sort.
  
  This function takes the array 'a' and sorts it. 'bottom' 
  is the array index of the lowest unsorted element. 'top'
  is the array index of the highest unsorted element.
**********************************************************/
void quick_sort(fp_t* a, int bottom, int top)
{
    /* Check if the list is sorted */
    if (top > bottom)
    {
        fp_t pivot = a[bottom];

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
            quick_sort(a, bottom, split_point-1);
            quick_sort(a, split_point+1, top);
        }
    }
}


/**********************************************************
  insertion_sort is an implementation of insertion sort.
  
  'a' is the array to be sorted and length is the length
  of this array.
**********************************************************/
void insertion_sort(bb_point_t *a, int length)
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
  swap exchanges the values pointed to by 'a' and 'b'.
  
  'a' and 'b' are reference to two values to be swapped.
**********************************************************/
inline void swap(bb_point_t &a, bb_point_t &b)
{
    bb_point_t temp = a;
    a = b;
    b = temp;
}


/**********************************************************
  partition splits the array 'a' about the pivot.
  
  'bottom' is the array index of the lowest unsorted element. 
  'top' is the array index of the highest unsorted element.
  'pivot' is the value to split the list either side of.
**********************************************************/
int partition(bb_point_t* a, fp_t pivot, int bottom, int top)
{
    while(bottom < top)
    {
        while(pivot < a[top].p && top > bottom)
        {
            --top;
        }
        swap(a[bottom], a[top]);

        while(pivot >= a[bottom].p && bottom < top)
        {
            ++bottom;
        }
        swap(a[top], a[bottom]);
    }
    return bottom;
}


/**********************************************************
  quick_sort is an implementation of quick sort.
  
  This function takes the array 'a' and sorts it. 'bottom' 
  is the array index of the lowest unsorted element. 'top'
  is the array index of the highest unsorted element.
**********************************************************/
void quick_sort(bb_point_t* a, int bottom, int top)
{
    /* Check if the list is sorted */
    if (top > bottom)
    {
        fp_t pivot = a[bottom].p;

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
            quick_sort(a, bottom, split_point-1);
            quick_sort(a, split_point+1, top);
        }
    }
}
