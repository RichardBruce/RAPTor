#ifndef __SAFE_ARRAY_H__
#define __SAFE_ARRAY_H__

/* Standard headers */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define SARRAY_DEFAULT_MIN_SIZE 16

namespace raptor_terrain
{
//!	SArray.
template < typename T, size_t N > class SArray
{
    public:
        T &                     operator[](size_t i)       
                                {
                                    T * const data = Data();
                                    return data[i];
                                }
        const T &               operator[](size_t i) const 
                                {
                                    const T * const data = Data();
                                    return data[i];
                                }
        size_t                  Size() const               
                                { 
                                    return m_size;
                                }
        T * const               Data()
                                { 
                                    return (m_maxSize == N)? m_data0 : m_data;
                                }
        const T * const         Data() const
                                { 
                                    return (m_maxSize == N)? m_data0 : m_data;
                                }
        void                    Clear()
                                {
                                    m_size = 0;
                                    delete [] m_data;
                                    m_data    = 0;
                                    m_maxSize = N;
                                }
        void                    PopBack()
                                {
                                    --m_size;
                                }
		void                    Resize(size_t size)
								{
									if (size > m_maxSize)
									{
                                        T * temp = new T[size];
										memcpy(temp, Data(), m_size*sizeof(T));
                                        delete [] m_data;
                                        m_data = temp;
                                        m_maxSize = size;
									}
								}
        void                    PushBack(const T &  value)
                                {
                                    if (m_size==m_maxSize)
                                    {
                                        size_t maxSize = (m_maxSize << 1);                                            
                                        T * temp = new T[maxSize];
                                        memcpy(temp, Data(), m_maxSize*sizeof(T));
                                        delete [] m_data;
                                        m_data = temp;
                                        m_maxSize = maxSize;
                                    }
                                    T * const data = Data();
                                    data[m_size++] = value;
                                }
        bool                    Find(const T & value, size_t & pos) const
                                {
                                    const T * const data = Data();
                                    for(pos = 0; pos < m_size; ++pos)
                                        if (value == data[pos]) return true;
                                    return false;
                                }
        bool                    Insert(const T & value)
                                {
                                    size_t pos;
                                    if (Find(value, pos)) return false;
                                    PushBack(value);
                                    return true;
                                }
        bool                    Erase(const T & value)
                                {
                                    size_t pos;
                                    T * const data = Data();
                                    if (Find(value, pos))
                                    {
                                        for(size_t j = pos+1;  j < m_size; ++j)
                                            data[j-1] = data[j];
                                        --m_size;
                                        return true;
                                    }
                                    return false;
                                }
        void                    operator=(const SArray & rhs)       
                                {
                                    if (m_maxSize < rhs.m_size)
                                    {
                                        delete [] m_data;
                                        m_maxSize = rhs.m_maxSize;
                                        m_data = new T[m_maxSize];
                                    }
                                    m_size = rhs.m_size;
                                    memcpy(Data(), rhs.Data(), m_size*sizeof(T));
                                }
        void                    Initialize()
                                {
                                    m_data    = 0;
                                    m_size    = 0;
                                    m_maxSize = N;
                                }
                                SArray(const SArray & rhs)
                                {
                                    m_data    = 0;
                                    m_size    = 0;
                                    m_maxSize = N;
                                    *this    = rhs;
                                }
                                SArray()
                                {
                                    Initialize();
                                }
                                ~SArray()
                                {
                                    delete [] m_data;
                                }
    private:
        T                       m_data0[N];
        T *                     m_data;
        size_t                  m_size;
        size_t                  m_maxSize;
};    
}; /* namespace raptor_terrain */
#endif /* #ifndef __SAFE_ARRAY_H__ */
