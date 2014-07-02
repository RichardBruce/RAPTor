#ifndef __CIRCULAR_LIST_H__
#define __CIRCULAR_LIST_H__

/* Standard headers */
#include <stdlib.h>

/* Terrain generator headers */
#include "allocator.h"


namespace raptor_terrain
{
//!	CircularListElement class.
template < typename T > class CircularListElement
{
public:
    T &                                     GetData() { return m_data; }
    const T &                               GetData() const { return m_data; }
    CircularListElement<T> * &				GetNext() { return m_next; }
    CircularListElement<T> * &				GetPrev() { return m_prev; }
    const CircularListElement<T> * &		GetNext() const { return m_next; }
    const CircularListElement<T> * &		GetPrev() const { return m_prev; }        
    //!	Constructor
											CircularListElement(const T & data) {m_data = data;}
											CircularListElement(void){}
    //! Destructor
											~CircularListElement(void){}
private:
    T										m_data;
    CircularListElement<T> *				m_next; 
    CircularListElement<T> *				m_prev;

											CircularListElement(const CircularListElement & rhs);
};


//!	CircularList class.
template < typename T > class CircularList
{
public:
    HeapManager * const     				GetHeapManager() const { return m_heapManager;}
	void									SetHeapManager(HeapManager * const heapManager) { m_heapManager = heapManager;}
    CircularListElement<T> *  &             GetHead() { return m_head;}        
	const CircularListElement<T> *          GetHead() const { return m_head;}
	bool                                    IsEmpty() const { return (m_size == 0);}
	size_t                                  GetSize() const { return m_size; }
	const T &                               GetData() const { return m_head->GetData(); }        
	T &                                     GetData() { return m_head->GetData();}
	bool                                    Delete() ;
    bool                                    Delete(CircularListElement<T> * element);
	CircularListElement<T> *                Add(const T * data = 0);
	CircularListElement<T> *                Add(const T & data);
	bool                                    Next();
	bool                                    Prev();
	void									Clear() { while(Delete());};
    const CircularList&						operator=(const CircularList& rhs);
	//!	Constructor											
											CircularList(HeapManager * heapManager)
											{ 
												m_head = 0; 
												m_size = 0;
                                                m_heapManager = heapManager;
											}
											CircularList(const CircularList& rhs);
	//! Destructor
	virtual								    ~CircularList(void) {Clear();};
private:
	CircularListElement<T> *				m_head;		//!< a pointer to the head of the circular list
	size_t									m_size;		//!< number of element in the circular list
    HeapManager *                           m_heapManager;
    
};
}; /* namespace raptor_terrain */
#include "circular_list.inl"
#endif /* #ifndef __CIRCULAR_LIST_H__*/
