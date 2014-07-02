#ifndef __ALLOCATOR_H__
#define __ALLOCATOR_H__

/* Standard headers */
#include <stdio.h>

typedef long long			NxI64;
typedef signed int			NxI32;
typedef signed short		NxI16;
typedef signed char			NxI8;

typedef unsigned long long	NxU64;
typedef unsigned int		NxU32;
typedef unsigned short		NxU16;
typedef unsigned char		NxU8;

typedef float				NxF32;
typedef double				NxF64;

namespace raptor_terrain
{

// user provided heap allocator
class MicroHeap
{
public:
  virtual void * micro_malloc(size_t size) = 0;
  virtual void   micro_free(void *p) = 0;
  virtual void * micro_realloc(void *oldMen,size_t newSize) = 0;
};

class MemoryChunk;

class MicroAllocator
{
public:
  virtual void *          malloc(size_t size) = 0;
  virtual void            free(void *p,MemoryChunk *chunk) = 0; // free relative to previously located MemoryChunk
  virtual MemoryChunk *   isMicroAlloc(const void *p) = 0; // returns pointer to the chunk this memory belongs to, or null if not a micro-allocated block.
  virtual NxU32           getChunkSize(MemoryChunk *chunk) = 0;
};

MicroAllocator *createMicroAllocator(MicroHeap *heap,NxU32 chunkSize=32768); // initial chunk size 32k per block.
void            releaseMicroAllocator(MicroAllocator *m);

class HeapManager
{
public:
  virtual void * heap_malloc(size_t size) = 0;
  virtual void   heap_free(void *p) = 0;
  virtual void * heap_realloc(void *oldMem,size_t newSize) = 0;
};


// creates a heap manager that uses micro-allocations for all allocations < 256 bytes and standard malloc/free for anything larger.
HeapManager * createHeapManager(NxU32 defaultChunkSize=32768);
void          releaseHeapManager(HeapManager *heap);

// about 10% faster than using the virtual interface, inlines the functions as much as possible.
void * heap_malloc(HeapManager *hm,size_t size);
void   heap_free(HeapManager *hm,void *p);
void * heap_realloc(HeapManager *hm,void *oldMem,size_t newSize);

void performUnitTests(void);

//

}; /* namespace raptor_terrain */
#endif /* #ifndef __ALLOCATOR_H__ */
