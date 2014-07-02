/*! \file x86timer.hpp
    \brief Timing based on elapsed CPU cycles on X86 or compatible CPUs using rdtsc register
*/

#ifndef X86TIMER_HPP
#define X86TIMER_HPP

#if defined(__i386__) || defined(__x86_64__)
#if defined(__GNUC__) || defined(_MSC_VER)

#ifndef unix
#include "SDL/SDL.h"
#endif

#ifdef _MSC_VER
typedef unsigned __int64 uint64_t;
typedef __int64 int64_t;
#endif

#ifdef __GNUC__
/** CPU cycles since processor startup */
inline uint64_t rdtsc() {
  uint32_t lo, hi;
 /* We cannot use "=A", since this would use %rax on x86_64 */
  __asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
  return (uint64_t)hi << 32 | lo;
}
#endif

#ifdef _MSC_VER
__declspec(naked)
unsigned __int64 __cdecl rdtsc(void)
{
   __asm
      {
            rdtsc
                  ret       ; return value at EDX:EAX
                     }
                     }
#endif

/** Accurate timing and sleeping on pentium or newer x86 computers */
class x86timer {
private:

uint64_t fstart,fend;
double clocks_per_nanosecond;
uint64_t start_,end_;
double sum;
int times;

public:

/** Nanosecond sleep */
void nanosleep(uint64_t nanoseconds)
{
uint64_t begin=rdtsc();
uint64_t now;
uint64_t dtime = nanoseconds*clocks_per_nanosecond;
do
	{
	now=rdtsc();
	} 
	while ( (now-begin) < dtime);
}

void start()
{
start_=rdtsc();
}

/** Returns elapsed nanoseconds */
uint64_t stop()
{
end_=rdtsc();
return double(end_-start_)/clocks_per_nanosecond;
}

x86timer() {

sum=0;
times=0;
uint64_t bench1=0;
uint64_t bench2=0;
clocks_per_nanosecond=0;

bench1=rdtsc();

#ifdef unix
usleep(250000); // 1/4 second
#else
SDL_Delay(250);
#endif

bench2=rdtsc();

clocks_per_nanosecond=bench2-bench1;
clocks_per_nanosecond*=4.0e-9;
}

};

#endif
#endif
#endif
