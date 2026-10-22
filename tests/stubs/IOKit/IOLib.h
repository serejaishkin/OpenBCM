#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#define IOLog printf
inline void IODelay(unsigned){} inline void IOSleep(unsigned){}
inline void* IOMalloc(size_t n){return malloc(n);} inline void IOFree(void*p,size_t){free(p);}
typedef int IOReturn; enum{kIOReturnSuccess=0};
enum{kIODirectionInOut=3,kIOMemoryPhysicallyContiguous=0x10,kIOPCIConfigBaseAddress0=0x10,kIOPCIConfigBaseAddress2=0x18};
extern void* kernel_task;
struct IOMemoryMap{void*getVirtualAddress();unsigned long long getLength();void release();};
