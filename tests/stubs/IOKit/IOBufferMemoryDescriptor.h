#pragma once
#include <IOKit/IOLib.h>
struct IOBufferMemoryDescriptor{static IOBufferMemoryDescriptor*inTaskWithPhysicalMask(void*,unsigned,size_t,unsigned long long);IOReturn prepare();void complete();void release();void*getBytesNoCopy();unsigned long long getPhysicalSegment(unsigned long long,void*);};
