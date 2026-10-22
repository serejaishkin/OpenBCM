#pragma once
#include <IOKit/IOLib.h>
struct IOPCIDevice{void setMemoryEnable(bool);void setBusMasterEnable(bool);IOMemoryMap*mapDeviceMemoryWithRegister(unsigned);uint32_t configRead32(uint32_t);void configWrite32(uint32_t,uint32_t);};
