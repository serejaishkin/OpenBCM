//
//  IOPCIEDeviceWrapper.cpp
//  AirportItlwm-Sonoma
//
//  Created by qcwap on 2023/6/27.
//  Copyright © 2023 钟先耀. All rights reserved.
//

#include "IOPCIEDeviceWrapper.hpp"
#include "Apple80211.h"

#include "ItlIwm.hpp"
#include "ItlIwx.hpp"
#include "ItlIwn.hpp"
#ifdef ITLWM_BCM
#include "ItlBcm.hpp"
#endif

#define super IOService
OSDefineMetaClassAndStructors(IOPCIEDeviceWrapper, IOService);

#define  PCI_MSI_FLAGS        2    /* Message Control */
#define  PCI_CAP_ID_MSI        0x05    /* Message Signalled Interrupts */
#define  PCI_MSIX_FLAGS        2    /* Message Control */
#define  PCI_CAP_ID_MSIX    0x11    /* MSI-X */
#define  PCI_MSIX_FLAGS_ENABLE    0x8000    /* MSI-X enable */
#define  PCI_MSI_FLAGS_ENABLE    0x0001    /* MSI feature enabled */

static IOPMPowerState powerStateArray[2] =
{
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {1, kIOPMDeviceUsable, kIOPMPowerOn, kIOPMPowerOn, 0, 0, 0, 0, 0, 0, 0, 0}
};

static void pciMsiSetEnable(IOPCIDevice *device, UInt8 msiCap, int enable)
{
    UInt16 control;
    
    control = device->configRead16(msiCap + PCI_MSI_FLAGS);
    control &= ~PCI_MSI_FLAGS_ENABLE;
    if (enable)
        control |= PCI_MSI_FLAGS_ENABLE;
    device->configWrite16(msiCap + PCI_MSI_FLAGS, control);
}

static void pciMsiXClearAndSet(IOPCIDevice *device, UInt8 msixCap, UInt16 clear, UInt16 set)
{
    UInt16 ctrl;
    
    ctrl = device->configRead16(msixCap + PCI_MSIX_FLAGS);
    ctrl &= ~clear;
    ctrl |= set;
    device->configWrite16(msixCap + PCI_MSIX_FLAGS, ctrl);
}

extern IOWorkLoop *_fWorkloop;

IOWorkLoop *IOPCIEDeviceWrapper::getWorkLoop() const
{
    return _fWorkloop;
}

IOService* IOPCIEDeviceWrapper::
probe(IOService *provider, SInt32 *score)
{
    XYLog("%s\n", __PRETTY_FUNCTION__);
    bool isMatch = false;
    super::probe(provider, score);
    UInt8 msiCap;
    UInt8 msixCap;
    IOPCIDevice* device = OSDynamicCast(IOPCIDevice, provider);
    if (!device)
        return NULL;
    if (ItlIwx::iwx_match(device)) {
        isMatch = true;
        fHalService = new ItlIwx;
    }
    if (!isMatch && ItlIwm::iwm_match(device)) {
        isMatch = true;
        fHalService = new ItlIwm;
    }
    if (!isMatch && ItlIwn::iwn_match(device)) {
        isMatch = true;
        fHalService = new ItlIwn;
    }
#ifdef ITLWM_BCM
    if (!isMatch && ItlBcm::bcm_match(device)) {
        isMatch = true;
        fHalService = new ItlBcm;
        // Read per-device personality config from Info.plist personality dict.
        ChipDeviceConfig devCfg {};
        devCfg.deviceId = device->configRead16(kIOPCIConfigDeviceID);
        OSString *fwFile = OSDynamicCast(OSString, device->getProperty("FirmwareFile"));
        if (fwFile) {
            char buf[64] = {0};
            fwFile->getCString(buf, sizeof(buf));
            strncpy(devCfg.firmwareResourceName, buf, sizeof(devCfg.firmwareResourceName) - 1);
        }
        OSNumber *chipNum = OSDynamicCast(OSNumber, device->getProperty("ChipNumber"));
        if (chipNum) devCfg.chipNumber = (UInt32)chipNum->unsigned32BitValue();
        OSNumber *chipRev = OSDynamicCast(OSNumber, device->getProperty("ChipRev"));
        if (chipRev) devCfg.chipRev = (UInt32)chipRev->unsigned32BitValue();
        OSString *chipType = OSDynamicCast(OSString, device->getProperty("ChipType"));
        if (chipType) {
            char buf[32] = {0};
            chipType->getCString(buf, sizeof(buf));
            if (strcmp(buf, "fullmac") == 0)
                devCfg.chipType = ChipType::kFullmac;
            else
                devCfg.chipType = ChipType::kSoftmac;
            devCfg.chipTypeSet = true;
        }
        OSString *fwSource = OSDynamicCast(OSString, device->getProperty("FirmwareSource"));
        if (fwSource) {
            char buf[32] = {0};
            fwSource->getCString(buf, sizeof(buf));
            if (strcmp(buf, "bundle") == 0)
                devCfg.fwSource = FirmwareSource::kBundle;
            else if (strcmp(buf, "filesystem") == 0)
                devCfg.fwSource = FirmwareSource::kFilesystem;
            else
                devCfg.fwSource = FirmwareSource::kEmbedded;
        }
        OSNumber *d11Base = OSDynamicCast(OSNumber, device->getProperty("D11CoreBase"));
        if (d11Base) devCfg.d11CoreBase = (UInt32)d11Base->unsigned32BitValue();
        OSNumber *needsR4 = OSDynamicCast(OSNumber, device->getProperty("NeedsCortexR4"));
        if (needsR4) {
            devCfg.needsCortexR4 = needsR4->unsigned32BitValue() != 0;
            devCfg.needsCortexR4Set = true;
        }
        ((ItlBcm *)fHalService)->setDeviceConfig(devCfg);
    }
#endif
    if (isMatch) {
        XYLog("%s Found\n", __FUNCTION__);
        device->findPCICapability(PCI_CAP_ID_MSIX, &msixCap);
        if (msixCap)
            pciMsiXClearAndSet(device, msixCap, PCI_MSIX_FLAGS_ENABLE, 0);
        device->findPCICapability(PCI_CAP_ID_MSI, &msiCap);
        if (msiCap)
            pciMsiSetEnable(device, msiCap, 1);
        if (!msiCap && !msixCap) {
            XYLog("%s No MSI cap\n", __FUNCTION__);
            fHalService->release();
            fHalService = NULL;
            return NULL;
        }
        this->pciNub = device;
        return this;
    }
    return NULL;
}

bool IOPCIEDeviceWrapper::
start(IOService *provider)
{
    XYLog("%s\n", __PRETTY_FUNCTION__);
    _fWorkloop = IO80211WorkQueue::workQueue();
    if (!super::start(provider)) {
        return false;
    }
    IOLog("%s::super start succeed\n", getName());
    UInt8 builtIn = 0;
    setProperty("built-in", OSData::withBytes(&builtIn, sizeof(builtIn)));
    PMinit();
    registerPowerDriver(this, powerStateArray, 2);
    provider->joinPMtree(this);
    registerService();
    return true;
}

void IOPCIEDeviceWrapper::
stop(IOService *provider)
{
    XYLog("%s\n", __PRETTY_FUNCTION__);
    PMstop();
    super::stop(provider);
}

IOReturn IOPCIEDeviceWrapper::
setPowerState(unsigned long powerStateOrdinal, IOService *whatDevice)
{
    return IOPMAckImplied;
}
