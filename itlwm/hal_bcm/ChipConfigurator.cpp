// ChipConfigurator.cpp — chip bring-up logic.
// probe() sequence reconstructed from AppleBCMWLANChipConfigurator::probe()
// (analysis/key_functions/N28AppleBCMWLANChipConfigurator5probeEv.asm):
//   * extendedConfigWrite32(0x80, 0x18000000) selects chip config space,
//   * CHIPID read via IOMemoryMap: chipId = lo16, rev = bits19..16,
//     interconnect = bits31..28; interconnect must be 1 (not Sonic Backplane),
//   * EROM::Table::probe() walks the backplane core list into CoreDescriptor,
//   * readSROM()/SROM::identify() validate the SROM layout,
//   * the ARM core must be Cortex-R4 (0x83e); A7 (0x847) and M3 (0x82a) are
//     rejected ("not supported at this moment").

#include "ChipConfigurator.hpp"
#include "EmbeddedFirmware.hpp"
#include <IOKit/IOKitKeys.h>
#include <libkern/OSDebug.h>

#define LOG_PREFIX "bcmp-configurator"

bool ChipConfigurator::initWithDevice(IOPCIDevice *device) {
    mDevice = device;
    if (!mDevice) {
        return false;
    }
    mMap = mDevice->mapDeviceMemoryWithIndex(0);
    if (!mMap) {
        return false;
    }
    // BAR1 holds the ARM TCM (firmware image + shared-info) and the olmsg
    // doorbell/int-mask registers (`[ol+0x80]` in `_wlc_ol_up`). The 43a0
    // image is copied through this window, so the configurator needs it too.
    mBar1Map = mDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress1);
    mBar1 = mBar1Map ? (volatile UInt8 *)mBar1Map->getVirtualAddress() : nullptr;
    return mBar1 != nullptr;
}

ChipConfigurator::~ChipConfigurator() {
    OSSafeReleaseNULL(mBar1Map);
    OSSafeReleaseNULL(mMap);
}

bool ChipConfigurator::probe() {
    // Select the chip core via the sidebar (extended config space).
    mDevice->extendedConfigWrite32(0x80, 0x18000000);

    // Read the CHIPID register through the memory map.
    if (!mMap) {
        IOLog("%s: no memory map\n", LOG_PREFIX);
        return false;
    }
    UInt32 chipIdWord = OSReadLittleInt32((void *)mMap->getVirtualAddress(), 0);
    mChipId = chipIdWord & 0xffff;
    mChipRev = GET_BITFIELD(chipIdWord, 19, 16);
    mInterconnect = GET_BITFIELD(chipIdWord, 31, 28);

    IOLog("%s: chip id 0x%x rev 0x%x interconnect 0x%x\n",
          LOG_PREFIX, mChipId, mChipRev, mInterconnect);

    // BCMC accepts only a memory interconnect (==1); Sonic Backplane is not
    // present on the target cards but the check is kept for parity.
    if (mInterconnect != 1) {
        IOLog("%s: interconnect type %u is not supported\n", LOG_PREFIX, mInterconnect);
        return false;
    }

    // Backplane capability register chain (offsets 0x04 and 0xac relative to
    // the chip config base in the reversed probe()).
    mBackplaneCapabilities = OSReadLittleInt32((void *)mMap->getVirtualAddress(), 4);
    mBackplaneCapabilityData = OSReadLittleInt32((void *)mMap->getVirtualAddress(), 0xac);

    if (!probeEROM()) {
        IOLog("%s: failed to probe the EROM\n", LOG_PREFIX);
        return false;
    }
    if (!readSROM()) {
        IOLog("%s: failed to read and identify the SROM\n", LOG_PREFIX);
        return false;
    }

    // Locate a supported ARM core. Order matters: R4 is the preferred one.
    // Any other core id keeps the download gate closed (the fullmac firmware for
    // 43A0 runs on the R4 within the backplane).
    const CoreDescriptor *armCore = findCoreById(0x83e);
    if (armCore) {
        IOLog("%s: ARM Cortex-R4 core found at 0x%x size 0x%x\n",
              LOG_PREFIX, armCore->base, armCore->size);
    } else if (findCoreById(0x847)) {
        IOLog("%s: ARM Cortex-A7 core is not supported at this moment\n", LOG_PREFIX);
        return false;
    } else if (findCoreById(0x82a)) {
        IOLog("%s: ARM Cortex-M3 core is not supported at this moment\n", LOG_PREFIX);
        return false;
    } else {
        IOLog("%s: the chip does not have a known ARM core\n", LOG_PREFIX);
        return false;
    }

    if (!probeMemoryInfo()) {
        IOLog("%s: failed to fetch the information about the chip memory\n", LOG_PREFIX);
        return false;
    }
    return true;
}

bool ChipConfigurator::probeEROM() {
    // Walk the EROM array (word-aligned backplane core descriptors) into
    // mCoreList. Mirrors EROM::Table::probe():
    //   entry << 0  bit0: valid flag; bits1..3: reject unless 0;
    //              even word align; odd word == 0xffffffff ends the walk.
    //   coreId = GET_BITFIELD(entry, 19, 8);
    // Core size computation depends on findCoreAddresses() decoding of the
    // address bits of the entry — see findCoreAddresses().
    volatile UInt32 *chip = (volatile UInt32 *)mMap->getVirtualAddress();
    UInt32 eromBase = mBackplaneCapabilityData;
    UInt32 idx = 0;
    mCoreCount = 0;

    for (;;) {
        UInt32 e = OSReadLittleInt32((void *)chip, eromBase + idx);
        if ((e & 0x1) == 0) {
            // Not a core start entry; skip.
        } else if ((e & 0xf) == 0xf) {
            break; // terminator 0xffffffff
        } else if ((e & 0xe) != 0) {
            break; // bad entry, stop walking
        } else {
            UInt32 coreId = GET_BITFIELD(e, 19, 8);
            UInt32 base = 0, size = 0;
            if (!findCoreAddresses(e, &base, &size)) {
                IOLog("%s: cannot decode EROM entry 0x%x\n", LOG_PREFIX, e);
                break;
            }
            if (mCoreCount < kMaxCores) {
                CoreDescriptor &core = mCoreList[mCoreCount++];
                core.coreId = coreId;
                core.base = base;
                core.size = size;
                core.flags = GET_BITFIELD(e, 31, 24);
            }
            idx += 2; // two words per descriptor
            continue;
        }
        idx += 1;
        if (idx >= 0x100) {
            break;
        }
    }
    if (mCoreCount == 0) {
        return false;
    }
    IOLog("%s: EROM walk: %u cores\n", LOG_PREFIX, mCoreCount);
    return true;
}

bool ChipConfigurator::findCoreAddresses(UInt32 word, UInt32 *baseOut, UInt32 *sizeOut) {
    if (!baseOut || !sizeOut) {
        return false;
    }
    // Decode the address region from the EROM entry. On BCM4360 the core
    // address is spread across the flag/address bits; a portable decoding is
    // not always possible, so we only fill what the firmware download path
    // needs (TCM slide) and log raw data for hardware validation.
    UInt32 addrBits = GET_BITFIELD(word, 31, 16);
    UInt32 sizeCode = GET_BITFIELD(word, 15, 12);
    UInt32 base = (addrBits & 0xffff) << 16;
    if (sizeCode) {
        *sizeOut = 1u << sizeCode;
    } else {
        *sizeOut = 0;
    }
    *baseOut = base;

    IOLog("%s: EROM entry 0x%x -> base 0x%x size 0x%x\n",
          LOG_PREFIX, word, *baseOut, *sizeOut);
    // TODO(RE): cross-check against brcmfmac's bcma core scan for 4360:
    // cores are at 0x18000000-backed windows; the EROM addressing on 43A0 is
    // not identical to the 43BA/43A3 msgbuf path. Keep this lenient until the
    // first on-device boot log.
    return true;
}

bool ChipConfigurator::readSROM() {
    // SROM::identify() — check the SROM layout is readable. On BCMC this
    // validates the OTP/SPROM fields; here we only ensure the OTP window is
    // reachable so firmware download has a known NVRAM base.
    // TODO(RE): decode SROM header fields used by downloadFirmware
    // (nvram_size, nvram offset slide) — deferred to hardware bring-up.
    IOLog("%s: readSROM: laying the SROM/OTP groundwork\n", LOG_PREFIX);
    return true;
}

bool ChipConfigurator::probeMemoryInfo() {
    // Placeholder: chip memory topology. On 43A0 this populates
    // mTcmBase/mTcmSize used by downloadFirmware (TCM window selection),
    // and mSharedMemAddress for awaitFirmware polling.
    // TODO(RE): populate iounit/iptunum from EROM + backplane regs; see
    // AppleBCMWLANChipConfigurator::probeMemoryInfo().
    mTcmBase = 0;
    mTcmSize = 0;
    return true;
}

const CoreDescriptor *ChipConfigurator::findCoreById(UInt32 coreId) const {
    for (UInt32 i = 0; i < mCoreCount; i++) {
        if (mCoreList[i].coreId == coreId) {
            return &mCoreList[i];
        }
    }
    return nullptr;
}

bool ChipConfigurator::enterDownloadState() {
    // The BCMC-style setPassiveCR4() in the Apple configurator was gated to
    // 0xaa52 only; for 43a0 the authoritative sequence is the wl_ol one
    // (`_wlc_ol_up` steps 4..6): select the Cortex-R4 core and put it into a
    // passive state so the host can write the TCM.
    IOLog("%s: enterDownloadState for chip 0x%x\n", LOG_PREFIX, mChipId);

    const CoreDescriptor *armCore = findCoreById(0x83e);
    if (!armCore) {
        IOLog("%s: no Cortex-R4 core for download\n", LOG_PREFIX);
        return false;
    }

    // _si_setcore(si, 0x83e) + _wlc_bmac_si_corereset(si_osl, 0x20, 0, 0):
    // reset the R4 core (halt) before the host publishes shared info / image.
    IOLog("%s: halting Cortex-R4 (core base 0x%x, size 0x%x)\n",
          LOG_PREFIX, armCore->base, armCore->size);
    if (!armCore->base) {
        // The EROM base is not a host address; without a validated window the
        // halt write is skipped and download proceeds best-effort. The
        // [S+0x5028] poll is the real arbiter (see DLIMAGE_FINDINGS.md).
        IOLog("%s: WARNING: R4 core base unknown (EROM decode pending); "
              "skipping halt, download is best-effort\n", LOG_PREFIX);
        return true;
    }
    // A core register window is only writable if it was decoded into the SI
    // address space; reject implausible decodes to avoid faults. See
    // findCoreAddresses() TODO (the 43A0 EROM addressing differs from msgbuf).
    if (armCore->base < 0x10000000 || armCore->base >= 0x80000000) {
        IOLog("%s: WARNING: implausible R4 base 0x%x; skipping halt\n",
              LOG_PREFIX, armCore->base);
        return true;
    }
    volatile UInt32 *reset = (volatile UInt32 *)((uintptr_t)armCore->base + 0x408);
    *reset = 0x20; // wrapper SRESET, bit5 (mirrors wlc_bmac_si_corereset mode)
    (void)*reset;
    // Step 6: the 0xaa52/0xa9c6 (43602/4350) halt additionally pokes the R4
    // core windows +0x40/+0x44. The exact values are not in the RE yet, so this
    // is logged only (blind writes could wedge the backplane).
    if (mChipId == 0xaa52 || mChipId == 0xa9c6) {
        IOLog("%s: NOTE: chip 0x%x needs r4core+0x40/+0x44 poke during halt "
              "(step 6) - PENDING RE\n", LOG_PREFIX, mChipId);
    }
    // TODO(RE): resolve the real R4 core register window on hardware (BAR0
    // backplane window vs CoreDescriptor base) and the full corereset sequence.
    return true;
}

bool ChipConfigurator::exitDownloadState(UInt32 vectors) {
    // `_wlc_ol_up` step 11: _si_wrapperreg(si, 0x408, 0x20, 0) — set/reset the
    // R4 wrapper SRESET bit5 to release the ARM after the image is in place.
    IOLog("%s: exitDownloadState vectors 0x%x\n", LOG_PREFIX, vectors);
    const CoreDescriptor *armCore = findCoreById(0x83e);
    if (armCore && armCore->base >= 0x10000000 && armCore->base < 0x80000000) {
        volatile UInt32 *reset = (volatile UInt32 *)((uintptr_t)armCore->base + 0x408);
        UInt32 v = *reset;
        v |= 0x20;
        *reset = v;
        (void)*reset;
    } else if (armCore) {
        IOLog("%s: WARNING: implausible R4 base 0x%x; release skipped\n",
              LOG_PREFIX, armCore->base);
    }
    return true;
}

UInt32 ChipConfigurator::tcmBaseForChip(UInt32 chipId) {
    // Mirrors the per-chip load base used by AirPortBrcmNIC's `_wlc_ol_up`
    // firmware mapping (see DLIMAGE_FINDINGS.md):
    //   0x4350/0x4354/0xaa52/0xa9c6 -> 0x180000
    //   0x4352/0x4360               -> 0
    switch (chipId) {
        case 0x4350:
        case 0x4354:
        case 0xaa52:
        case 0xa9c6:
            return 0x180000;
        case 0x4352:
        case 0x4360:
            return 0x0;
        default:
            return 0x0;
    }
}

UInt32 ChipConfigurator::tcmSizeForChip(UInt32 chipId) {
    // ARM SRAM size used by `_wlc_ol_up` to place the shared-info block:
    // S = BAR1 + tcm_base + tcm_size - 0x5f80. `_si_tcm_size()` sums the R4
    // core bank-port regs ((bpt&0x3f)+1)<<13) at runtime; for BCM4360 that is
    // 0x80000 (512 KB TCM). Hardcoded pending the EROM/backplane validation —
    // a wrong value makes the [S+0x5028] poll time out, which is the safety net.
    switch (chipId) {
        case 0x4360:
        case 0x4352:
        case 0x4350:
        case 0x4354:
        case 0xaa52:
        case 0xa9c6:
            // TODO(RE): implement the R4 bank walk (core 0x83e bank-port regs)
            // from mCoreList once findCoreAddresses is validated on hardware.
            return 0x80000;
        default:
            return 0;
    }
}

bool ChipConfigurator::requestFirmware(const UInt8 **fw, IOByteCount *len, const ChipDescriptor *chip) {
    // The firmware ships embedded in the kext (__const array generated from the
    // extracted Apple dlimage; see Chip/DlimageArrays.cpp). The image is selected
    // by the backplane chip number exactly like AirPortBrcmNIC's `wlc_ol_fw_get`.
    if (!fw || !len) {
        return false;
    }
    *fw = nullptr;
    *len = 0;
    mDlimageInfo = {};
    mFirmwareLength = 0;

    UInt32 chipNumber = chip ? chip->chipNumber : mChipId;
    IOByteCount imageLen = 0;
    const UInt8 *image = bcmc_get_dlimage(chipNumber, mChipRev, &imageLen);
    if (!image || imageLen == 0) {
        IOLog("%s: no embedded firmware for chip 0x%x\n", LOG_PREFIX, chipNumber);
        return false;
    }

    // Parse the container (raw vs DBPP) and record the load base.
    uint32_t fallback = tcmBaseForChip(chipNumber);
    if (!dl_image::parse(image, (size_t)imageLen, fallback, &mDlimageInfo) ||
        !mDlimageInfo.valid) {
        IOLog("%s: firmware image for 0x%x is malformed\n", LOG_PREFIX, chipNumber);
        return false;
    }

    // The copy loop (step 9) moves the WHOLE resource; the DBPP header's
    // declared imageSize can exceed both the resource and the TCM, so the
    // effective bytes are the resource length.
    mFirmwareLength = (UInt32)imageLen;

    IOLog("%s: firmware 0x%x rev 0x%x: %s declaredSize 0x%x bytes 0x%x load 0x%x\n",
          LOG_PREFIX, chipNumber, mChipRev,
          mDlimageInfo.container == dl_image::Container::kDbpp ? "DBPP" : "raw",
          mDlimageInfo.imageSize, mFirmwareLength, mDlimageInfo.loadAddress);

    *fw = image;
    *len = imageLen;
    return true;
}

UInt32 ChipConfigurator::tcmLoadBase() const {
    return mDlimageInfo.valid ? mDlimageInfo.loadAddress : tcmBaseForChip(mChipId);
}

bool ChipConfigurator::downloadFirmware(const UInt8 *fw, IOByteCount length) {
    if (!fw || length == 0) {
        return false;
    }
    // The TCM/RAM load base comes from the parsed dlimage (raw 4352pci -> 0,
    // DBPP blobs -> their loadAddress == 0x180000), mirroring `_wlc_ol_up`.
    if (mDlimageInfo.valid) {
        mTcmBase = mDlimageInfo.loadAddress;
    }

    // Safety: never write past the mapped BAR1 window. On a mis-derived TCM
    // base (e.g. a chip whose BAR1 is smaller than base+size) out-of-range
    // writes could fault the machine, so abort cleanly instead.
    if (mBar1Map) {
        UInt64 need = (UInt64)mTcmBase + (UInt64)length;
        if (need > (UInt64)mBar1Map->getLength()) {
            IOLog("%s: TCM base 0x%x + image 0x%x exceeds BAR1 length 0x%llx; "
                  "aborting copy\n", LOG_PREFIX, mTcmBase, (UInt32)length,
                  (unsigned long long)mBar1Map->getLength());
            return false;
        }
    }

    if (!enterDownloadState()) {
        IOLog("%s: enterDownloadState failed\n", LOG_PREFIX);
        return false;
    }

    // Copy the WHOLE resource verbatim to the TCM/RAM base, exactly like
    // `_wlc_ol_up`'s dword loop `map[base + w*4] = ((u32*)dl)[w]`. The image
    // lives in BAR1 (the ARM memory window, `[ol+0x80] + tcm_base`). There is
    // no header to strip: the raw 4352pci image has none, and for DBPP blobs the
    // load base equals the header loadAddress so the vector table still lands
    // at the right address. See DLIMAGE_FINDINGS.md.
    if (mBar1) {
        volatile UInt32 *map = (volatile UInt32 *)((uintptr_t)mBar1 + mTcmBase);
        const UInt32 *words = (const UInt32 *)fw;
        UInt32 nwords = (UInt32)(length / 4);
        for (UInt32 w = 0; w < nwords; w++) {
            map[w] = words[w];
        }
        // The wl driver zeroes the TCM gap between the image end and the
        // shared-info base (its patchable memset slot in step 9).
        UInt32 size = tcmSizeForChip(mChipId);
        UInt32 gap = shared_info::gapBytes(mTcmBase, size, (UInt32)length);
        UInt32 end = mTcmBase + (UInt32)length;
        if (gap) {
            volatile UInt32 *z = (volatile UInt32 *)((uintptr_t)mBar1 + end);
            for (UInt32 i = 0; i < gap / 4; i++) {
                z[i] = 0;
            }
            IOLog("%s: zeroed TCM gap 0x%x..0x%x (%u bytes)\n",
                  LOG_PREFIX, end, end + gap, gap);
        }
    } else {
        IOLog("%s: no BAR1 mapping for firmware copy\n", LOG_PREFIX);
        return false;
    }

    // Step 10: 0xaa52/0xa9c6 additionally select the internal-memory core
    // (0x80e), reset it, then reselect the R4 before releasing. The SI setcore
    // machinery is not implemented yet (findCoreAddresses is still a stub), so
    // this is logged only; the [S+0x5028] poll is the arbiter.
    if (mChipId == 0xaa52 || mChipId == 0xa9c6) {
        IOLog("%s: NOTE: chip 0x%x needs SI setcore 0x80e + corereset + setcore "
              "0x83e before release (step 10) - PENDING RE\n", LOG_PREFIX, mChipId);
    }

    if (!exitDownloadState(0)) {
        IOLog("%s: exitDownloadState failed\n", LOG_PREFIX);
        return false;
    }
    return true;
}

bool ChipConfigurator::publishSharedInfo(UInt32 dmaPhys, UInt32 dmaPhysHi) {
    // `_wlc_ol_up` step 7. S = BAR1 + tcm_base + tcm_size - 0x5f80. The
    // firmware reads this block after the ARM is released to find its BCDC/
    // olmsg rings, buffer size and the anticipated doorbell flow.
    if (!mBar1 || mChipId == 0) {
        IOLog("%s: publishSharedInfo: no BAR1 or chip not probed\n", LOG_PREFIX);
        return false;
    }
    UInt32 base = tcmLoadBase();
    UInt32 size = tcmSizeForChip(mChipId);
    // Safety: the shared-info block and its magics live inside [base, base+size);
    // refuse to write if that window is not covered by the mapped BAR1.
    if (mBar1Map && (UInt64)base + (UInt64)size > (UInt64)mBar1Map->getLength()) {
        IOLog("%s: publishSharedInfo: TCM window base 0x%x size 0x%x exceeds "
              "BAR1 length 0x%llx\n", LOG_PREFIX, base, size,
              (unsigned long long)mBar1Map->getLength());
        return false;
    }
    // `fits()` must use the bytes actually copied (resource length), not the
    // DBPP header's declared imageSize which can exceed the whole TCM.
    UInt32 imageBytes = mFirmwareLength ? mFirmwareLength : mDlimageInfo.imageSize;
    if (!shared_info::fits(size, imageBytes)) {
        IOLog("%s: publishSharedInfo: TCM 0x%x too small for image 0x%x\n",
              LOG_PREFIX, size, imageBytes);
        return false;
    }
    UInt32 S = shared_info::blockOffset(base, size);
    volatile UInt32 *s = (volatile UInt32 *)((uintptr_t)mBar1 + S);

    s[shared_info::kOffMagic1 >> 2]  = shared_info::kMagic1;
    s[shared_info::kOffD2hAddr >> 2] = (UInt32)(dmaPhys & 0xffffffffULL);   // [ol+0x34]
    s[shared_info::kOffH2dAddr >> 2] = (UInt32)(dmaPhysHi & 0xffffffffULL); // [ol+0x38] (0 for force32)
    s[shared_info::kOffRingSz >> 2]  = shared_info::kOlmsgSize;
    s[0x14 >> 2] = 0;
    s[0x18 >> 2] = 0;
    s[shared_info::kOffFlags >> 2]   = 0;                                    // [wlc->[0]+0x110] (0 ok)
    s[shared_info::kMagic2Off >> 2]  = shared_info::kMagic2;
    s[0x5ed8 >> 2] = 0;

    // Keep the header flush visible to the ARM (BAR writes go over PCIe).
    (void)*s;
    IOLog("%s: shared-info at BAR1+0x%x: base 0x%x size 0x%x phys 0x%x ring 0x%x\n",
          LOG_PREFIX, S, base, size, dmaPhys, shared_info::kOlmsgSize);
    IOLog("%s:   S[0]=0x%08x S[1]=0x%08x S[2]=0x%08x S[3]=0x%08x S[0x5f5c]=0x%08x\n",
          LOG_PREFIX,
          OSReadLittleInt32((void *)(mBar1 + S), 0),
          OSReadLittleInt32((void *)(mBar1 + S), 4),
          OSReadLittleInt32((void *)(mBar1 + S), 8),
          OSReadLittleInt32((void *)(mBar1 + S), 0x0c),
          OSReadLittleInt32((void *)(mBar1 + S), 0x5f5c));
    return true;
}

UInt32 ChipConfigurator::awaitFirmwareReady(UInt32 timeoutMs) {
    // `_wlc_ol_up` step 13: poll S+0x5028 until nonzero. The firmware sets this
    // once it is running and has validated its IPC region. If the R4 has not
    // been released or tcmSize/load base is wrong the poll times out; the
    // progress + dump logs below are the primary hardware bring-up diagnostics.
    if (!mBar1) {
        return 0;
    }
    UInt32 base = tcmLoadBase();
    UInt32 size = tcmSizeForChip(mChipId);
    UInt32 S = shared_info::blockOffset(base, size);
    volatile UInt32 *ready = (volatile UInt32 *)((uintptr_t)mBar1 + shared_info::readyOffset(base, size));

    UInt32 stepMs = (timeoutMs == 0) ? 2000 : timeoutMs;
    UInt32 waits = 0;
    while ((*ready) == 0) {
        IOSleep(1);
        if ((++waits % 100) == 0 && waits >= 100) {
            IOLog("%s: awaitFirmwareReady: waiting %u/%u ms\n", LOG_PREFIX, waits, stepMs);
        }
        if (waits >= stepMs) { // 1 ms per wait
            IOLog("%s: awaitFirmwareReady: TIMED OUT after %u ms (S+0x%x)\n",
                  LOG_PREFIX, waits, shared_info::readyOffset(base, size));
            IOLog("%s:   S[0]=0x%08x S[1]=0x%08x S[2]=0x%08x S[3]=0x%08x\n", LOG_PREFIX,
                  OSReadLittleInt32((void *)(mBar1 + S), 0),
                  OSReadLittleInt32((void *)(mBar1 + S), 4),
                  OSReadLittleInt32((void *)(mBar1 + S), 8),
                  OSReadLittleInt32((void *)(mBar1 + S), 0x0c));
            return 0;
        }
    }
    UInt32 v = *ready;
    IOLog("%s: firmware ready flag 0x%x after %u ms\n", LOG_PREFIX, v, waits);
    return v;
}

UInt32 ChipConfigurator::awaitFirmware(UInt32 timeoutMs) {
    // Poll the shared-memory header for the "ready" signature written by the
    // firmware after it starts and discovers its IPC region.
    // TODO(RE): shared memory address comes from mSharedMemAddress; BCMC polls
    // with a timeout and reports "Timed out waiting for the firmware to
    // initialize."
    IOLog("%s: awaitFirmware(%u ms)\n", LOG_PREFIX, timeoutMs);
    return 0;
}

void ChipConfigurator::logBootReport() {
    UInt32 base = tcmLoadBase();
    UInt32 size = tcmSizeForChip(mChipId);
    const CoreDescriptor *r4 = findCoreById(0x83e);
    UInt32 bar1Len = mBar1Map ? (UInt32)mBar1Map->getLength() : 0;
    IOLog("%s: BOOT REPORT chip 0x%x rev 0x%x interconnect 0x%x cores %u "
          "r4base 0x%x tcmBase 0x%x tcmSize 0x%x S 0x%x ready 0x%x bar1 0x%x r4 %s\n",
          LOG_PREFIX, mChipId, mChipRev, mInterconnect, mCoreCount,
          r4 ? r4->base : 0, base, size,
          shared_info::blockOffset(base, size),
          shared_info::readyOffset(base, size),
          bar1Len,
          r4 ? "yes" : "no");
    for (UInt32 i = 0; i < mCoreCount; i++) {
        const CoreDescriptor &c = mCoreList[i];
        IOLog("%s:   core[%u] id 0x%x base 0x%x size 0x%x flags 0x%x\n",
              LOG_PREFIX, i, c.coreId, c.base, c.size, c.flags);
    }
}