# OpenBCM: softmac + fullmac в одном kext

## Модель
`ChipType` — свойство **персоналии** (Info.plist), а не железа: один и тот же PCI id
может работать с Apple dlimage (softmac, host-driven D11) или с fullmac-прошивкой.
Приоритет: plist `ChipType` → таблица `kBcmChipTable` (`ChipDescriptor::applyOverride`,
флаги `chipTypeSet` / `needsCortexR4Set`). Дескриптор живёт в `ItlBcm::mDescriptor`.

| | softmac | fullmac |
|---|---|---|
| MAC/PHY | хост (D11 dma64 rings) | прошивка |
| enable() | RX/TX кольца + RX poll + WLC poll | только WLC D2H poll |
| scan | WLC escan (`BCM_HOST_SCAN` — host-скан) | WLC escan |
| join | net80211 AUTH/ASSOC через TX-кольцо | AUTH → `bcm_wlc_join`, RUN по `E_PSK_SUP(6)`/`E_LINK` |
| TX данных | D11 TX ring | **TODO** (сейчас дроп + `if_oerrors`) |

## Что дальше (порядок)
1. Сборка на macOS (в этой правке не проверялась — нет Xcode/MacKernelSDK).
2. Join (сделано, не проверено на железе): `bcm_wlc_join` шлёт `wpa_auth`/`auth`/`wsec`/`sup_wpa`, `WLC_SET_WSEC_PMK` (PSK из `ic_psk`) и bsscfg `join` (ext_join_params, раскладка с natural alignment); `bcm_wlc_join_legacy` (WLC_SET_SSID) — запасной путь. WPA3/SAE и 802.1X не поддержаны.
3. D2H события: `E_LINK` парсится как голый u32, реальные события несут заголовок
   `brcmf_event_msg`; сверить на железе (`ver`/`cur_etheraddr` через olmsg).
4. Данные fullmac: для Apple dlimage канала данных нет (D11 RE заблокирован, см. RE_FINDINGS.md).
   Для чипов с прошивкой Linux (43602, 4350, 4356, 4358, 4359, 4365/66 — имена файлов сверить с
   linux-firmware) реализовать `MsgbufTransport` по brcmfmac `pcie.c`/`msgbuf.c`:
   shared RAM, ctrl submit/complete ring, rxpost, flowring, ioctl и events через ctrl-кольца.
   BCM4360 (43a0) в brcmfmac не поддерживается → только softmac.
5. Для fullmac-прошивок нужны также board nvram (`.txt`) и при необходимости `clm_blob`.
6. `kBcmChipTable`: записи 0x4352/0x4357/0x4358/0x4359/0x4365/0x4356 — это, похоже, номера чипов,
   а не PCI device id; сверить, прежде чем на них опираться.

## Готово без железа (host-тесты: `scripts/run_host_tests.sh`)
- `Chanspec.hpp` — d11n/d11ac (4360/43602/4350 = AC).
- `WlcCodec.hpp` — escan, ext join (72 B), bsscfg-iovar, WSEC_PMK (132 B), bss_info v109, escan_result, event_msg, синтетический beacon.
- `Msgbuf.hpp` — структуры msgbuf, commonring, shared RAM, план колец/индексов.
- `Nvram.hpp` — nvram txt -> blob + токен длины.

## Осталось для `MsgbufTransport` (нужно железо / IOKit)
1. BAR0/BAR2 окна, сброс ARM-ядра, загрузка fw в TCM, nvram в конец RAM, поиск shared-адреса.
2. DMA-буферы колец и host-индексов, doorbell и маска прерываний PCIe-ядра.
3. ioctl/event/rxpost циклы поверх `Msgbuf.hpp`, flowring create, tx/rx в net80211.

## Update: transport glue written (untested on hardware)
- `Erom.hpp` — EROM/DMP scan (host-tested).
- `FullmacPcie.hpp` — IOKit glue: BAR0 regs + BAR2 TCM + cfg window, DMA via IOBufferMemoryDescriptor,
  bring-up → fw/nvram download → shared RAM → common rings (TCM indices) → msgbuf::Engine, polled
  mailbox, synchronous `ioctl()`. Syntax-checked against stub headers only.
- Still to do: hook `Pcie::start()` into ItlBcm fullmac attach (fw/nvram loading, 43ba personality),
  replace WLC calls with `Pcie::ioctl` (WLC_GET/SET_VAR = 262/263), then flowrings + rxpost data path.

## Update: ItlBcm integration
- `ItlBcm::attach` branches on `isFullmac()` → `fullmac_attach()` (loads `FirmwareFile` + same-name `.txt`
  nvram via the bundle/filesystem loader, runs `fullmac::Pcie::start`), then `cur_etheraddr` → `ic_myaddr`.
- `bcm_wlc_send_ioctl/iovar_set` and `bcm_wlc_poll_d2h` route to msgbuf when `mFm` is set; events go
  through the shared `bcm_wlc_handle_event`.
- The `BCM43602` personality is fullmac by default. OpenCore DeviceProperties `openbcm-mode`,
  `openbcm-fw`, `openbcm-nvram` select mode/firmware/nvram per card (see firmware/fullmac/README.md).
- OpenCore workflow: firmware is embedded in the kext binary. Run
  `scripts/gen_fullmac_arrays.py` (embeds firmware/fullmac/*.bin + *.txt), rebuild, inject the kext via
  config.plist as usual. No files are read from disk (disk lookup is only a fallback).
- Caveat: `Pcie::ioctl` polls synchronously and may deliver events (escan results) re-entrantly from within a send.
- Data path done (untested on HW): `MsgbufData.hpp` (flowring per ifidx+AC, TX copy into DMA slots, rxpost, rx complete)
  wired into `ItlBcm` (`bcm_tx_start` → `Pcie::transmit`, RX → `if_input` as 802.3). Polled every 2 ms for now.
- Still to do: MSI/MSIX interrupt instead of polling, host DMA indices, CLM blob upload (`clmload`) for 4356/43570,
  key install via firmware (WSEC_PMK path already used), power save / roaming / bgscan tuning, monitor mode.
