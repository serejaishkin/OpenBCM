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
2. Join: нет `wsec` / `wpa_auth` / `sup_wpa` / `WLC_SET_WSEC_PMK` — без них только open-сети.
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
