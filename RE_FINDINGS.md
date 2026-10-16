# OpenBCM — результаты реверса по REVERSE_ENGINEERING.md

Проверено по исходникам Linux (torvalds/linux master): `brcmfmac/{fwil.h,fwil_types.h,fweh.h,cfg80211.c}`, `include/brcmu_wifi.h`, `brcmsmac/phy/phy_cmn.c`.

## 1. Главное: пункт 2.1 из документа (регистры PHY для смены канала) через brcmfmac не выполним

- `brcmfmac` — FullMAC-драйвер. Хост **никогда** не пишет регистры D11/PHY и не переключает канал. Всё делает прошивка; хост шлёт только команды (ioctl/iovar).
- Файлы и символы из §1.1 не существуют в brcmfmac: `brcmfmac_main.c`, `brcm_wifi.c`, `d11.c`, `dma.c`, `phy.c`, `phy_shim.c`, `brcmf_wlan.h`, `chip_common.c`, `_wlc_bmac_set_channel`, `_wlc_phy_channel`, `wlc_ol_dpc`, `_wlc_ol_*`. Реальные: `cfg80211.c`, `fwil.h`, `fwil_types.h`, `fweh.h`, `chip.c`, `core.c`.
- Код PHY есть только в **brcmsmac** (SoftMAC) и только для N-PHY и LCN-PHY (`wlc_phy_attach` отвергает остальные типы). BCM4360/43602 — AC-PHY, там его нет.
- Значения `0x44c / 0x44e / 0x4e / D11_PHY_BAND_*` в документе ничем не подтверждены. В `phy_cmn.c` `0x44c` — регистр **LCN-PHY**, используется в `wlc_phy_switch_radio` (вкл/выкл радио), не канал. Не использовать как `kD11PhyChanReg`.
- Что реально подтверждено про механизм доступа к PHY (brcmsmac, corerev ≥ 24):
  - PHY reg write: 32-бит запись `addr | (val << 16)` в `D11REGOFFS(phyregaddr)`.
  - PHY reg read: запись `addr` в `phyregaddr`, чтение `phyregdata`.
  - Radio reg: `radioregaddr` / `radioregdata`.
  - Смена канала в brcmsmac: `wlc_phy_chanspec_set` пишет SHM `M_CURCHANNEL` (канал | 5G-флаг | 40MHz-флаг) и вызывает PHY-специфичный `pi_fptr.chanset`. Для AC-PHY такого кода в открытых исходниках нет.
  - Смещения `phyregaddr/phyregdata` в `d11.h` здесь не проверялись.
- Вывод: **ChipConfigurator::setChannel() с записью в PHY-регистры не реализуем из открытых источников**. Единственный путь — оставить смену канала прошивке (п. 2 ниже) либо дизассемблировать AirPortBrcmNIC/dlimage.

## 2. Рабочий путь: скан и join силами прошивки (раздел 2.2 документа)

Все номера подтверждены, см. `WlcDefs.hpp`. Ключевое:

| Что | Значение |
|---|---|
| iovar get / set | `WLC_GET_VAR=262` / `WLC_SET_VAR=263` (`"name\0"` + data) |
| Скан | iovar `"escan"`: `[u32 version][u16 action=1][u16 sync_id=0x1234][scan_params]` |
| scan_params | fixed 64 B (v1) или 72 B (v2); `channel_num=(n_ssids<<16)\|n_ch`; каналы — массив **chanspec u16** |
| Abort скана | `WLC_SCAN=50`, `channel_list[0]=-1`, `channel_num=1` |
| Результаты | событие `ESCAN_RESULT=69 (0x45)`; `status=PARTIAL(8)` — один BSS (`bss_count=1`), иначе скан завершён (`SUCCESS=0`, `ABORT=4`) |
| Старый путь | `WLC_SCAN=50` + `WLC_SCAN_RESULTS=51`, событие `SCAN_COMPLETE=26` |
| Join | bsscfg iovar `"join"` (ext_join_params), fallback `WLC_SET_SSID=26` |
| Security | bsscfg `wpa_auth`, `auth`, `wsec`; `sup_wpa=1`; `WLC_SET_WSEC_PMK=268` (PSK 32 B) — 4-way handshake делает прошивка (`PSK_SUP=46`, status 6 = готово) |
| Chanspec 20 MHz (d11n) | `ch \| 0x0800 \| 0x0300 \| (2G:0x2000 / 5G:0x1000)` → ch1=`0x2B01`, ch6=`0x2B06`, ch36=`0x1B24` |
| `WLC_SET_CHANNEL=30` | принимает номер канала (u32), не регистр |

Оговорки:

1. Для 43602/4360 (d11 **ac**) brcmfmac использует другую раскладку chanspec (`brcmu_d11ac_encchspec` в `brcmu_d11.c`; не загружалась). Перед использованием сверить.
2. Это словарь **Linux**-прошивки (dcmd/BCDC). Документ сам отмечает, что dlimage Apple работает по olmsg. Совпадут ли ID сообщений olmsg с WLC-ioctl — гипотеза, проверяется на железе (отправить `WLC_GET_VAR "ver"`/`"cur_etheraddr"` через olmsg и посмотреть ответ).
3. Список D2H-событий из документа (`0x4,0x10,0x11,0x12,0x15,0x41`): если это Linux-коды событий, ни один не scan-done (`0x1A`/`0x45`). Скорее всего это другое пространство ID (olmsg).

## 3. Что делать дальше (по порядку)

1. На железе через существующий olmsg-транспорт послать минимальный iovar `"ver"` (WLC_GET_VAR=262). Ответ → подтверждает, что Linux-словарь применим.
2. Если применим: реализовать `escan` с chanspec-списком, вместо ручной смены канала в `ItlBcm::bcm_scan_next_channel()` — один запрос на весь `sc_scan_list`, результаты из события 69.
3. Если не применим: нужен `AirPortBrcmNIC` (macOS 10.13–11.7) — искать диспетчер ioctl/olmsg и сопоставление ID; brcmfmac тут не поможет.
4. RSSI из RX-заголовка (пункт чек-листа): brcmsmac (`wlc_phy_rssi_compute`) читает `PhyRxStatus_1 & PRXS1_JSSI_MASK` — это формат N/LCN; для AC другой. Источник для 4360 — `brcmfmac` его не видит (прошивка отдаёт RSSI в `bss_info.RSSI`/event), поэтому для RX-пути брать RSSI из ответа прошивки, а не из rxh.

## 4. Правки для REVERSE_ENGINEERING.md

- §1.1: заменить список файлов/символов на реальные (см. §1 выше); убрать `_wlc_*` из «ключевых символов brcmfmac» — они только в прошивке/Apple-кексте.
- §2.1: пометить как «невыполнимо через brcmfmac»; значения `0x44c/0x4e` удалить.
- §4: убрать пример `setChannel()` с записью в `0x44c`.
- Чек-лист §5: пункты про channel reg и band bit — снять; добавить «подтвердить применимость WLC-словаря на dlimage».
