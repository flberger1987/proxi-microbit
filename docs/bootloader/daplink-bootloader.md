# Bootloader-Architektur

## Übersicht

Das ProxiMicro-Projekt verwendet ein **eigenes Boot-Pointer OTA System** anstelle von MCUboot. Dies ermöglicht:
- Kabellose Firmware-Updates via BLE
- Sofortiges Slot-Switching ohne Kopieren
- Automatischer Fallback bei Boot-Fehlern
- Persistente Settings (Kalibrierung, PID, BLE-Bonds)

---

## DAPLink (Interface-Chip)

Der micro:bit hat einen separaten Interface-Chip mit DAPLink-Firmware:

```
┌──────────────────────────────────────────────────────────────┐
│                    Interface MCU                              │
│                  (nRF52833 oder nRF52820)                     │
│                                                               │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐   │
│  │ Mass Storage│  │  CMSIS-DAP  │  │   CDC Serial        │   │
│  │  (MICROBIT) │  │  (Debug)    │  │   (Console)         │   │
│  └─────────────┘  └─────────────┘  └─────────────────────┘   │
└──────────────────────────────────────────────────────────────┘
```

**Funktionen:**
- **Drag & Drop**: `.hex` auf MICROBIT-Laufwerk kopieren
- **CMSIS-DAP**: Debugging via pyOCD/OpenOCD
- **CDC Serial**: USB-Konsole (115200 Baud)

---

## ProxiMicro Boot-Pointer System

### Flash-Layout (512KB nRF52833)

```
0x00000 ┌─────────────────────┐
        │ Mini-Bootloader     │  8 KB
        │ (Boot-Pointer)      │
0x02000 ├─────────────────────┤
        │                     │
        │ Slot A (Primary)    │  244 KB
        │                     │
0x3F000 ├─────────────────────┤
        │                     │
        │ Slot B (Secondary)  │  244 KB
        │                     │
0x7C000 ├─────────────────────┤
        │ Boot Control Block  │  4 KB
0x7D000 ├─────────────────────┤
        │ OTA State           │  4 KB
0x7E000 ├─────────────────────┤
        │ Settings (NVS)      │  8 KB
        │ - Mag Kalibrierung  │
        │ - PID Parameter     │
        │ - BLE Bonds         │
0x80000 └─────────────────────┘
```

### Boot-Entscheidung

Der Mini-Bootloader (8KB) führt beim Start folgende Prüfungen durch:

```
1. Boot Control Block lesen & validieren
2. CRC32 beider Slots prüfen
3. Slot wählen:
   ├─ Beide gültig → Höhere Build-Nummer
   ├─ Nur einer gültig → Diesen verwenden
   └─ Keiner gültig → Fallback zu Slot A
4. boot_count prüfen:
   └─ ≥3 → Auf anderen Slot wechseln (wenn gültig)
5. boot_count inkrementieren
6. Vector Table validieren
7. Zu Firmware springen
```

### Boot Control Block

```c
struct boot_control {
    uint32_t magic;           /* 0xB007F1E0 */
    uint8_t  version;         /* Struktur-Version (2) */
    uint8_t  active_slot;     /* 0 = Slot A, 1 = Slot B */
    uint8_t  slot_a_valid;    /* CRC OK */
    uint8_t  slot_b_valid;    /* CRC OK */
    uint8_t  boot_count;      /* Fallback-Zähler (≥3 → Rollback) */
    uint8_t  fallback_reason; /* Diagnostik */
    uint32_t slot_a_crc;      /* CRC32 Slot A */
    uint32_t slot_b_crc;      /* CRC32 Slot B */
    uint32_t slot_a_size;     /* Firmware-Größe */
    uint32_t slot_b_size;
    uint32_t slot_a_version;  /* Build-Nummer */
    uint32_t slot_b_version;
    uint32_t checksum;        /* Block-CRC32 */
};
```

### Fallback-Mechanismus

| boot_count | Aktion |
|------------|--------|
| 0 | Normaler Boot, Firmware ruft `confirm_boot()` |
| 1-2 | Boot-Versuch, wartet auf `confirm_boot()` |
| ≥3 | **Automatischer Rollback** zum anderen Slot |

**Erfolgreicher Boot:** Firmware ruft `boot_control_confirm_boot()` → setzt boot_count auf 0.

---

## BLE OTA Update

### Protokoll

Updates erfolgen über **Nordic UART Service (NUS)** mit eigenem Paketformat.

```
┌────────┬─────────┬──────────────────────────┐
│ Magic  │ Command │ Payload                  │
│ 0xF0   │ 1 Byte  │ variabel                 │
└────────┴─────────┴──────────────────────────┘
```

### OTA State Machine

```
IDLE ──► ERASE ──► TRANSFER ──► VALIDATE ──► COMMIT ──► REBOOT
                       │              │
                       └── SUSPENDED ─┘ (bei Disconnect)
```

### Update-Ablauf

1. **INIT**: Host sendet Firmware-Größe + CRC32 + Version
2. **ERASE**: Ziel-Slot page-weise löschen
3. **TRANSFER**: 224-Byte Blöcke mit CRC16
4. **VALIDATE**: Gesamte Firmware CRC32 prüfen
5. **COMMIT**: Boot-Pointer umschalten
6. **REBOOT**: Neustart in neue Firmware

### Resume-Fähigkeit

Bei Verbindungsabbruch:
1. Transfer wechselt zu SUSPENDED
2. Host reconnectet, sendet QUERY
3. Device antwortet mit letztem Block
4. Host setzt Transfer fort

---

## Flash-Methoden

### Initial (einmalig)

```bash
./build_flash.sh flash   # Bootloader + App + Settings löschen
```

### Updates (empfohlen)

```bash
./build_flash.sh ota     # Via BLE, behält alles
```

### USB ohne Settings-Verlust

```bash
./build_flash.sh flash-app   # Nur App, Settings bleiben
```

---

## Maintenance Mode (DAPLink)

Falls der Bootloader beschädigt ist:

1. **RESET-Taste gedrückt halten** während USB anschließen
2. Laufwerk erscheint als **MAINTENANCE**
3. DAPLink-Firmware via Drag & Drop reparieren

---

## Referenzen

| Ressource | URL |
|-----------|-----|
| DAPLink Interface | https://tech.microbit.org/software/daplink-interface/ |
| micro:bit Tech Site | https://tech.microbit.org/ |
| pyOCD | https://pyocd.io/ |
