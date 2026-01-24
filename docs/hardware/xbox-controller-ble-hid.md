# Xbox Wireless Controller BLE HID Protocol

**Dokumentiert:** 2026-01-24
**Controller:** Xbox Wireless Controller (Model 1914)
**Verbindung:** Bluetooth Low Energy (BLE)

## Übersicht

Der Xbox Wireless Controller sendet HID Reports über BLE mit 16 Bytes pro Report. Die Daten werden ca. 8-10x pro Sekunde gesendet.

## BLE Service Information

| Parameter | Wert |
|-----------|------|
| Service | HID Service (0x1812) |
| Report Characteristic | Handle 0x001e |
| Report Length | 16 Bytes |
| Update Rate | ~120ms (~8 Hz) |

## HID Report Format (16 Bytes)

```
Byte:  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
      └──┴──┘└──┴──┘└──┴──┘└──┴──┘└──┴──┘└──┴──┘ │  │  │  │
       LX     LY     RX     RY     LT     RT    DP B1 B2 --
```

### Detaillierte Byte-Beschreibung

| Bytes | Name | Typ | Beschreibung |
|-------|------|-----|--------------|
| 0-1 | Left Stick X | uint16 LE | Links/Rechts, Center=32768 |
| 2-3 | Left Stick Y | uint16 LE | Oben/Unten, Center=32768, **invertiert** |
| 4-5 | Right Stick X | uint16 LE | Links/Rechts, Center=32768 |
| 6-7 | Right Stick Y | uint16 LE | Oben/Unten, Center=32768, **invertiert** |
| 8-9 | Left Trigger | uint16 LE | 0-1023 (10-bit), nur untere 10 Bits gültig |
| 10-11 | Right Trigger | uint16 LE | 0-1023 (10-bit), nur untere 10 Bits gültig |
| 12 | D-Pad | uint8 | Hat Switch (siehe unten) |
| 13 | Buttons Low | uint8 | Face Buttons + Bumpers |
| 14 | Buttons High | uint8 | System Buttons + Stick Clicks |
| 15 | Reserved | uint8 | Unbenutzt (immer 0x00) |

## Analog Sticks

### Koordinatensystem

```
                Up (Y = 0)
                    ↑
                    │
   Left (X = 0) ←───┼───→ Right (X = 65535)
                    │
                    ↓
              Down (Y = 65535)
```

### Wertebereiche

| Wert | Raw | Signed (center=32768) |
|------|-----|----------------------|
| Minimum | 0 | -32768 |
| Center | 32768 | 0 |
| Maximum | 65535 | +32767 |

### Y-Achsen Invertierung

**WICHTIG:** Die Y-Achse ist invertiert gegenüber der erwarteten Konvention:
- Stick nach **OBEN** drücken → Y-Wert wird **kleiner** (→ -32768)
- Stick nach **UNTEN** ziehen → Y-Wert wird **größer** (→ +32767)

```c
// Konvertierung zu intuitivem Format (oben = positiv):
int16_t forward = -left_stick_y;  // Invertieren für "oben = vorwärts"
```

### Stick Drift

Die Sticks haben typischerweise Drift von ±500-2000 um den Centerwert. Eine Deadzone von ~3000-4000 wird empfohlen.

```c
#define DEADZONE 3277  // ~10% des Bereichs

int16_t apply_deadzone(int16_t value) {
    if (value > DEADZONE) return value - DEADZONE;
    if (value < -DEADZONE) return value + DEADZONE;
    return 0;
}
```

## Trigger (LT/RT)

Die Trigger sind 10-bit Analog-Werte (0-1023).

| Zustand | Wert |
|---------|------|
| Nicht gedrückt | 0 |
| Voll gedrückt | 1023 |

### Trigger Parsing

```c
// Bytes 8-9: Left Trigger (10-bit)
uint16_t lt = (data[8] | (data[9] << 8)) & 0x03FF;

// Bytes 10-11: Right Trigger (10-bit)
uint16_t rt = (data[10] | (data[11] << 8)) & 0x03FF;
```

## D-Pad (Hat Switch)

Byte 12 enthält den D-Pad Zustand als Hat Switch mit 8 Richtungen + Neutral:

| Wert | Richtung | Bits |
|------|----------|------|
| 0x00 | Neutral (keine Taste) | - |
| 0x01 | Up | ↑ |
| 0x02 | Up-Right | ↗ |
| 0x03 | Right | → |
| 0x04 | Down-Right | ↘ |
| 0x05 | Down | ↓ |
| 0x06 | Down-Left | ↙ |
| 0x07 | Left | ← |
| 0x08 | Up-Left | ↖ |

### D-Pad Diagramm

```
        0x01 (Up)
    0x08 ↖ ↑ ↗ 0x02
          \|/
   0x07 ←──●──→ 0x03
          /|\
    0x06 ↙ ↓ ↘ 0x04
        0x05 (Down)

   Center = 0x00
```

## Buttons

### Byte 13: Face Buttons & Bumpers

| Bit | Maske | Button |
|-----|-------|--------|
| 0 | 0x01 | A (grün) |
| 1 | 0x02 | B (rot) |
| 2 | 0x04 | - (unbenutzt) |
| 3 | 0x08 | X (blau) |
| 4 | 0x10 | Y (gelb) |
| 5 | 0x20 | - (unbenutzt) |
| 6 | 0x40 | LB (Left Bumper) |
| 7 | 0x80 | RB (Right Bumper) |

### Byte 14: System Buttons & Stick Clicks

| Bit | Maske | Button |
|-----|-------|--------|
| 0 | 0x01 | - (unbenutzt) |
| 1 | 0x02 | - (unbenutzt) |
| 2 | 0x04 | View (⧉) |
| 3 | 0x08 | Menu (☰) |
| 4 | 0x10 | Xbox (Ⓧ) |
| 5 | 0x20 | L3 (Left Stick Click) |
| 6 | 0x40 | R3 (Right Stick Click) |
| 7 | 0x80 | - (unbenutzt) |

### Button Layout Referenz

```
        ┌─────────────────────────────────┐
        │     [LB]             [RB]       │
        │      ___               ___      │
        │     /   \             /   \     │
        │    ( LT  )           ( RT  )    │  LT/RT = Trigger (analog)
        │     \___/             \___/     │
        │                                 │
        │   ┌───┐                 [Y]     │
        │   │ ↑ │               [X] [B]   │
        │ ┌─┼───┼─┐               [A]     │
        │ │←│   │→│                       │
        │ └─┼───┼─┘    [View] Ⓧ [Menu]   │
        │   │ ↓ │                         │
        │   └───┘     ○       ○           │
        │   D-Pad    (L3)    (R3)         │
        │          Left    Right          │
        │          Stick   Stick          │
        └─────────────────────────────────┘
```

## Beispiel: Raw HID Report

```
Ruhezustand (keine Eingaben, leichter Stick-Drift):
91 7e 57 80 60 82 50 85 00 00 00 00 00 00 00 00
│  │  │  │  │  │  │  │  │  │  │  │  │  │  │  └─ Reserved
│  │  │  │  │  │  │  │  │  │  │  │  │  │  └──── Buttons High
│  │  │  │  │  │  │  │  │  │  │  │  │  └─────── Buttons Low
│  │  │  │  │  │  │  │  │  │  │  │  └────────── D-Pad
│  │  │  │  │  │  │  │  │  │  └──┴───────────── Right Trigger
│  │  │  │  │  │  │  │  └──┴─────────────────── Left Trigger
│  │  │  │  │  │  └──┴───────────────────────── Right Stick Y
│  │  │  │  └──┴─────────────────────────────── Right Stick X
│  │  └──┴───────────────────────────────────── Left Stick Y
└──┴─────────────────────────────────────────── Left Stick X

Parsing:
- LX: 0x7E91 = 32401 → signed: 32401 - 32768 = -367
- LY: 0x8057 = 32855 → signed: 32855 - 32768 = +87
- RX: 0x8260 = 33376 → signed: +608
- RY: 0x8550 = 34128 → signed: +1360
- LT: 0x0000 & 0x3FF = 0
- RT: 0x0000 & 0x3FF = 0
- D-Pad: 0x00 = Neutral
- Buttons: 0x0000 = keine
```

## C-Code Referenz

### Strukturen

```c
/* Button Masken */
#define XBOX_BTN_A          0x0001
#define XBOX_BTN_B          0x0002
#define XBOX_BTN_X          0x0004
#define XBOX_BTN_Y          0x0008
#define XBOX_BTN_LB         0x0010
#define XBOX_BTN_RB         0x0020
#define XBOX_BTN_VIEW       0x0040
#define XBOX_BTN_MENU       0x0080
#define XBOX_BTN_LSTICK     0x0100
#define XBOX_BTN_RSTICK     0x0200
#define XBOX_BTN_XBOX       0x0400

/* D-Pad Werte */
#define XBOX_DPAD_NONE      0x00
#define XBOX_DPAD_UP        0x01
#define XBOX_DPAD_UP_RIGHT  0x02
#define XBOX_DPAD_RIGHT     0x03
#define XBOX_DPAD_DOWN_RIGHT 0x04
#define XBOX_DPAD_DOWN      0x05
#define XBOX_DPAD_DOWN_LEFT 0x06
#define XBOX_DPAD_LEFT      0x07
#define XBOX_DPAD_UP_LEFT   0x08

/* Input Struktur */
struct xbox_input {
    int16_t left_stick_x;   /* -32768 to 32767 */
    int16_t left_stick_y;   /* -32768 to 32767 (invertiert!) */
    int16_t right_stick_x;
    int16_t right_stick_y;
    uint16_t left_trigger;  /* 0 to 1023 */
    uint16_t right_trigger;
    uint8_t dpad;           /* Hat switch 0-8 */
    uint16_t buttons;       /* Bitmask */
};
```

### Parser Funktion

```c
int parse_xbox_hid(const uint8_t *data, uint16_t len, struct xbox_input *out)
{
    if (len < 14) return -1;

    /* Sticks (16-bit unsigned → signed) */
    out->left_stick_x  = (int16_t)(data[0] | (data[1] << 8)) - 32768;
    out->left_stick_y  = (int16_t)(data[2] | (data[3] << 8)) - 32768;
    out->right_stick_x = (int16_t)(data[4] | (data[5] << 8)) - 32768;
    out->right_stick_y = (int16_t)(data[6] | (data[7] << 8)) - 32768;

    /* Trigger (10-bit) */
    out->left_trigger  = (data[8] | (data[9] << 8)) & 0x03FF;
    out->right_trigger = (data[10] | (data[11] << 8)) & 0x03FF;

    /* D-Pad */
    out->dpad = data[12];

    /* Buttons - Remap von Raw zu logischen Masken */
    uint8_t btn_lo = data[13];
    uint8_t btn_hi = data[14];

    uint16_t buttons = 0;
    if (btn_lo & 0x01) buttons |= XBOX_BTN_A;
    if (btn_lo & 0x02) buttons |= XBOX_BTN_B;
    if (btn_lo & 0x08) buttons |= XBOX_BTN_X;
    if (btn_lo & 0x10) buttons |= XBOX_BTN_Y;
    if (btn_lo & 0x40) buttons |= XBOX_BTN_LB;
    if (btn_lo & 0x80) buttons |= XBOX_BTN_RB;
    if (btn_hi & 0x04) buttons |= XBOX_BTN_VIEW;
    if (btn_hi & 0x08) buttons |= XBOX_BTN_MENU;
    if (btn_hi & 0x10) buttons |= XBOX_BTN_XBOX;
    if (btn_hi & 0x20) buttons |= XBOX_BTN_LSTICK;
    if (btn_hi & 0x40) buttons |= XBOX_BTN_RSTICK;
    out->buttons = buttons;

    return 0;
}
```

## Bekannte Eigenheiten

1. **Stick Drift**: Alle Xbox Controller haben leichten Stick-Drift. Deadzone von 3000-4000 empfohlen.

2. **Y-Achsen Invertierung**: Oben drücken = negativer Wert. Für intuitive Steuerung invertieren.

3. **Button-Bit Layout**: Die Raw-Bits entsprechen NICHT direkt den logischen Button-Masken. Remapping erforderlich.

4. **Trigger Rauschen**: Bei losgelassenen Triggern können Werte von 0-5 auftreten. Threshold von ~10 empfohlen.

5. **Report Rate**: ~8 Hz (120ms) - für reaktive Steuerung ausreichend, aber nicht ideal für schnelle Spiele.

## Verbindungsaufbau

1. Controller in Pairing-Modus versetzen (Xbox-Taste 3s halten)
2. BLE Scan nach "Xbox Wireless Controller"
3. Verbinden und Pairing durchführen (Level 2 Security)
4. HID Service (0x1812) discovern
5. Report Characteristic (0x001e) subscriben
6. Reports empfangen und parsen

## Referenzen

- [USB HID Usage Tables](https://usb.org/hid)
- [Xbox Accessories App](https://www.xbox.com/accessories-app)
- micro:bit v2 Firmware: `imu_orientation/firmware/src/hid_parser.c`
