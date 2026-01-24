# Debugging micro:bit v2 ohne externen SWD-Debugger

## Übersicht

Der micro:bit v2 hat einen **eingebauten CMSIS-DAP Debug-Adapter** - kein externer SWD-Debugger wie J-Link oder ST-Link ist erforderlich!

## Offizielle Dokumentation

- **DAPLink Interface**: https://tech.microbit.org/software/daplink-interface/
- **Debugging Guide**: https://microbit-v2-debugging.readthedocs.io/
- **pyOCD**: https://pyocd.io/
- **OpenOCD micro:bit**: https://natlandsmyr.com/2023/04/05/openocd-microbit.html

## Architektur

```
┌──────────┐      USB      ┌──────────────┐      SWD      ┌──────────┐
│   PC     │◄─────────────►│ Interface MCU │◄─────────────►│ Main MCU │
│          │               │ (DAPLink)     │               │ nRF52833 │
│ GDB/IDE  │               │ CMSIS-DAP     │               │ (Target) │
└──────────┘               └──────────────┘               └──────────┘
```

Der Interface-Chip (nRF52833/nRF52820 auf V2.2x) läuft DAPLink Firmware und stellt CMSIS-DAP über USB bereit.

## Option 1: pyOCD (Empfohlen)

### Installation

```bash
pip install pyocd

# Optional: Zusätzliche Target-Packs
pyocd pack install nrf52833
```

### GDB Server starten

```bash
# Standard
pyocd gdbserver -t nrf52833

# Mit Optionen
pyocd gdbserver -t nrf52833 --persist -f 1000000

# Verfügbare Targets anzeigen
pyocd list --targets
```

### Firmware flashen

```bash
# HEX-Datei flashen
pyocd flash -t nrf52833 firmware.hex

# Mit Erase
pyocd flash -t nrf52833 --erase chip firmware.hex
```

### GDB Verbindung

```bash
# GDB starten
arm-none-eabi-gdb firmware.elf

# In GDB:
(gdb) target remote localhost:3333
(gdb) load
(gdb) monitor reset
(gdb) break main
(gdb) continue
```

## Option 2: OpenOCD

### Installation

```bash
# macOS
brew install openocd

# Linux
sudo apt install openocd

# Windows: Download von openocd.org
```

### GDB Server starten

```bash
openocd -f interface/cmsis-dap.cfg -f target/nrf52.cfg
```

Erfolgreiche Ausgabe:
```
Info : Using CMSIS-DAPv2 interface with VID:PID=0x0d28:0x0204
Info : CMSIS-DAP: SWD supported
Info : nRF52833 detected
Info : starting gdb server for nrf52.cpu on 3333
```

### GDB Verbindung

```bash
arm-none-eabi-gdb firmware.elf

(gdb) target remote :3333
(gdb) monitor reset halt
(gdb) load
(gdb) break main
(gdb) continue
```

## Option 3: VS Code Integration

### Extensions installieren

1. **Cortex-Debug** (marus25.cortex-debug)
2. Optional: **C/C++** (ms-vscode.cpptools)

### launch.json Konfiguration

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug micro:bit (pyOCD)",
            "type": "cortex-debug",
            "request": "launch",
            "servertype": "pyocd",
            "cwd": "${workspaceRoot}",
            "executable": "${workspaceRoot}/build/firmware.elf",
            "targetId": "nrf52833",
            "runToEntryPoint": "main",
            "showDevDebugOutput": "parsed"
        },
        {
            "name": "Debug micro:bit (OpenOCD)",
            "type": "cortex-debug",
            "request": "launch",
            "servertype": "openocd",
            "cwd": "${workspaceRoot}",
            "executable": "${workspaceRoot}/build/firmware.elf",
            "configFiles": [
                "interface/cmsis-dap.cfg",
                "target/nrf52.cfg"
            ],
            "runToEntryPoint": "main"
        }
    ]
}
```

## Nützliche GDB Befehle

| Befehl | Beschreibung |
|--------|--------------|
| `b main` | Breakpoint bei main |
| `b *0x18310` | Breakpoint an Adresse |
| `c` | Continue |
| `n` | Next (Step Over) |
| `s` | Step (Step Into) |
| `si` | Step Instruction |
| `p variable` | Variable ausgeben |
| `x/10x 0x20000000` | Memory dump |
| `info reg` | Register anzeigen |
| `bt` | Backtrace |
| `mon reset` | Target Reset |
| `mon halt` | Target anhalten |

## Watchpoints

```gdb
# Watch auf Variable
watch my_variable

# Watch auf Speicheradresse
watch *0x20003ffc

# Read Watchpoint
rwatch *0x20003ffc

# Access Watchpoint (read oder write)
awatch *0x20003ffc
```

## Semihosting (printf über Debug)

Mit Semihosting können `printf` Ausgaben über den Debugger zum PC geschickt werden.

### In Code aktivieren

```c
#include <stdio.h>

extern void initialise_monitor_handles(void);

int main(void)
{
    initialise_monitor_handles();
    printf("Debug output!\n");
    // ...
}
```

### In OpenOCD aktivieren

```bash
openocd -f interface/cmsis-dap.cfg -f target/nrf52.cfg \
        -c "arm semihosting enable"
```

## Troubleshooting

### "No device found"

1. Prüfe USB-Verbindung
2. Prüfe ob micro:bit als USB-Laufwerk erscheint
3. Versuche anderen USB-Port

### "Target not halted"

```gdb
(gdb) monitor reset halt
```

### "Flash write failed"

```bash
# Chip komplett löschen
pyocd erase -t nrf52833 --chip

# Dann neu flashen
pyocd flash -t nrf52833 firmware.hex
```

### Langsame Debug-Verbindung

```bash
# Höhere SWD-Frequenz
pyocd gdbserver -t nrf52833 -f 4000000
```
