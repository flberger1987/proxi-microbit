# Zephyr RTOS für micro:bit v2

## Übersicht

Zephyr ist ein skalierbares, optimiertes, sicheres Echtzeit-Betriebssystem (RTOS) mit vollständiger Unterstützung für den micro:bit v2.

## Offizielle Dokumentation

- **Zephyr micro:bit v2 Board**: https://docs.zephyrproject.org/latest/boards/bbc/microbit_v2/doc/index.html
- **Zephyr GitHub Board Support**: https://github.com/zephyrproject-rtos/zephyr/blob/main/boards/bbc/microbit_v2/
- **Zephyr Samples**: https://docs.zephyrproject.org/latest/samples/index.html
- **OpenThread Tutorial**: https://zephyrproject.org/the-bbc-microbit-v2-and-openthread/

## Installation

### 1. Zephyr SDK installieren

```bash
# West (Zephyr Meta-Tool) installieren
pip install west

# Zephyr Workspace erstellen
west init ~/zephyrproject
cd ~/zephyrproject
west update

# Zephyr SDK installieren (Download von zephyrproject.org)
# macOS:
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/zephyr-sdk-0.16.8_macos-x86_64.tar.xz
tar xf zephyr-sdk-0.16.8_macos-x86_64.tar.xz
cd zephyr-sdk-0.16.8
./setup.sh

# Python Dependencies
pip install -r ~/zephyrproject/zephyr/scripts/requirements.txt
```

### 2. Umgebung einrichten

```bash
# Zephyr Environment laden
source ~/zephyrproject/zephyr/zephyr-env.sh

# Oder in .bashrc/.zshrc:
export ZEPHYR_BASE=~/zephyrproject/zephyr
```

## Projekt erstellen

### Einfaches Beispiel: Blinky

```bash
cd ~/zephyrproject/zephyr

# Build für micro:bit v2
west build -b bbc_microbit_v2 samples/basic/blinky

# Flashen
west flash
```

### Eigenes Projekt

Projektstruktur:
```
my_project/
├── CMakeLists.txt
├── prj.conf
└── src/
    └── main.c
```

**CMakeLists.txt**:
```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_project)
target_sources(app PRIVATE src/main.c)
```

**prj.conf**:
```conf
CONFIG_GPIO=y
CONFIG_PRINTK=y
```

**src/main.c**:
```c
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define LED_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

int main(void)
{
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    while (1) {
        gpio_pin_toggle_dt(&led);
        k_msleep(500);
    }
    return 0;
}
```

## Threading mit Zephyr

### Einfaches Thread-Beispiel

```c
#include <zephyr/kernel.h>

#define STACK_SIZE 1024
#define THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(thread1_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread2_stack, STACK_SIZE);

struct k_thread thread1_data;
struct k_thread thread2_data;

void thread1_entry(void *p1, void *p2, void *p3)
{
    while (1) {
        printk("Thread 1 running\n");
        k_msleep(1000);
    }
}

void thread2_entry(void *p1, void *p2, void *p3)
{
    while (1) {
        printk("Thread 2 running\n");
        k_msleep(500);
    }
}

int main(void)
{
    k_thread_create(&thread1_data, thread1_stack,
                    K_THREAD_STACK_SIZEOF(thread1_stack),
                    thread1_entry, NULL, NULL, NULL,
                    THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_create(&thread2_data, thread2_stack,
                    K_THREAD_STACK_SIZEOF(thread2_stack),
                    thread2_entry, NULL, NULL, NULL,
                    THREAD_PRIORITY + 1, 0, K_NO_WAIT);

    return 0;
}
```

### Synchronisation

```c
#include <zephyr/kernel.h>

// Semaphore
K_SEM_DEFINE(my_sem, 0, 1);

// Mutex
K_MUTEX_DEFINE(my_mutex);

// Message Queue
K_MSGQ_DEFINE(my_msgq, sizeof(int), 10, 4);

void producer_thread(void)
{
    int data = 42;
    k_msgq_put(&my_msgq, &data, K_FOREVER);
}

void consumer_thread(void)
{
    int data;
    k_msgq_get(&my_msgq, &data, K_FOREVER);
    printk("Received: %d\n", data);
}
```

## prj.conf Optionen für micro:bit v2

```conf
# Threading
CONFIG_MULTITHREADING=y
CONFIG_NUM_PREEMPT_PRIORITIES=16

# Bluetooth
CONFIG_BT=y
CONFIG_BT_PERIPHERAL=y

# Display (LED Matrix)
CONFIG_DISPLAY=y
CONFIG_MICROBIT_DISPLAY=y

# I2C (Sensoren)
CONFIG_I2C=y
CONFIG_SENSOR=y

# USB Console
CONFIG_USB_DEVICE_STACK=y
CONFIG_USB_CDC_ACM=y
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y

# Debugging
CONFIG_DEBUG=y
CONFIG_DEBUG_THREAD_INFO=y
```

## Build & Flash

```bash
# Build
west build -b bbc_microbit_v2 .

# Flash (nutzt DAPLink automatisch)
west flash

# Flash mit spezifischem Runner
west flash --runner pyocd

# Clean Build
west build -t pristine
```

## Debugging mit Zephyr

```bash
# GDB Server starten
west debugserver

# In anderem Terminal: GDB verbinden
west debug
```

## Nützliche Zephyr Samples für micro:bit

| Sample | Beschreibung |
|--------|--------------|
| `samples/basic/blinky` | LED blinken |
| `samples/basic/button` | Button Events |
| `samples/basic/threads` | Multi-Threading |
| `samples/bluetooth/peripheral` | BLE Peripheral |
| `samples/sensor/bmi160` | Accelerometer |
| `samples/philosophers` | RTOS Synchronisation |
