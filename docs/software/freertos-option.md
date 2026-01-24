# FreeRTOS Option für micro:bit v2

## Übersicht

FreeRTOS ist ein weit verbreitetes Open-Source RTOS, das auf dem nRF52833 (micro:bit v2 MCU) lauffähig ist.

## Offizielle Quellen

- **Nordic DevZone - FreeRTOS nRF52833**: https://devzone.nordicsemi.com/f/nordic-q-a/75825/support-for-freertos-in-nrf52833
- **MakeCode Forum Diskussion**: https://forum.makecode.com/t/compile-c-and-freertos-for-microbit-v-2/17233
- **Nordic nRF5 SDK FreeRTOS**: https://docs.aws.amazon.com/freertos/latest/userguide/getting_started_nordic.html

## Status

FreeRTOS ist **nicht offiziell** für den micro:bit v2 unterstützt, aber technisch möglich:

1. Der nRF52833 wird von FreeRTOS unterstützt
2. Es gibt FreeRTOS Beispiele für nRF52840-DK (ähnlicher Chip)
3. Anpassungen für nRF52833 sind erforderlich

## Empfehlung

**Zephyr RTOS wird empfohlen** statt FreeRTOS für micro:bit v2:

| Kriterium | FreeRTOS | Zephyr |
|-----------|----------|--------|
| micro:bit Support | Inoffiziell | Offiziell |
| Board Definition | Muss erstellt werden | Vorhanden |
| Dokumentation | Allgemein nRF52 | micro:bit spezifisch |
| Community | Gering für micro:bit | Aktiv |
| Build System | Manuell | West (integriert) |

## Falls FreeRTOS gewünscht

### Ansatz 1: Nordic nRF5 SDK + FreeRTOS

1. Nordic nRF5 SDK herunterladen
2. FreeRTOS Port für nRF52 verwenden
3. Board-spezifische Anpassungen vornehmen

### Ansatz 2: Über PlatformIO

```ini
; platformio.ini
[env:microbit_v2]
platform = nordicnrf52
board = bbcmicrobit_v2
framework = freertos
```

**Hinweis**: Experimentell und möglicherweise nicht vollständig unterstützt.

## FreeRTOS Konzepte

Falls Sie mit FreeRTOS arbeiten möchten, hier die Grundkonzepte:

### Tasks (Threads)

```c
#include "FreeRTOS.h"
#include "task.h"

void vTask1(void *pvParameters)
{
    while(1)
    {
        // Task Code
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void)
{
    xTaskCreate(vTask1, "Task1", 256, NULL, 1, NULL);
    vTaskStartScheduler();

    while(1); // Sollte nie erreicht werden
}
```

### Semaphoren

```c
#include "semphr.h"

SemaphoreHandle_t xSemaphore;

void vProducer(void *pvParameters)
{
    while(1)
    {
        // Produziere etwas
        xSemaphoreGive(xSemaphore);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vConsumer(void *pvParameters)
{
    while(1)
    {
        if(xSemaphoreTake(xSemaphore, portMAX_DELAY))
        {
            // Konsumiere
        }
    }
}
```

### Queues

```c
#include "queue.h"

QueueHandle_t xQueue;

void vSender(void *pvParameters)
{
    int value = 42;
    xQueueSend(xQueue, &value, portMAX_DELAY);
}

void vReceiver(void *pvParameters)
{
    int received;
    xQueueReceive(xQueue, &received, portMAX_DELAY);
}
```

## Tickless Idle für Stromsparen

FreeRTOS auf Nordic Chips unterstützt Tickless Idle:

```c
// FreeRTOSConfig.h
#define configUSE_TICKLESS_IDLE 1
```

Dies unterdrückt Timer-Interrupts während Idle und spart erheblich Strom.
