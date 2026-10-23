# STM32 FreeRTOS Sensor Demo

A small multi-task FreeRTOS application on the **NUCLEO-F446RE** (STM32F446RE, Cortex-M4F) that demonstrates inter-task communication with a **message queue** and shared-resource protection with a **mutex**.

The on-chip temperature sensor is sampled by one task, passed through a queue to a second task that prints it over UART, while a third task deliberately floods the same UART to create contention.

## Features

- Three FreeRTOS tasks (CMSIS-RTOS v2 API)
- Producer/consumer pattern using a message queue
- Mutex-protected UART output, with a compile-time switch to observe the failure mode without it
- ADC reading of the internal temperature sensor, converted with fixed-point (integer-only) arithmetic
- `printf` retargeted to USART2 (ST-Link Virtual COM Port)

## Architecture

```
 [sensorTask] --ADC--> temp (x0.1 C, int32_t)
       |
       |  osMessageQueuePut()
       v
  [tempQueue]  (depth 8)
       |
       |  osMessageQueueGet(osWaitForever)
       v
 [printTask] ----+
                 |   uartMutex
 [blinkTask] ----+-------------> USART2 (printf)
   (LED toggle + UART load)
```

| Task       | Role                                                           | Stack (words) |
|------------|----------------------------------------------------------------|---------------|
| sensorTask | Samples ADC1 temperature channel every 1 s, pushes to queue    | 512           |
| printTask  | Blocks on the queue, prints each reading with a sequence number| 512           |
| blinkTask  | Toggles LD2 and prints a long line every 5 ms to load the UART | 512           |

## Design notes

**Event-driven consumer.** `printTask` has no delay of its own. It blocks on `osMessageQueueGet(..., osWaitForever)` and consumes no CPU until `sensorTask` posts data, so the queue provides both data transfer and synchronization.

**Non-blocking producer.** `sensorTask` posts with a timeout of 0. If the queue is ever full, the newest sample is dropped instead of stalling the sampling task.

**Fixed-point conversion.** The datasheet relation `T = (Vsense - 0.76 V) / 2.5 mV + 25` is rewritten in millivolts and tenths of a degree as `T_x10 = (mV - 760) * 4 + 250`, avoiding floating point and `%f` (unsupported by newlib-nano by default).

**Why the mutex wraps `printf`, not `_write`.** A single `printf` may call `_write` more than once. Locking around the whole `printf` guarantees each line is transmitted as one unit.

## Mutex experiment

`main.c` contains:

```c
#define USE_UART_MUTEX  1
```

- **`USE_UART_MUTEX 0`**: under UART load from `blinkTask`, the sequence numbers printed by `printTask` skip (e.g. `[12]` followed by `[14]`). When one task is inside `HAL_UART_Transmit`, the HAL handle is busy and a second caller gets `HAL_BUSY`, so that line is silently lost.
- **`USE_UART_MUTEX 1`**: the second task blocks on the mutex and waits its turn. Sequence numbers are continuous.

Example output with the mutex enabled:

```
[blink] 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789
[blink] 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789
[0] temp = 31.2 C
[blink] 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789
[1] temp = 31.3 C
```

## Configuration highlights

- FreeRTOS via STM32CubeMX, CMSIS-RTOS v2 interface
- HAL timebase moved from SysTick to TIM6 (SysTick is used by the FreeRTOS scheduler)
- `USE_NEWLIB_REENTRANT` enabled so each task has its own newlib state when calling `printf`
- USART2: 115200 baud, 8N1

## Hardware

- NUCLEO-F446RE
- No external components (internal temperature sensor, on-board LD2, ST-Link VCP)

## Build and run

Developed with STM32CubeIDE 2.1.1 and standalone STM32CubeMX on Linux.

1. In STM32CubeIDE: **File → Open Projects from File System**, select this folder.
2. Build the project.
3. Connect the board, then **Run As → STM32 C/C++ Application**.
4. Open the serial port at 115200 8N1, e.g.

   ```
   picocom -b 115200 /dev/ttyACM0
   ```

   (On Windows, use the "STLink Virtual COM Port" shown in Device Manager.)

## Possible next steps

- Replace blocking `HAL_UART_Transmit` with interrupt- or DMA-driven transmission
- Port to the native FreeRTOS API and a Makefile/CMake build with `arm-none-eabi-gcc`
- Add an external sensor (e.g. accelerometer over I2C/SPI)
