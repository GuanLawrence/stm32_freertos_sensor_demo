# STM32 FreeRTOS Sensor Demo

A small multi-task FreeRTOS application on the **NUCLEO-F446RE** (STM32F446RE, Cortex-M4F) that demonstrates inter-task communication with a **message queue**, shared-resource protection with a **mutex**, and interrupt-to-task signaling with **semaphores and task notifications**.

The on-chip temperature sensor is sampled by one task, passed through a queue to a second task that prints it over UART, while a third task deliberately floods the same UART to create contention. A fourth task is woken by the USER button interrupt through a semaphore.

## Features

- Four FreeRTOS tasks (CMSIS-RTOS v2 API)
- Producer/consumer pattern using a message queue
- Mutex-protected UART output, with a compile-time switch to observe the failure mode without it
- ADC reading of the internal temperature sensor, converted with fixed-point (integer-only) arithmetic
- `printf` retargeted to USART2 (ST-Link Virtual COM Port)
- Button EXTI with deferred processing; comparison of binary/counting semaphores, thread flags and task notifications
- Gatekeeper logging task with UART DMA; measured caller latency and drop behavior under overload
- Per-task stack high water mark reporting and stack overflow detection
- IWDG with per-task health check-ins; watchdog cause preserved across reset

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
   (LED toggle   |
    + UART load) |
                 |
 [buttonTask] ---+
       ^
       |  osSemaphoreAcquire()
  [buttonSem / buttonCountSem]
       ^
       |  osSemaphoreRelease()
  EXTI ISR (USER button, PC13)
```

| Task                | Role                                                                                                                                    | Stack (words) |
|-----------------|--------------------------------------------------------------------------------------------|-------------------|
| sensorTask  | Samples ADC1 temperature channel every 1 s, pushes to queue           | 512                     |
| printTask      | Blocks on the queue, prints each reading with a sequence number     | 512                     |
| blinkTask      | Toggles LD2 and prints a long line every 5 ms to load the UART             | 512                     |
| buttonTask  | Woken by the button ISR via semaphore, simulates slow processing   | 512                     |

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
[0] temp = 29.8 C
[blink] 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789
[1] temp = 30.6 C
```

## Button sync experiment: semaphores, thread flags, task notification

The USER button (PC13) triggers an EXTI interrupt. The ISR only signals
`buttonTask` (osPriorityNormal), which does the actual work. This is deferred
interrupt processing: keep the ISR short, move the work to a task.

`buttonTask` sleeps 2 s after each event to simulate slow processing.
`main.c` selects the signaling mechanism:

```c
#define BUTTON_SYNC  0   // 0: binary sem, 1: counting sem, 2: CMSIS thread flags, 3: native task notification
```

Pressing the button 5 times within 2 s:

| `BUTTON_SYNC` | Mechanism | Events processed | Why |
|---|---|---|---|
| 0 | Binary semaphore | 2 | Max count 1; extra releases are discarded while a token is pending |
| 1 | Counting semaphore (max 10, initial 0) | 5 | Every release is counted |
| 2 | CMSIS thread flags | 2 | Setting the same bit repeatedly is idempotent; the wait clears it |
| 3 | Native task notification (`ulTaskNotifyTake(pdFALSE, ...)`) | 5 | Each give increments the notification value; each take decrements it by 1 |

Notes:

- Thread flags are implemented on FreeRTOS task notifications, so they need no
  separate kernel object. Do not mix thread flags and native notifications on
  the same task: they share the same notification value.
- Mode 3 uses the native API directly in the ISR (`vTaskNotifyGiveFromISR` +
  `portYIELD_FROM_ISR`), which the CMSIS wrapper otherwise handles internally.
- The button EXTI priority is 15, numerically above
  `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` (5), so the ISR may call
  FreeRTOS APIs.

- FreeRTOS via STM32CubeMX, CMSIS-RTOS v2 interface
- HAL timebase moved from SysTick to TIM6 (SysTick is used by the FreeRTOS scheduler)
- `USE_NEWLIB_REENTRANT` enabled so each task has its own newlib state when calling `printf`
- USART2: 115200 baud, 8N1

## Logging experiment: direct printf vs gatekeeper task

`main.c`:

```c
#define USE_GATEKEEPER  1   // 0: tasks print directly (blocking UART + mutex), 1: log queue + gatekeeper task with UART DMA
```

- **Mode 0**: each task calls `printf`, which blocks in `HAL_UART_Transmit` until every byte is sent. The UART is protected by `uartMutex`.
- **Mode 1**: tasks format the message with `vsnprintf` and post it to a 16-entry log queue, then return immediately. A single `logTask` owns the UART, sends each message with `HAL_UART_Transmit_DMA`, and waits for the TX-complete callback (thread flag) before taking the next one. No mutex is needed.

Time spent by `blinkTask` per log call, measured with the DWT cycle counter:

| Mode | blinkTask period | Max time per log call | Dropped |
|---|---|---|---|
| 0: direct printf + mutex | 5 ms | 5487 µs | 0 |
| 1: gatekeeper + DMA | 5 ms | 34 µs | 0 |
| 1: gatekeeper + DMA | 4 ms (overload) | 23–28 µs | ~46 / s |

**Mode 0 never drops, but the caller pays.** Each 56-byte line takes about 4.9 ms at 115200 baud, so `blinkTask`'s loop stretches from 5 ms to about 10 ms. The UART's limit is pushed back onto the producer.

**Mode 1 keeps callers fast, but cannot create bandwidth.** At 115200 8N1 the UART carries about 11520 B/s, or roughly 205 blink lines per second. With a 4 ms period `blinkTask` produces 250 lines per second, so about 45 lines per second must be dropped. The measured rate was 46 per second.

**Drop policy matters.** With a single drop-when-full policy, the once-per-second temperature line was dropped along with the flood of blink lines. Important messages now use `LOG_IMPORTANT()`, which waits up to 100 ms for a free slot, while `blinkTask` uses `LOG()`, which drops immediately. Under overload only blink lines are lost.

Notes:

- The DMA TX buffer must stay unchanged until the transfer completes, so `logTask` waits for `FLAG_TX_DONE` before reusing it.
- The STM32F446 (Cortex-M4) has no data cache, so no cache maintenance is needed before DMA. On a Cortex-M7 the buffer would have to be cleaned (`SCB_CleanDCache_by_Addr`) first.
- In mode 1, `printf` must not be called anywhere: `_write` still uses blocking `HAL_UART_Transmit`, which would collide with the DMA transfer.

## Stack usage experiment

FreeRTOS fills each task stack with `0xA5` when the task is created.
`osThreadGetStackSpace()` (which calls `uxTaskGetStackHighWaterMark()`) scans
for untouched bytes and reports the minimum free stack since start-up.
`blinkTask` logs it for every task every 10 readings.

Measured after several minutes, with the button pressed:

| Task | Allocated (bytes) | Min free | Used | Work |
|---|---|---|---|---|
| blinkTask | 2048 | 1544 | 504 | `vsnprintf` + queue |
| printTask | 2048 | 1456 | 592 | `vsnprintf` with the longest format |
| buttonTask | 2048 | 1488 | 560 | `vsnprintf` + queue |
| sensorTask | 2048 | 1848 | 200 | ADC + queue |
| logTask | 1024 | 808 | 216 | queue + DMA start (static buffer) |

Tasks that format strings use 2–3× more stack than those that don't; most of it
comes from `vsnprintf` and the 64-byte local message buffer. A high water mark
only covers code paths exercised during the test, so a margin is still required.

### Overflow detection

`configCHECK_FOR_STACK_OVERFLOW` is set to 2: at each context switch the kernel
checks that the lowest 16 bytes of the outgoing task's stack still hold `0xA5`,
and calls `vApplicationStackOverflowHook()` if not.

```c
#define STACK_OVERFLOW_TEST  0   // 1: blinkTask allocates a large local array to overflow its stack
```

Setting it to 1 makes `blinkTask` allocate a 1900-byte local array on top of its
normal ~500 bytes of usage. The hook was called and the debugger showed
`stack_overflow_task = "blinkTask"`.

Limitations of this software check:

- It runs only at context switches, so memory beyond the stack may already be
  corrupted when the hook is called. The hook therefore disables interrupts and
  halts instead of trying to continue.
- It reports the task whose guard bytes are damaged. If one task overflows into
  another task's memory, the reported task can be the victim, not the culprit.
- An overflow that jumps past the 16-byte pattern without writing it is missed.

Hardware alternatives are an MPU guard region at the bottom of each stack or, on
ARMv8-M cores, the PSPLIM stack limit register.

## Watchdog experiment

The independent watchdog (IWDG, ~4 s timeout from the LSI clock) is refreshed
only when every monitored task has recently proven it is alive.

- `blinkTask`, `sensorTask` and `printTask` each set their own bit in an
  event flags object (`aliveFlags`).
- `wdgTask` (osPriorityAboveNormal) waits up to 1.5 s for all bits with
  `osFlagsWaitAll`. If all arrive it refreshes the IWDG; otherwise it logs the
  missing bits and does not refresh.
- `__HAL_DBGMCU_FREEZE_IWDG()` stops the IWDG while the debugger halts the core.

Refreshing the watchdog from a single task would only prove that one task is
running; requiring a check-in from each task catches any one of them hanging.

```c
#define WDG_FAULT_TEST  0   // 1: sensorTask hangs after 10 readings
```

With `WDG_FAULT_TEST 1`, `sensorTask` enters an infinite loop:

```
WDG: missing 0x06
WDG: missing 0x06
*** Reset cause: IWDG ***
*** [BKP]    last missing 0x06 ***
*** [noinit] last missing 0x06 ***
```

`0x06` is `sensorTask` (0x02) plus `printTask` (0x04). `printTask` is not
faulty, but it blocks on the queue fed by `sensorTask`, so it stops checking in
too. The watchdog reports which functions stopped; identifying the root cause
requires knowing the data flow. Checking in on a receive timeout instead would
isolate `sensorTask`. `blinkTask` keeps running thanks to time slicing.

### Keeping the cause across the reset

The IWDG has no interrupt, so `wdgTask` stores the missing bits as soon as it
decides not to refresh. They are kept in two places, both with a magic number to
reject stale or random content:

- RTC backup register `BKP0R` (backup domain; survives system reset, and power
  loss if VBAT has a battery)
- A `.noinit` RAM section added to the linker script (`NOLOAD`, not cleared by
  the startup code; survives system reset, lost on power loss)

On boot, if the IWDG reset flag is set, both are read and printed, then cleared.
On the NUCLEO board VBAT is tied to VDD, so both are lost when USB power is removed.


## Hardware

- NUCLEO-F446RE
- No external components (internal temperature sensor, on-board LD2, USER button B1, ST-Link VCP)

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
