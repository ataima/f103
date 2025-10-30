# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a bare-metal embedded project for the **STM32F103C8T6** microcontroller (commonly known as "Blue Pill"). The project is configured to run at maximum speed (72 MHz) using an external 8 MHz crystal with PLL multiplier.

**Target Hardware:**
- MCU: STM32F103C8T6 (ARM Cortex-M3)
- Flash: 64 KB @ 0x08000000
- RAM: 20 KB total (18 KB general + 2 KB dedicated log buffer)
- Clock: HSE 8 MHz → PLL ×9 → 72 MHz SYSCLK

## Project Structure

```
f103/
├── inc/                        # Header files
│   ├── common.h               # Common types, macros, includes
│   ├── stm32f103_regs.h       # Register definitions
│   ├── clock.h                # Clock configuration API
│   ├── log.h                  # Logging system API
│   ├── itm.h                  # ITM console via SWO API
│   ├── uart.h                 # UART console API
│   ├── gpio.h                 # GPIO configuration API (CNC controller)
│   ├── systick.h              # SysTick system timer API
│   ├── test.h                 # Hardware test functions
│   └── utils.h                # Utility functions
├── src/                        # Source files
│   ├── main.c                 # Main application
│   ├── clock.c                # Clock configuration
│   ├── log.c                  # Logging system
│   ├── itm.c                  # ITM console support
│   ├── uart.c                 # UART console support
│   ├── gpio.c                 # GPIO configuration (CNC controller)
│   ├── systick.c              # SysTick system timer
│   ├── test.c                 # Hardware test implementations
│   ├── utils.c                # Utility functions
│   ├── syscalls.c             # Semihosting support
│   └── sysmem.c               # Memory management
├── doc/                        # Documentation
│   ├── pin_out.md             # GPIO pinout for 3-axis CNC
│   ├── rm0008-reference-manual.pdf
│   └── stm32f103c8.pdf
├── Startup/
│   └── startup_stm32f103c8tx.s # Startup assembly code
├── CMakeLists.txt              # Build configuration
├── cubeide-gcc.cmake           # ARM GCC toolchain file
├── STM32F103C8TX_FLASH.ld      # Linker script
└── CLAUDE.md                   # This file
```

## Build System

This project uses **CMake** with a custom toolchain file for ARM GCC.

### Environment Setup

Before building, you must set up the toolchain PATH:

```bash
source source_this
```

This script adds the STM32CubeIDE ARM GCC toolchain to your PATH. You need to run this **once per terminal session** before building.

### Build Commands

```bash
# Quick build (after sourcing environment)
source source_this
make -C Debug -j

# Or use convenience scripts (these source the environment automatically)
./do         # Build
./rebuild    # Clean and rebuild
./distclean  # Remove all build artifacts

# Manual CMake build
source source_this
cmake -DCMAKE_TOOLCHAIN_FILE=cubeide-gcc.cmake -S ./ -B Debug -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug
make -C Debug -j

# Release build
source source_this
cmake -DCMAKE_TOOLCHAIN_FILE=cubeide-gcc.cmake -S ./ -B Release -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
make -C Release -j
```

The output is `Debug/f103.elf` or `Release/f103.elf`.

**Typical binary sizes (Debug build):**
- Flash (text): ~11.7 KB (11956 bytes)
  - System initialization and drivers
  - Includes: clock, GPIO, SysTick, UART, ITM, logging, hardware tests
- Data: 8 bytes (initialized variables)
- RAM (bss): ~4 KB (4040 bytes)
  - 2 KB dedicated log buffer (0x20004800)
  - SysTick tick counter and statistics (~16 bytes)
  - Test module state variables (~24 bytes)
  - Other uninitialized variables
- **Total:** ~16 KB

**Flash usage:** ~18.4% of 64 KB available
**RAM usage:** ~20% of 20 KB available

### Adding New Source Files

Edit `CMakeLists.txt` and add files to the `PROJECT_SOURCES` variable (lines 48-62).

## Architecture

### Memory Layout (STM32F103C8TX_FLASH.ld)

The linker script defines a **custom memory partition** that separates logging RAM from general RAM:

- **RAM (0x20000000 - 0x20003FFF):** 16 KB for general use (.data, .bss, heap, stack)
- **RAM_LOG (0x20004800 - 0x20004FFF):** 4 KB dedicated log buffer (`.log_buffer` section)
- **FLASH (0x08000000 - 0x0800FFFF):** 64 KB for code and constants

The log buffer uses `(NOLOAD)` to preserve contents across soft resets for post-mortem debugging.

**Note:** The actual usable log buffer size is 2048 bytes (2 KB) due to the `LOG_BUFFER_SIZE` constant in log.h, even though 4 KB is reserved in the linker script.

### Clock Configuration (src/clock.c, inc/clock.h)

The clock system is configured in a **strict sequence** to reach 72 MHz safely:

1. Enable HSE (8 MHz external oscillator)
2. Configure Flash wait states (2 WS required for 72 MHz)
3. Configure bus prescalers **before** enabling PLL:
   - AHB: SYSCLK / 1 = 72 MHz (HCLK)
   - APB1: HCLK / 2 = 36 MHz (max allowed for APB1)
   - APB2: HCLK / 1 = 72 MHz
   - ADC: PCLK2 / 6 = 12 MHz
4. Configure PLL: HSE × 9 = 72 MHz
5. Enable PLL and wait for lock
6. Switch SYSCLK to PLL
7. Enable Clock Security System (CSS)

**Critical constraints:**
- APB1 must not exceed 36 MHz (hardware limit)
- Flash must be configured to 2 wait states before switching to 72 MHz
- All register writes use direct memory-mapped access (no HAL/SPL dependencies)

Functions:
- `SystemClock_Config()` - Initialize to 72 MHz (returns CLOCK_OK, CLOCK_ERROR_HSE, or CLOCK_ERROR_PLL)
- `SystemClock_GetFreq()`, `SystemClock_GetHCLK()`, `SystemClock_GetPCLK1()`, `SystemClock_GetPCLK2()` - Query current frequencies

### Logging System (src/log.c, inc/log.h)

A **thread-safe circular buffer** logging system with dedicated RAM allocation:

**Architecture:**
- 2 KB buffer in dedicated RAM section (`.log_buffer`)
- Variable-length message format: `[LENGTH(2B)][MESSAGE]['\0']`
- Thread-safe via global interrupt disable (`__disable_irq` / `__enable_irq`)
- Automatic overflow handling: oldest messages are removed when full

**API:**
- `log_init()` - Initialize (called once after clock config)
- `log_write(msg)` - Write raw message (PUSH)
- `log_read(dest)` - Read oldest message (POP)
- Convenience macros with prefixes (support printf-style formatting):
  - `log_error("msg", ...)` → `[ERR]:msg`
  - `log_warning("msg", ...)` → `[WRN]:msg`
  - `log_info("msg", ...)` → `[INF]:msg`
  - `log_debug("msg", ...)` → `[DBG]:msg`
- Statistics: `log_get_count()`, `log_get_used_bytes()`, `log_get_free_bytes()`
- `log_clear()` - Empty buffer
- `log_get_buffer_address()` - Get RAM address (0x20004800) for debugger inspection

**Important:**
- Messages limited to `LOG_MAX_MESSAGE_SIZE` (128 bytes)
- Lock acquisition can fail (returns `LOG_ERROR_LOCKED`) if buffer is busy
- Buffer survives soft resets due to `(NOLOAD)` linker attribute

### Main Application (src/main.c)

**Initialization sequence:**
1. `SystemClock_Config()` - Configure clock to 72 MHz
2. Delay for SWV/ITM sync (workaround for debugger)
3. `log_init()` - Initialize logging system
4. `itm_init()` - Initialize ITM console via SWO (optional, if `ENABLE_ITM` is set)
5. `gpio_init_all()` - Initialize all GPIO for CNC controller
6. `systick_init()` - Initialize SysTick timer for 1ms system tick
7. `uart_init()` - Initialize UART3 console (115200 baud, PB10=TX, PB11=RX)
8. Main loop with hardware tests and LED status blink pattern

**Main loop behavior:**
The main loop executes hardware tests (if enabled) and manages the LED status indicator:

```c
int limit_active = 0;

while(1) {
    // Execute enabled hardware tests
    EVAL_TEST_LIMIT(limit_active = test_limit();)
    EVAL_TEST_STEP_IO(limit_active = test_step_io();)
    EVAL_TEST_DIR_IO(limit_active = test_dir_io();)
    EVAL_TEST_ENABLE_IO(limit_active = test_enable_io();)

    if (!limit_active) {
        // Normal LED blink pattern: 200ms OFF → 400ms ON → 400ms OFF → 200ms ON
        gpio_set(LED_STATUS_PORT, LED_STATUS_PIN);     // OFF
        systick_delay_ms(200);
        gpio_reset(LED_STATUS_PORT, LED_STATUS_PIN);   // ON
        systick_delay_ms(400);
        gpio_set(LED_STATUS_PORT, LED_STATUS_PIN);     // OFF
        systick_delay_ms(400);
        gpio_reset(LED_STATUS_PORT, LED_STATUS_PIN);   // ON
        systick_delay_ms(200);
    } else {
        // Test detected issue: LED stays ON continuously
        gpio_reset(LED_STATUS_PORT, LED_STATUS_PIN);
        systick_delay_ms(100);
    }
}
```

**LED Status Indicator (PC13):**
- **Normal operation:** Recognizable blink pattern (1200ms cycle)
- **Test failure:** LED stays ON continuously
- **LED type:** Active LOW (0=ON, 1=OFF)
- **Timing:** Uses `systick_delay_ms()` for precise delays

The `limit_active` variable serves dual purpose:
1. Controls LED behavior based on test results
2. Will be used for future state machine implementation

This pattern allows quick visual verification that:
- The firmware is running
- The clock is configured correctly
- GPIO initialization succeeded
- SysTick timer is working correctly
- All enabled hardware tests are passing

### GPIO Configuration (src/gpio.c, inc/gpio.h)

**Complete GPIO management for 3-axis CNC controller:**

This module handles all GPIO configuration for the CNC controller according to the pinout defined in `doc/pin_out.md`.

**Pin Groups:**
- **Stepper Motors (9 pins):**
  - STEP signals: PA0 (X), PA1 (Y), PB0 (Z) - Will use TIM2/TIM3 for 20kHz PWM
  - DIR signals: PB12 (X), PB13 (Y), PB14 (Z) - Direction control
  - ENABLE signals: PB3 (X), PB4 (Y), PB5 (Z) - Active LOW enable

- **Rotary Encoders (6 pins):**
  - X-axis: PA6/PA7 (TIM3 CH1/CH2)
  - Y-axis: PB6/PB7 (TIM4 CH1/CH2)
  - Z-axis: PA8/PA9 (TIM1 CH1/CH2)

- **Limit Switches (6 pins):**
  - X: PB8 (MIN), PB9 (MAX)
  - Y: PA2 (MIN), PA3 (MAX)
  - Z: PA4 (MIN), PA5 (MAX)
  - All configured with internal pull-up (active LOW switches)

- **Generic I/O (6 pins):**
  - IO_1: PC14, IO_2: PC15, IO_3: PB1
  - IO_4: PA10, IO_5: PA15, IO_6: PB15
  - Configurable for LCD, relays, LEDs, etc.

- **Status LED:**
  - PC13 - Built-in LED, active LOW

**Note:** UART3 pins (PB10=TX, PB11=RX) are configured automatically by `uart_init()` and are NOT managed by the GPIO module to avoid conflicts.

**Key Functions:**
- `gpio_init_all()` - Initialize ALL GPIO (enables clocks + calls all group functions)
- `gpio_init_motors()` - Configure stepper motor pins
- `gpio_init_encoders()` - Configure encoder inputs
- `gpio_init_limit_switches()` - Configure limit switch inputs
- `gpio_init_io()` - Configure generic I/O pins
- `gpio_init_led()` - Configure status LED

**Helper Functions (inline):**
- `gpio_set(port, pin)` - Set pin HIGH
- `gpio_reset(port, pin)` - Set pin LOW
- `gpio_read(port, pin)` - Read pin state
- `gpio_toggle(port, pin)` - Toggle pin state

**Implementation Phases:**
- **Phase 1 (Current):** All pins configured as standard GPIO
  - STEP pins: GPIO output (for manual testing)
  - Encoder pins: GPIO input (for reading)
  - Ready for basic testing and validation

- **Phase 2 (Future):** Hardware peripherals
  - STEP pins: Reconfigure as TIM2/TIM3 PWM outputs
  - Encoder pins: Reconfigure as TIM1/TIM3/TIM4 encoder inputs
  - Limit switches: Add EXTI interrupt support

**Usage Example:**
```c
#include "gpio.h"

int main(void) {
    SystemClock_Config();
    log_init();
    gpio_init_all();  // Initialize all GPIO

    // Enable motor X
    gpio_reset(ENABLE_X_PORT, ENABLE_X_PIN);

    // Generate manual step pulse
    gpio_set(STEP_X_PORT, STEP_X_PIN);
    delay_us(10);
    gpio_reset(STEP_X_PORT, STEP_X_PIN);

    // Read limit switch
    if (gpio_read(X_MIN_PORT, X_MIN_PIN) == 0) {
        log_warning("X MIN limit triggered!");
    }

    // Blink status LED
    gpio_toggle(LED_STATUS_PORT, LED_STATUS_PIN);
}
```

**Important Notes:**
- GPIO clocks (GPIOA/B/C) must be enabled before accessing registers (done automatically in `gpio_init_all()`)
- All ENABLE pins initialized HIGH (motors disabled) for safety
- Many pins are 5V tolerant (see `doc/pin_out.md`)
- STEP pins configured at 50MHz for future PWM use
- Complete pinout documentation available in `doc/pin_out.md`

### SysTick System Timer (src/systick.c, inc/systick.h)

**Hardware timer for system scheduling and timing:**

The SysTick timer is a 24-bit down-counter integrated in the ARM Cortex-M3 core, configured to generate interrupts every 1ms for precise system timing.

**Configuration:**
- **Frequency:** 1000 Hz (1ms per tick)
- **Clock source:** HCLK (72 MHz)
- **Reload value:** 71999 (72000 - 1)
- **Tick counter:** 32-bit unsigned (wraps after ~49.7 days)
- **Interrupt:** `SysTick_Handler()` called every 1ms

**Key Features:**
- **Precise timing:** Hardware-based, independent of CPU load
- **Low power:** Uses `__WFI()` in delays to put CPU in sleep mode
- **Wraparound handling:** Arithmetic handles overflow correctly
- **Thread-safe:** 32-bit reads are atomic on Cortex-M3

**API Functions:**

*Initialization:*
- `systick_init(sysclk_hz)` - Initialize SysTick (call after clock config)
- `systick_disable()` - Disable SysTick timer

*Tick Counter:*
- `systick_get_tick()` - Get current tick count (milliseconds since init)
- `systick_millis()` - Alias for get_tick()

*Delay Functions:*
- `systick_delay_ms(ms)` - Blocking delay with WFI for power saving
- `systick_timeout(start, ms)` - Non-blocking timeout check

*Statistics:*
- `systick_get_interrupt_count()` - Total interrupts processed
- `systick_get_hw_counter()` - Raw 24-bit hardware counter (sub-ms resolution)

*Utilities (inline):*
- `systick_ms_to_ticks()`, `systick_ticks_to_ms()`, `systick_seconds_to_ticks()`

**Usage Examples:**

```c
// Blocking delay
systick_delay_ms(500);  // Wait 500ms (CPU sleeps with WFI)

// Non-blocking timeout
u32 start = systick_get_tick();
while (!systick_timeout(start, 1000)) {
    // Do something for max 1 second
    if (task_done()) break;
}

// Measure elapsed time
u32 t1 = systick_get_tick();
do_something();
u32 elapsed = systick_get_tick() - t1;  // Handles wraparound correctly
log_info("Elapsed: %u ms", elapsed);

// Get system uptime
u32 uptime_ms = systick_millis();
u32 uptime_sec = uptime_ms / 1000;
```

**Implementation Details:**
- **Interrupt priority:** Default (can be configured if needed)
- **ISR overhead:** Minimal (~10 CPU cycles: increment counters only)
- **No logging in ISR:** Prevents deadlock with log system
- **Registers:** Uses definitions from `stm32f103_regs.h` (SYSTICK_CTRL, SYSTICK_LOAD, SYSTICK_VAL)

**Important Notes:**
- Must be initialized AFTER `SystemClock_Config()` (needs SYSCLK frequency)
- Replaces busy-wait loops with efficient timer-based delays
- Tick counter wraps at 2^32 - 1, but subtraction handles this correctly
- For sub-millisecond timing, use `systick_get_hw_counter()` (13.9ns resolution @ 72MHz)

### Hardware Test Framework (src/test.c, inc/test.h)

**Modular hardware validation system with conditional compilation:**

The test framework provides automated validation of CNC controller hardware through loopback and functional tests. Tests can be individually enabled/disabled via macros in `test.h` for zero overhead when not needed.

**Architecture:**

*Conditional Compilation Pattern:*
Each test has a corresponding `ENABLE_TEST_*` macro and `EVAL_TEST_*()` wrapper:

```c
// In test.h
#define ENABLE_TEST_STEP_IO    1  // Enable test

#if ENABLE_TEST_STEP_IO
#define EVAL_TEST_STEP_IO(MSG)  MSG  // Include code
bool test_step_io(void);
#else
#define EVAL_TEST_STEP_IO(MSG)  /* Remove code */
#endif
```

Usage in main loop:
```c
int limit_active = 0;
EVAL_TEST_STEP_IO(limit_active = test_step_io();)  // Entire statement removed if disabled
```

**Benefits:**
- Clean code without scattered `#if/#endif` blocks
- Zero code/RAM overhead when disabled
- Easy to enable/disable tests for different build configurations
- Statement entirely removed from preprocessed code (not just skipped)

**Available Tests:**

**1. test_limit() - Limit Switch Functional Test**
- **Macro:** `ENABLE_TEST_LIMIT` (currently disabled, test completed)
- **Purpose:** Verify all 6 limit switches (X/Y/Z MIN/MAX) respond correctly
- **Method:**
  - Monitors all limit switch inputs continuously
  - Detects and logs edge transitions (pressed/released)
  - Uses internal pull-up resistors (active LOW)
- **Pass criteria:** All switches toggle correctly when pressed
- **Failure indication:** No failures (logs events only)
- **Returns:** `true` if any switch is currently pressed (stops LED)

**2. test_step_io() - STEP Pin Loopback Test**
- **Macro:** `ENABLE_TEST_STEP_IO` (currently enabled)
- **Purpose:** Verify STEP output pins can drive signals correctly
- **Method:** Loopback testing with external jumper wires
  - Toggles STEP_X/Y/Z outputs every 500ms
  - Reads back on X/Y/Z_MIN inputs
  - Accounts for pull-up inversion: `expected = !output`
- **Hardware setup required:**
  - PA0 (STEP_X) → PB8 (X_MIN)
  - PA1 (STEP_Y) → PA2 (Y_MIN)
  - PB0 (STEP_Z) → PA4 (Z_MIN)
- **Pass criteria:** Input reads match inverted output state
- **Failure indication:** Logs error, stops LED
- **Returns:** `true` if mismatch detected

**3. test_dir_io() - DIR Pin Loopback Test**
- **Macro:** `ENABLE_TEST_DIR_IO` (currently disabled)
- **Purpose:** Verify DIR output pins can drive signals correctly
- **Method:** Same as test_step_io() but for DIR pins
- **Hardware setup required:**
  - PB12 (DIR_X) → PB9 (X_MAX)
  - PB13 (DIR_Y) → PA3 (Y_MAX)
  - PB14 (DIR_Z) → PA5 (Z_MAX)
- **Pass criteria:** Input reads match inverted output state
- **Failure indication:** Logs error, stops LED
- **Returns:** `true` if mismatch detected

**4. test_enable_io() - ENABLE Pin Loopback Test**
- **Macro:** `ENABLE_TEST_ENABLE_IO` (currently disabled)
- **Purpose:** Verify ENABLE output pins can drive signals correctly
- **Method:** Same as test_step_io() but for ENABLE pins
- **Hardware setup required:**
  - PB3 (ENABLE_X) → PA6 (ENC_X_A)
  - PB4 (ENABLE_Y) → PB6 (ENC_Y_A)
  - PB5 (ENABLE_Z) → PA8 (ENC_Z_A)
- **Pass criteria:** Input reads match inverted output state
- **Failure indication:** Logs error, stops LED
- **Returns:** `true` if mismatch detected

**Test Implementation Details:**

All loopback tests follow this pattern:
```c
bool test_step_io(void) {
    static bool first_run = true;
    static u32 last_toggle_time = 0;
    static bool output_state = false;
    bool mismatch_detected = false;

    // Print instructions on first run
    if (first_run) {
        log_info("=== Test STEP I/O Attivo ===");
        log_info("Collega: PA0->PB8, PA1->PA2, PB0->PA4");
        first_run = false;
    }

    // Toggle outputs every 500ms
    if (systick_timeout(last_toggle_time, 500)) {
        output_state = !output_state;
        // Set/reset GPIO outputs...
        last_toggle_time = systick_get_tick();
    }

    systick_delay_ms(10);  // Signal stabilization

    // Read inputs and verify (accounting for pull-up inversion)
    bool expected_input = !output_state;
    if (input_read != expected_input) {
        log_error("Mismatch detected!");
        mismatch_detected = true;
    }

    return mismatch_detected;
}
```

**Key Features:**
- **Edge detection:** test_limit() only logs transitions (not continuous state)
- **Debouncing:** 10ms delay after output change for signal stabilization
- **Pull-up awareness:** All inputs have internal pull-ups, so HIGH output → LOW input when connected
- **Non-blocking:** Uses `systick_timeout()` for timing without blocking
- **First-run init:** Prints instructions once at test start
- **Visual feedback:** LED stops blinking when test fails/detects active condition

**Usage Example:**

```c
// Enable desired tests in test.h
#define ENABLE_TEST_STEP_IO    1
#define ENABLE_TEST_DIR_IO     0
#define ENABLE_TEST_ENABLE_IO  0

// In main.c
int limit_active = 0;
while(1) {
    EVAL_TEST_STEP_IO(limit_active = test_step_io();)

    if (!limit_active) {
        // Normal LED pattern
    } else {
        // LED stays on - test failed or condition detected
    }
}
```

**Testing Workflow:**

1. Enable one test at a time in `test.h`
2. Build and flash firmware
3. Connect required jumper wires for loopback
4. Power on - LED should blink normally if test passes
5. Monitor ITM/UART console for detailed status
6. If LED stops: check console for error messages
7. Once test passes, disable it and move to next test

**Important Notes:**
- Only enable ONE loopback test at a time (pins conflict)
- Loopback tests require external wire connections
- Pull-up resistors are active on all input pins (~40kΩ)
- Test functions use static variables to maintain state between calls
- All tests are non-blocking and designed for continuous execution in main loop
- LED behavior provides instant visual feedback without console

### ITM Console Support (src/itm.c, inc/itm.h)

**Real-time console output via ST-Link SWO (Serial Wire Output):**

The ITM (Instrumentation Trace Macrocell) module enables printf-style debugging without using UART pins:

**Features:**
- Zero GPIO overhead (uses SWO pin on debugger)
- High speed output (up to 2 MHz configured, adjustable)
- No CPU blocking (hardware FIFO)
- Works with ST-Link V2 and above

**Key Functions:**
- `itm_init()` - Initialize ITM (call after `SystemClock_Config()`)
- `itm_finit()` - Deinitialize and disable ITM
- `itm_write(str)` - Send string to ITM console
- `itm_putchar(ch)` - Send single character
- `log_via_itm()` - **Dump entire circular log buffer to ITM console**

**Important Configuration:**
- Clock settings in `itm.h`:
  - `ITM_SYSCLK_HZ`: Must match actual SYSCLK (72 MHz for this project)
  - `ITM_SWO_FREQ_HZ`: SWO baud rate (2 MHz default, can reduce to 500 kHz for stability)
- Debugger must be configured with matching clock values
- See extensive notes in `itm.h` for STM32CubeIDE/OpenOCD/ST-Link Utility setup

**Usage Example:**
```c
SystemClock_Config();
log_init();
itm_init();

// Direct ITM output
itm_write("Hello from ITM!\n");

// Log to circular buffer
log_info("System started");
log_debug("Clock: 72 MHz");

// Dump all logs to ITM console
itm_write("\n=== Log Dump ===\n");
int count = log_via_itm();  // Drains circular buffer to ITM
```

**Viewing ITM Output:**
- **STM32CubeIDE:** Window → Show View → SWV → SWV ITM Data Console (configure Core Clock: 72 MHz, SWO Clock: 500 kHz)
- **OpenOCD:** `monitor tpiu config internal - uart off 72000000`
- **ST-Link Utility:** ST-Link → Printf via SWO viewer

**Hardware Requirements:**
- Debugger with SWO pin (J-Link, ST-Link V2.1, or ST-Link V2 with 10-pin connector)
- SWO connection: Debugger SWO → Blue Pill PB3

### UART Console Support (src/uart.c, inc/uart.h)

**Serial console output without debugger dependency:**

Simple UART driver for debug output via USB-Serial adapter.

**Current Configuration:**
- **USART3** (APB1 @ 36 MHz)
- **Pins:** PB10 (TX), PB11 (RX)
- **Baud rate:** 115200
- **Format:** 8N1 (8 data bits, no parity, 1 stop bit)
- **Mode:** TX + RX enabled

**Features:**
- Works without debugger attached
- Configurable to use USART1/2/3 (see `uart.h`)
- Helper functions for integers and hex output
- Compatible with any USB-UART adapter
- GPIO pins configured automatically by `uart_init()`

**Key Functions:**
- `uart_init()` - Initialize UART3 at 115200,8,N,1 (configures GPIO pins internally)
- `uart_finit()` - Deinitialize UART
- `uart_write(str)` - Send string to UART
- `uart_putchar(ch)` - Send single character
- `log_via_uart()` - **Dump entire circular log buffer to UART** ⭐
- `uart_write_int(num)` - Write integer (helper)
- `uart_write_hex(num)` - Write hexadecimal (helper)

**Hardware Connection:**
```
USB-UART Adapter    →  STM32F103 (Blue Pill)
GND                 →  GND
RX                  →  PB10 (USART3 TX)
TX                  →  PB11 (USART3 RX)
```

**Viewing UART Output:**
```bash
# Linux (minicom)
sudo minicom -D /dev/ttyUSB0 -b 115200

# Linux (screen)
screen /dev/ttyUSB0 115200

# Windows (PuTTY)
# Serial line: COM3, Speed: 115200, 8N1
```

**Configuration:**
- Current: USART3, TX=PB10, RX=PB11, 115200 baud
- Can be changed in `inc/uart.h`:
  - `UART_USE_USART` (1, 2, or 3) - Currently set to 3
  - `UART_BAUDRATE` (9600 to 921600)
  - `UART_ENABLE_RX` (0=TX only, 1=TX+RX)

**Usage Example:**
```c
// GPIO must be initialized first (done in main)
gpio_init_all();

// UART configures its own pins (PB10/PB11) automatically
uart_init();
uart_write("System started\r\n");
uart_write("Temperature: ");
uart_write_int(temperature);
uart_write("\r\n");

// Dump all logs to UART
log_via_uart();
```

### Utilities (src/utils.c, inc/utils.h)

Provides lightweight printf-style formatting for the logging system:
- `mini_vsnprintf()` - Variable argument formatted string printing (used by log_error, log_warning, etc.)
- Supports format specifiers: %d, %u, %x, %X, %b, %c, %s, %%
- No floating point support (to minimize code size)

### Register Definitions (inc/stm32f103_regs.h)

Comprehensive register definitions for STM32F103C8T6:
- **Memory-mapped peripheral access** via volatile pointers
- Base addresses for all peripherals (APB1, APB2, AHB buses)
- Bit definitions for all common registers (RCC, GPIO, USART, Flash, SysTick, NVIC, etc.)
- **Emulation support:** Can redefine `PERIPH_BASE` before including to map registers to different memory

### Common Definitions (inc/common.h)

Project-wide common header (include in all files):
- Standard library includes (stdint.h, stdbool.h, stddef.h, string.h)
- Type aliases: u8/i8, u16/i16, u32/i32, u64/i64
- Interrupt control: `__disable_irq()`, `__enable_irq()`, `__WFI()`, `__WFE()`
- Utility macros: `BIT()`, `SET_BIT()`, `CLR_BIT()`, `MIN()`, `MAX()`, `ARRAY_SIZE()`
- Includes stm32f103_regs.h

## Hardware Abstraction

This is a **register-level bare-metal implementation** with no HAL/SPL:
- All peripherals accessed via direct memory-mapped register writes
- Register addresses and bit masks defined in `inc/stm32f103_regs.h`
- Common typedefs (u8, u16, u32, i8, i16, i32) and utility macros in `inc/common.h`
- Interrupt control macros (`__disable_irq`, `__enable_irq`) defined in `inc/common.h`

To add new peripheral support:
1. Add register definitions to `inc/stm32f103_regs.h` if not already present
2. Create new .c/.h files in src/inc directories
3. Add source file to `PROJECT_SOURCES` in CMakeLists.txt (lines 48-57)
4. Write initialization functions that configure registers in correct sequence
5. Follow existing pattern in `src/clock.c` with extensive comments

## Common Development Tasks

### Viewing Logs

**Option 1: Via ITM Console (Real-time, Recommended for Development)**

The ITM console provides real-time log output without using UART pins:

1. **Initialize ITM in your code:**
   ```c
   itm_init();  // After SystemClock_Config()
   ```

2. **Configure STM32CubeIDE debugger:**
   - Run → Debug Configurations → [Your Config] → Debugger tab
   - Serial Wire Viewer (SWV) section:
     - Enable: ✓
     - Core Clock: 72.0 MHz
     - SWO Clock: 2000 kHz (or 500 kHz for better stability)
   - Apply

3. **View output during debugging:**
   - Window → Show View → SWV → SWV ITM Data Console
   - Click "Configure Trace" button, enable Port 0
   - Click "Start Trace"

4. **Dump circular log buffer to ITM:**
   ```c
   log_via_itm();  // Sends all buffered logs to ITM console
   ```

**Option 2: Inspect Log Buffer Directly in Memory**

The circular log buffer can be inspected in memory when debugger is attached:

```
# GDB
x/2048xb 0x20004800

# OpenOCD
mdw 0x20004800 512
```

### Flashing to Hardware

The project generates `.elf` files. Use STM32CubeProgrammer, OpenOCD, or st-flash:

```bash
# With OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program Debug/f103.elf verify reset exit"

# With st-flash (requires .bin conversion)
arm-none-eabi-objcopy -O binary Debug/f103.elf Debug/f103.bin
st-flash write Debug/f103.bin 0x8000000
```

### Debugging Clock Issues

If system crashes after `SystemClock_Config()`:
1. Verify HSE crystal is present (8 MHz)
2. Check capacitor values (typically 20-30 pF for STM32F103)
3. Ensure Flash wait states are set before PLL switch (src/clock.c:82-87)
4. Check return code: `CLOCK_ERROR_HSE` (crystal issue) or `CLOCK_ERROR_PLL` (PLL lock failure)

### Quick Build and Flash

For rapid development, use these convenience scripts:
```bash
# Build
./do

# Clean and rebuild
./rebuild

# Clean all build artifacts
./distclean
```

Note: These scripts are wrappers around the CMake commands shown in the Build Commands section.

### Compiler Flags

Defined in `CMakeLists.txt` line 90-91:
- `-mcpu=Cortex-M3` - Target Cortex-M3 core
- `-mthumb` - Use Thumb instruction set
- `-std=gnu11` - C11 with GNU extensions
- `--specs=nano.specs` - Newlib-nano (reduced libc)
- `--specs=nosys.specs` - No system calls (using custom syscalls.c)
- `-Wall -Werror` - All warnings enabled and treated as errors

## Code Style Conventions

Based on existing code:
- **Comments:** Extensive block comments explaining hardware behavior and register sequences (Italian/English)
- **Register access:** Use `volatile uint32_t *` with direct address casting
- **Constants:** All caps with module prefix (e.g., `RCC_CR_HSEON`, `LOG_BUFFER_SIZE`)
- **Error handling:** Integer return codes with descriptive constants (e.g., `CLOCK_OK`, `LOG_ERROR_INVALID`)
- **Documentation:** Doxygen-style `@brief`, `@details`, `@param`, `@retval`, `@note` comments

## Limitations and Constraints

1. **No HAL/SPL:** All peripheral access is register-level
2. **No RTOS:** Bare-metal single-threaded (except ISRs)
3. **Semihosting:** Uses `syscalls.c` for printf support - requires debugger connection for output
4. **APB1 maximum:** 36 MHz hard limit (do not change prescaler)
5. **Log buffer:** 2 KB limit, oldest messages auto-deleted on overflow
6. **Thread safety:** Logging disables all interrupts during write/read (can add latency)
7. **Logging can be disabled:** Set `ENABLE_LOG` to 0 in `inc/log.h` to disable all logging macros

## Adding Peripherals

When implementing new peripherals (UART, SPI, I2C, timers, etc.):

1. Check bus assignment:
   - APB1 (36 MHz): I2C1/2, UART2-5, TIM2-7, SPI2
   - APB2 (72 MHz): USART1, SPI1, TIM1, ADC1/2, GPIOA-E
2. Enable peripheral clock in RCC (use `RCC_APBxENR` register)
3. Configure GPIO alternate functions if needed (see `src/gpio.c` for examples)
4. Write initialization following clock.c pattern (detailed comments, step-by-step)
5. Remember timer clocks on APB1/APB2 are automatically doubled when prescaler ≠ 1

**Note:** Basic GPIO configuration is already implemented in `src/gpio.c` for the CNC controller. See the "GPIO Configuration" section above for details on available pins and their current assignments.

## Known Issues

None currently documented.
