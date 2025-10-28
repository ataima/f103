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
│   └── utils.h                # Utility functions
├── src/                        # Source files
│   ├── main.c                 # Main application
│   ├── clock.c                # Clock configuration
│   ├── log.c                  # Logging system
│   ├── itm.c                  # ITM console support
│   ├── uart.c                 # UART console support
│   ├── utils.c                # Utility functions
│   ├── syscalls.c             # Semihosting support
│   └── sysmem.c               # Memory management
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

**Typical binary sizes:**
- Flash (text): ~8 KB (code)
- RAM (bss): ~4 KB (includes 2 KB log buffer)

### Adding New Source Files

Edit `CMakeLists.txt` and add files to the `PROJECT_SOURCES` variable (lines 48-58).

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

Initialization sequence:
1. `SystemClock_Config()` - Configure clock to 72 MHz
2. `log_init()` - Initialize logging system
3. Log startup messages using the logging system
4. Infinite main loop (currently idle)

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

Simple UART driver for debug output via USB-Serial adapter:

**Features:**
- Works without debugger attached
- Configurable baud rate (default: 115200)
- TX-only mode (RX optional, currently disabled)
- Helper functions for integers and hex output
- Compatible with any USB-UART adapter

**Key Functions:**
- `uart_init()` - Initialize UART1 at 115200,8,N,1
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
RX                  →  PA9 (USART1 TX)
(TX)                →  (PA10 - USART1 RX, optional)
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
- Default: USART1, TX=PA9, 115200 baud
- Can be changed in `inc/uart.h`:
  - `UART_USE_USART` (1, 2, or 3)
  - `UART_BAUDRATE` (9600 to 921600)
  - `UART_ENABLE_RX` (0=TX only, 1=TX+RX)

**Usage Example:**
```c
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

When implementing new peripherals (UART, SPI, I2C, GPIO, etc.):

1. Check bus assignment:
   - APB1 (36 MHz): I2C1/2, UART2-5, TIM2-7, SPI2
   - APB2 (72 MHz): USART1, SPI1, TIM1, ADC1/2, GPIOA-E
2. Enable peripheral clock in RCC (define `RCC_APBxENR` register)
3. Configure GPIO alternate functions if needed
4. Write initialization following clock.c pattern (detailed comments, step-by-step)
5. Remember timer clocks on APB1/APB2 are automatically doubled when prescaler ≠ 1

## Known Issues

None currently documented.
