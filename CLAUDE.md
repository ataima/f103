# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a bare-metal embedded project for the **STM32F103C8T6** microcontroller (commonly known as "Blue Pill"). The project is configured to run at maximum speed (72 MHz) using an external 8 MHz crystal with PLL multiplier.

**Target Hardware:**
- MCU: STM32F103C8T6 (ARM Cortex-M3)
- Flash: 64 KB @ 0x08000000
- RAM: 20 KB total (18 KB general + 2 KB dedicated log buffer)
- Clock: HSE 8 MHz → PLL ×9 → 72 MHz SYSCLK

## Build System

This project uses **CMake** with a custom toolchain file for ARM GCC.

### Build Commands

```bash
# Debug build (recommended for development)
cmake -DCMAKE_TOOLCHAIN_FILE=cubeide-gcc.cmake -S ./ -B Debug -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug
make -C Debug -j

# Release build
cmake -DCMAKE_TOOLCHAIN_FILE=cubeide-gcc.cmake -S ./ -B Release -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
make -C Release -j

# Clean build
rm -rf Debug && cmake -DCMAKE_TOOLCHAIN_FILE=cubeide-gcc.cmake -S ./ -B Debug -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug && make -C Debug -j
```

The output is `Debug/f103.elf` or `Release/f103.elf`.

### Adding New Source Files

Edit `CMakeLists.txt` and add files to the `PROJECT_SOURCES` variable (lines 48-58).

## Architecture

### Memory Layout (STM32F103C8TX_FLASH.ld)

The linker script defines a **custom memory partition** that separates logging RAM from general RAM:

- **RAM (0x20000000 - 0x20004800):** 18 KB for general use (.data, .bss, heap, stack)
- **RAM_LOG (0x20004800 - 0x20004FFF):** 2 KB dedicated log buffer (`.log_buffer` section)
- **FLASH (0x08000000 - 0x0800FFFF):** 64 KB for code and constants

The log buffer uses `(NOLOAD)` to preserve contents across soft resets for post-mortem debugging.

### Clock Configuration (Sources/clock.c/h)

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

### Logging System (Sources/log.c/h)

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
- Convenience macros with prefixes:
  - `log_error("msg")` → `[ER]  msg`
  - `log_warning("msg")` → `[WRN] msg`
  - `log_info("msg")` → `[INFO]msg`
  - `log_debug("msg")` → `[DBG] msg`
- Statistics: `log_get_count()`, `log_get_used_bytes()`, `log_get_free_bytes()`
- `log_clear()` - Empty buffer
- `log_get_buffer_address()` - Get RAM address (0x20004800) for debugger inspection

**Important:**
- Messages limited to `LOG_MAX_MESSAGE_SIZE` (128 bytes)
- Lock acquisition can fail (returns `LOG_ERROR_LOCKED`) if buffer is busy
- Buffer survives soft resets due to `(NOLOAD)` linker attribute

### Main Application (Sources/main.c)

Initialization sequence:
1. `SystemClock_Config()` - Configure clock to 72 MHz
2. `log_init()` - Initialize logging system
3. Print clock frequencies via `printf()` (requires `syscalls.c` for semihosting)
4. Demo logging system with all levels
5. Infinite main loop with periodic heartbeat

## Hardware Abstraction

This is a **register-level bare-metal implementation** with no HAL/SPL:
- All peripherals accessed via direct memory-mapped register writes
- Register addresses defined in `clock.c` (e.g., `RCC_BASE`, `FLASH_BASE`)
- Bit masks defined as constants (e.g., `RCC_CR_HSEON`, `RCC_CFGR_SW_PLL`)

To add new peripheral support:
1. Define base address and register offsets
2. Define bit masks for register fields
3. Write initialization functions that configure registers in correct sequence
4. Follow existing pattern in `clock.c` with extensive comments

## Common Development Tasks

### Viewing Logs in Debugger

The log buffer can be inspected directly in memory:

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
3. Ensure Flash wait states are set before PLL switch (line 196-200 in clock.c)
4. Check return code: `CLOCK_ERROR_HSE` (crystal issue) or `CLOCK_ERROR_PLL` (PLL lock failure)

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
3. **Printf via semihosting:** Uses `syscalls.c` - requires debugger connection for output
4. **APB1 maximum:** 36 MHz hard limit (do not change prescaler)
5. **Log buffer:** 2 KB limit, oldest messages auto-deleted on overflow
6. **Thread safety:** Logging disables all interrupts during write/read (can add latency)

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
