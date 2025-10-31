# CLAUDE.md

This file provides guidance to Claude Code when working with this STM32F103C8T6 bare-metal embedded project.

## Project Overview

**Hardware:**
- MCU: STM32F103C8T6 "Blue Pill" (ARM Cortex-M3)
- Flash: 64 KB @ 0x08000000
- RAM: 20 KB (18 KB general + 2 KB log buffer @ 0x20004800)
- Clock: HSE 8 MHz → PLL ×9 → 72 MHz SYSCLK

**Architecture:**
- Bare-metal register-level (no HAL/SPL)
- All peripherals in `inc/stm32f103_regs.h`
- Common types/macros in `inc/common.h`

## Build System

**Quick Build:**
```bash
source source_this        # Setup toolchain PATH (once per session)
make -C Debug -j          # Build
./do                      # Alternative (auto-sources environment)
./rebuild                 # Clean and rebuild
```

**Adding Source Files:**
Edit `CMakeLists.txt` → `PROJECT_SOURCES` variable (lines 48-62)

**Current Binary Sizes (Debug):**
- Flash: ~22.7 KB (35.0% of 64 KB)
- RAM: ~2.2 KB (10.5% of 20 KB - excludes 2KB log buffer)

## Memory Layout (STM32F103C8TX_FLASH.ld)

- **RAM (0x20000000-0x20003FFF):** 16 KB general (.data, .bss, heap, stack)
- **RAM_LOG (0x20004800-0x20004FFF):** 2 KB log buffer (`.log_buffer` section, NOLOAD)
- **FLASH (0x08000000-0x0800FFFF):** 64 KB code/constants

## Module Reference

### Clock (src/clock.c, inc/clock.h)
Configures system to 72 MHz via HSE + PLL. **Critical constraints:**
- APB1 max 36 MHz, APB2 max 72 MHz
- Flash 2 wait states required @ 72 MHz
- Sequence: HSE → Flash WS → Prescalers → PLL → SYSCLK switch

**API:** `SystemClock_Config()`, `SystemClock_GetFreq()`, `SystemClock_GetHCLK/PCLK1/PCLK2()`

### Logging (src/log.c, inc/log.h)
Thread-safe circular buffer (2 KB) in dedicated RAM. Format: `[LENGTH(2B)][MESSAGE][\0]`

**API:**
- Init: `log_init()`
- Write: `log_write(msg)`, `log_error/warning/info/debug(fmt, ...)`
- Read: `log_read(dest)` (POP oldest)
- Stats: `log_get_count/used_bytes/free_bytes()`
- Clear: `log_clear()`
- Address: `log_get_buffer_address()` → 0x20004800

**Notes:**
- Max message 128 bytes (`LOG_MAX_MESSAGE_SIZE`)
- Auto-overflow: removes oldest when full
- Lock via `__disable_irq/__enable_irq`
- Buffer survives soft reset (NOLOAD)
- Disable all logging: set `ENABLE_LOG=0` in log.h

### GPIO (src/gpio.c, inc/gpio.h)
Complete CNC controller pinout (see `doc/pin_out.md`):
- **Motors (9):** STEP (PA0/PA1/PB0), DIR (PB12/13/14), ENABLE (PB3/4/5)
- **Encoders (6):** X(PA6/7), Y(PB6/7), Z(PA8/9) → TIM3/4/1
- **Limits (6):** X(PB8/9), Y(PA2/3), Z(PA4/5) - pull-up, active LOW
- **Generic I/O (6):** PC14/15, PB1, PA10, PA15, PB15
- **LED:** PC13 (active LOW)

**API:** `gpio_init_all()`, `gpio_set/reset/read/toggle(port, pin)`

**Note:** UART3 pins (PB10=TX, PB11=RX) managed by `uart_init()`, NOT gpio module.

### SysTick (src/systick.c, inc/systick.h)
1ms system timer @ 72 MHz (1000 Hz), 32-bit tick counter (wraps ~49.7 days).

**API:**
- Init: `systick_init(sysclk_hz)`
- Time: `systick_get_tick()`, `systick_millis()`
- Delay: `systick_delay_ms(ms)` (blocking, uses WFI), `systick_timeout(start, ms)` (non-blocking)
- Callback: `systick_register_encoder_callback(fn)` - 50Hz callback for CNC encoders

### UART (src/uart.c, inc/uart.h)
Serial console without debugger. **Current:** USART3 @ 115200,8,N,1, pins PB10=TX, PB11=RX.

**API:** `uart_init()`, `uart_write(str)`, `uart_putchar(ch)`, `log_via_uart()` (dumps log buffer)

**Config (uart.h):** `UART_USE_USART` (1/2/3), `UART_BAUDRATE`, `UART_ENABLE_RX`

**Connection:** USB-UART adapter: GND-GND, RX→PB10, TX→PB11

### ITM Console (src/itm.c, inc/itm.h)
Real-time debug via SWO (debugger only), no GPIO pins used.

**API:** `itm_init()`, `itm_write(str)`, `log_via_itm()` (dumps log buffer)

**Config (itm.h):** `ITM_SYSCLK_HZ=72000000`, `ITM_SWO_FREQ_HZ=2000000` (or 500kHz for stability)

**CubeIDE Setup:** Window → SWV ITM Data Console, Core Clock: 72 MHz, SWO Clock: 500 kHz

### Encoder (src/encoder.c, inc/encoder.h)
Hardware quadrature decoding via TIM1/3/4 (×4 resolution, 16-bit counters).

**API:** `encoder_init()`, `encoder_read_x/y/z()` (read + reset), `encoder_get_count_x/y/z()` (read only)

**Note:** Called automatically @ 50Hz by CNC system via SysTick callback.

### CNC State Machine (src/cnc.c, inc/cnc.h)
3-axis controller with homing, limit switch EXTI, emergency retraction, and real-time movement via TIM2.

**States:** `ST_IDLE`, `ST_RUNNING`, `ST_HOMING`, `ST_TEST`, `ST_EMERGENCY_STOP`, `ST_ALARM_X/Y/Z`

**Init Sequence:** `cnc_init()` → EXTI config → TIM2 setup → HOMING (Z→Y→X) → Encoder init → Callback

**API:**
- Init: `cnc_init()`
- Homing: `cnc_home()` (all axes), `cnc_home_x/y/z()` (single)
- Position: `cnc_get_position_x/y/z()` (absolute encoder counts)
- Emergency: `cnc_process_emergency()` (call in main loop), `cnc_has_emergency()`
- State: `cnc_get_state()`, `cnc_set_state()`
- **Real-time Movement:**
  - `cnc_move_init_rt(state, start, end, min_hz, max_hz)` - Setup movimento (~104 bytes)
  - `cnc_move_step_rt(state)` - Calcola step Bresenham (returns axis mask: bit0=X, bit1=Y, bit2=Z)
  - `cnc_move_get_speed_rt(state)` - Velocità corrente trapezoidale (Hz)
  - `cnc_move_execute_rt(state)` - Esegue movimento completo (blocking)

**Real-time Movement Features:**
- **Bresenham 3D incrementale:** Step simultanei multi-asse (no segmentazione traiettoria)
- **Profilo trapezoidale on-the-fly:** 25% accel, 50% const, 25% decel
- **RAM efficiente:** ~104 bytes vs 12 KB array pre-calcolato
- **TIM2 ISR:** Genera STEP hardware, aggiorna velocità dinamicamente

**Emergency Flow:** EXTI ISR → disable axes → queue → main loop `cnc_process_emergency()` → retract (invert DIR, TIM2 STEP @ 1kHz) → release limit → idle

**Priority:** EXTI=0 (highest), SysTick=default, TIM2=2

### Hardware Tests (src/test.c, inc/test.h)
Conditional compilation framework. Enable/disable in `test.h` via `ENABLE_TEST_*` macros.

**Tests:**
1. `test_limit()` - Monitor all 6 limit switches (edge detection)
2. `test_step_io()` - Loopback: STEP pins → MIN limits (requires jumpers)
3. `test_dir_io()` - Loopback: DIR pins → MAX limits (requires jumpers)
4. `test_enable_io()` - Loopback: ENABLE pins → Encoder A pins (requires jumpers)

**Usage:** Enable ONE loopback test at a time (pins conflict). LED stops blinking on failure.

### Utils (src/utils.c, inc/utils.h)
Lightweight printf for logging: `mini_vsnprintf()` supports %d, %u, %x, %X, %b, %c, %s, %%.

## Main Application (src/main.c)

**Init Sequence:**
1. `SystemClock_Config()` → 72 MHz
2. Delay for SWV sync
3. `log_init()`
4. `itm_init()` (if `ENABLE_ITM` set)
5. `gpio_init_all()`
6. `systick_init(72000000)`
7. `uart_init()`
8. Hardware tests (if enabled)

**Main Loop:** Executes tests, LED blink pattern (1.2s cycle), emergency check.

**LED (PC13):** Normal = recognizable blink, Failure = stays ON.

## Common Tasks

### View Logs
**ITM (debugger):** `itm_init()` → CubeIDE SWV console → `log_via_itm()` dumps buffer
**UART (standalone):** `uart_init()` → serial terminal @ 115200 → `log_via_uart()` dumps buffer
**Memory (debugger):** GDB `x/2048xb 0x20004800` or OpenOCD `mdw 0x20004800 512`

### Flash Hardware
```bash
# OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program Debug/f103.elf verify reset exit"

# st-flash
arm-none-eabi-objcopy -O binary Debug/f103.elf Debug/f103.bin
st-flash write Debug/f103.bin 0x8000000
```

### Clock Issues
If crash after `SystemClock_Config()`:
1. Check HSE crystal (8 MHz) + capacitors (20-30 pF)
2. Verify Flash wait states before PLL switch (src/clock.c:82-87)
3. Check return code: `CLOCK_ERROR_HSE` or `CLOCK_ERROR_PLL`

### Add Peripheral
1. Check bus: APB1 (36 MHz) or APB2 (72 MHz)
2. Enable clock: `RCC_APBxENR |= peripheral_bit`
3. Config GPIO alt functions (if needed)
4. Follow `src/clock.c` pattern: detailed comments, step-by-step
5. Add source to `CMakeLists.txt` → `PROJECT_SOURCES`

## Code Style

- **Comments:** Doxygen-style (@brief, @param, @retval), extensive hardware explanations (IT/EN)
- **Registers:** Direct `volatile uint32_t *` access
- **Constants:** ALL_CAPS with prefix (e.g., `LOG_MAX_MESSAGE_SIZE`)
- **Errors:** Integer codes with descriptive constants (e.g., `UART_OK`, `LOG_ERROR_INVALID`)

## Constraints

1. **No HAL/SPL** - Register-level only
2. **No RTOS** - Bare-metal + ISRs
3. **APB1 max 36 MHz** - Hardware limit
4. **Log 2 KB limit** - Auto-overflow
5. **Thread safety** - Logging disables all IRQs (adds latency)
