# CLAUDE.md — NuttX 12.5.1 Driver Development

## Session Preferences

- **Token usage**: The user wants token read/write counts and remaining context shown at the end of each response. Note: Claude Code does not have access to this information — it cannot be fulfilled, but the preference is recorded here.

## Project Overview

NuttX RTOS v12.5.1 — learning environment for embedded driver development.
Target board: **STM32F3Discovery** (STM32F303VCT6, ARM Cortex-M4).
Active work branch: `lsm303dlhc` — implementing the LSM303DLHC IMU sensor driver (I2C: accelerometer + magnetometer).

## Build & Flash

```bash
# Configure (from nuttxspace/)
cd nuttxspace && ./tools/configure.sh stm32f3discovery:lsm303dlhc

# Build
make -j$(nproc)

# Flash (ST-LINK)
openocd -f interface/stlink.cfg -f target/stm32f3x.cfg \
  -c "program nuttx.bin verify reset exit 0x08000000"
```

Previous confirmed-working build artifacts are stored in `_builds/stm32f3/lsm303dlhc/`.

## Connect to NuttShell

```bash
screen /dev/ttyACM0 115200
```

SYSLOG is routed to USART1 (ST-LINK VCP). The USER-USB port is not configured.
Serial Monitor in VS Code only shows output — use `screen` for interactive NSH.
ITM is not used (conflict with GDB).

## Project Layout (relevant paths)

```
boards/arm/stm32/stm32f3discovery/
  src/stm32_bringup.c        # Driver registration entry point
  src/stm32f3discovery.h     # GPIO/peripheral definitions, HAVE_RTC_DRIVER guard
  configs/lsm303dlhc/        # Active defconfig
  configs/alarm_rtc/         # RTC alarm config

drivers/
  sensors/lsm303dlhc.c       # Generic upper-half LSM303DLHC driver
  timers/rtc.c               # RTC upper-half driver

_rentzboy/                   # Personal reference files and templates
  upperdriver_template.c     # Upper/lower half driver pattern reference
  rtc_lowerhalf_template.c   # RTC lower-half pattern reference
  LEER.txt                   # Active configuration notes (USART1, ITM notes)
  lsm303dlhc_gemini.h        # LSM303DLHC register map reference

_builds/stm32f3/             # Saved build outputs
  lsm303dlhc/                # LSM303DLHC build artifacts
  alarm_RTC/                 # RTC alarm build artifacts
```

## NuttX Driver Architecture

NuttX uses a **upper/lower half** split for most drivers:

- **Lower half** (`arch/` or `boards/`): hardware-specific, implements `ops` vtable.
- **Upper half** (`drivers/`): generic POSIX interface, calls lower-half ops via vtable.
- **Board bringup** (`stm32_bringup.c`): instantiates lower half, calls upper half `_initialize()`.

Registration pattern:
```c
/* Lower half returns a pointer to its ops-embedded struct */
lower = stm32_rtc_lowerhalf();

/* Upper half binds lower and registers /dev/rtcN */
rtc_initialize(0, lower);
```

## Inviolable NuttX Rules

These are non-negotiable — never break them:

1. **Strict POSIX compliance** — never compromise the portable OS interface for expediency.
2. **Modular architecture** — formal internal interfaces, minimal global variables, documented porting interfaces.
3. **Coding standard** — see below. Personal preference is never a justification for deviation.
4. **License** — Apache 2.0 (SPDX: `Apache-2.0`). No copyleft. Every file needs the header.
5. **All users matter** — support Linux, Windows (MSYS/Cygwin/native), macOS. GCC, Clang, SDCC, ZiLOG ZDS-II, IAR.

## C Coding Standard

Reference: https://nuttx.apache.org/docs/latest/contributing/coding_style.html

### Language

- **C89** for all common code. C99/C11 only in arch-specific code.
- No C++ comments (`//`). Use `/* ... */` exclusively.

### File Header

Every `.c` and `.h` file starts with Apache 2.0 license block containing `SPDX-License-Identifier: Apache-2.0`.

### Source File Section Order

Each section separated by a `/**** ... ****/` block comment:
1. Included Files
2. Pre-processor Definitions
3. Private Types
4. Private Function Prototypes
5. Private Data
6. Public Data
7. Private Functions
8. Public Functions

### Header Guards

Derived from file path: `include/nuttx/arch.h` → `__INCLUDE_NUTTX_ARCH_H`.

### Formatting

- **Indent**: 2 spaces per level. **No TAB characters** in `.c`/`.h` files.
- **Line width**: max 78 characters.
- **Line endings**: Unix (`\n`).
- **One statement per line**.
- **Braces**: always required, even for single-line blocks. Opening brace on its own line.
  - Control structures (`if`, `while`, `for`): braces indented with the block.
  - Definitions (`struct`, `enum`, functions): opening brace at column 1.

### Naming

| Kind | Convention | Example |
|------|-----------|---------|
| Global variables | `g_` prefix | `g_initialized` |
| Structures | `_s` suffix | `struct my_driver_s` |
| Unions | `_u` suffix | `union my_union_u` |
| Enumerations | `_e` suffix | `enum my_state_e` |
| Typedefs | `_t` suffix | `typedef int my_int_t` |
| Macros / enum values | ALL_CAPS_UNDERSCORE | `MY_MACRO_VALUE` |
| Functions | lowercase, `module_verb()` | `adc_read()` |
| Pointer `*` | adjacent to variable name | `char *buffer` |

### Operators & Expressions

- Space before and after binary operators: `a = b + c`.
- No space after unary operators: `!flag`, `++i`, `*ptr`, `&var`.
- Space after keywords: `if (`, `for (`, `while (`.
- No space between function name and `(`: `my_func(arg)`.
- No parentheses around return value: `return 0;` not `return(0);`.

### Function Header (mandatory for every function)

```c
/****************************************************************************
 * Name: my_function
 *
 * Description:
 *   Brief description of what the function does.
 *
 * Input Parameters:
 *   param1 - Description of the first parameter.
 *   param2 - Description of the second parameter.
 *
 * Returned Value:
 *   Returns 0 (OK) on success. A negated errno value on failure.
 *
 ****************************************************************************/
```

### Reference Snippets

**Conditional block:**
```c
#ifdef CONFIG_MY_FEATURE
  my_feature_initialize();
#else
  g_my_feature_available = false;
#endif
```

**Structure definition:**
```c
struct my_driver_s
{
  int      field1;      /* Description for field1. */
  uint16_t field2;      /* Description for field2. */
  bool     initialized; /* True: driver is initialized. */
};
```

## Key Design Reminders

- Sometimes code duplication is preferable to introducing tight coupling.
- Keep the big picture: POSIX compliance and modularity take priority over local convenience.
- No shortcuts. Expediency does not justify violating the coding standard or POSIX interface.
