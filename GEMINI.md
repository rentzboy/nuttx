# Gemini Code Assistant Configuration for NuttX

This document provides a summary of the NuttX coding standards to guide the Gemini Code Assistant in generating and modifying code that aligns with the project's conventions.

## General Principles

- **Follow the NuttX C Coding Standard**: The official [NuttX C Coding Standard](https://nuttx.apache.org/docs/latest/contributing/coding_style.html) is the primary source of truth.
- **Consistency is Key**: When in doubt, the style of the surrounding code should be followed.
- **C89 Compatibility**: All common code must be compatible with the ANSI C89 standard. C99/C11 features are only permissible in architecture-specific code.

## File Structure and Formatting

- **SPDX License Identifier**: All source files must begin with a comment containing the SPDX license identifier. For this project, it is `SPDX-License-Identifier: Apache-2.0`. This should be part of the main file header comment block.
- **File Header**: Every `.c`, `.h`, makefile, and script must start with a standard Apache 2.0 license header block.
- **File Organization**: Source files (`.c`) should follow this order of sections, each separated by a block comment:
    1. Included Files
    2. Pre-processor Definitions
    3. Private Types
    4. Private Function Prototypes
    5. Private Data
    6. Public Data
    7. Private Functions
    8. Public Functions
- **Header Guards**: All header files (`.h`) must use pre-processor guards to prevent multiple inclusions. The guard name is derived from the file path (e.g., `include/nuttx/arch.h` becomes `__INCLUDE_NUTTX_ARCH_H`).
- **Line Endings**: Use Unix-style line endings (`
`).
- **Line Width**: Maximum line width is 78 characters.
- **Indentation**:
    - Use 2 spaces for each indentation level.
    - **DO NOT use TAB characters** in C/H files.
    - Braces for control structures (`if`, `while`, etc.) are on their own line and indented. Braces for definitions (`struct`, `enum`, functions) are not indented and start at column 1.
- **Braces (`{}`)**:
    - Always use braces for `if`, `else`, `while`, `for`, etc., even for single-line blocks.
    - The opening brace is on a new line.
    - The closing brace is on a new line, aligned with the opening keyword.

## Naming Conventions

- **Global Variables**: Prefixed with `g_`. Example: `g_initialized`.
- **Structures**: End with `_s`. Example: `struct my_data_s`.
- **Unions**: End with `_u`. Example: `union my_union_u`.
- **Enumerations**: End with `_e`. Example: `enum my_state_e`.
- **`typedef`**: End with `_t`. Example: `typedef int my_integer_t;`. Typedefs for structures are generally discouraged.
- **Macros & Enum Values**: All uppercase, with words separated by underscores. Example: `MY_MACRO_VALUE`.
- **Functions**:
    - All lowercase.
    - Prefixed with the module/subsystem name, followed by an underscore. Example: `adc_read()`, `net_initialize()`.
    - Use verb-object or object-verb form consistently within a module.
- **Pointers**: The asterisk (`*`) should be placed next to the variable name, not the type. Example: `char *buffer;`.

## Comments

- **Use C-style comments (`/* ... */`) only**. Do not use C++ style comments (`//`).
- **Block Comments**: Used to separate major sections of a file.
- **Function Headers**: Every function must be preceded by a standardized block comment detailing its name, description, input parameters, and return values.
- **Grammar**: Comments should be in clear, standard US English with correct grammar and punctuation.
- **Placement**: Comments should generally precede the code they describe and be indented to the same level. Comments to the right of statements are discouraged but acceptable for short notes on data definitions.

## Statements and Expressions

- **One Statement Per Line**: Only one statement is allowed per line.
- **Operators**:
    - Place a space before and after binary operators (`=`, `+`, `==`, etc.).
    - Do not place a space after unary operators (`!`, `++`, `--`, `*` (dereference), `&` (address-of)).
- **Parentheses**:
    - Place a space after keywords like `if`, `for`, `while`, `switch`.
    - Do not put a space between a function name and its opening parenthesis.
    - Avoid unnecessary parentheses in expressions.
- **Return Values**: Do not enclose the return value in parentheses unless required for expression evaluation. `return 0;` is correct, `return(0);` is incorrect.

## Example Snippets

### Function Definition

```c
/****************************************************************************
 * Name: my_function
 *
 * Description:
 *   A brief description of what the function does.
 *
 * Input Parameters:
 *   param1 - Description of the first parameter.
 *   param2 - Description of the second parameter.
 *
 * Returned Value:
 *   Returns 0 (OK) on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int my_function(int param1, FAR const char *param2)
{
  int ret = OK;

  /* Function logic goes here */

  if (param1 < 0)
    {
      ret = -EINVAL;
    }

  return ret;
}
```

### Structure Definition

```c
struct my_driver_s
{
  int      field1;     /* Description for field1. */
  uint16_t field2;     /* Description for field2. */
  bool     initialized;/* True: driver is initialized. */
};
```

### Conditional Block

```c
#ifdef CONFIG_MY_FEATURE
  /* Code for when the feature is enabled. */

  my_feature_initialize();
#else
  /* Code for when the feature is disabled. */

  g_my_feature_available = false;
#endif
```