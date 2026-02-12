# Style Guide (C/C++ / ESP-IDF)

This document defines coding conventions for agents and contributors.

## Formatting

- Use `clang-format` with:

```text
--style={ BasedOnStyle: LLVM, ColumnLimit: 100, UseTab: Never, IndentWidth: 2, TabWidth: 2, BreakBeforeBraces: Attach, CommentPragmas: "^", SortIncludes: false, PointerAlignment: Right, ReferenceAlignment: Right, IndentCaseLabels: false }
```

- Indentation: 2 spaces, no tabs.
- Braces: attach style.
- Keep lines <= 100 columns.
- Do not rely on include sorting; keep includes intentional.
- One-line `static inline` helpers are fine when they fit within the 100 column limit.

## Headers

- Do not use `#pragma once`.
- Use include guards:

```c
#ifndef SOME_UNIQUE_HEADER_GUARD_H
#define SOME_UNIQUE_HEADER_GUARD_H

// ...

#endif
```

## Naming

### Constants

- Constants use ALL_CAPS.
- Prefer `static constexpr` (C++) or `#define` (C) as appropriate.

```cpp
static constexpr int WIDTH = 144;
static constexpr int HEIGHT = 296;
static constexpr size_t BUFFER_SIZE = (WIDTH * HEIGHT) / 8;
```

### Types

- Class/struct/typedef names use ALL_CAPS when they represent external model/product identifiers.
- Otherwise use a clear, conventional C++ type name (project-specific preference may vary).

### Enums

- Do not prefix enum values with `k`.
- With `enum class`, use clean value names:

```cpp
enum class Mode {
  Unknown = 0,
  Full,
  Fast,
};
```

### Private members

- Private functions/variables use a leading underscore:

```cpp
private:
  esp_err_t _do_thing();
  int _state;
```

- Exception: private pointer/handle/object members should use a trailing underscore to make
  ownership/handle-ness obvious:

```cpp
private:
  spi_device_handle_t spi_;
  SomeObj* obj_;
```

## Error handling

- Avoid repetitive inline `if (err != ESP_OK) return err;` blocks when flow becomes noisy.
- Prefer standardized helper macros for `return`/`goto` error flow.
- Logging in error macros is optional; default is *no log message* unless it materially helps.
- If logging is needed, printing the `esp_err_t` name/code is sufficient.

ESP-IDF has helper macros (they typically log):

```c
ESP_RETURN_ON_ERROR(expr, TAG, "optional message");
ESP_GOTO_ON_ERROR(expr, cleanup, TAG, "optional message");
```

If you don't want logging, use small local helpers instead:

```cpp
#define RETURN_ON_ERROR(expr) do { esp_err_t _err = (expr); if (_err != ESP_OK) return _err; } while (0)
#define RETURN_IF_ERROR(err) do { if ((err) != ESP_OK) return (err); } while (0)
#define GOTO_ON_ERROR(expr, label) do { err = (expr); if (err != ESP_OK) goto label; } while (0)
```

## GPIO conventions

- For optional/unconnected pins, use `GPIO_NUM_MAX` as the sentinel.
- Avoid inventing custom sentinel values.

## Concurrency / SPI bus access

- Prefer ESP-IDF native bus locking:
  - `spi_device_acquire_bus(handle, wait_ticks)`
  - `spi_device_release_bus(handle)`

- Standardized locking rule:
  - Acquire/release is done only in top-level public operations.
  - Hold the bus only while actively transmitting (CMD/DATA bursts, RAM streaming, update triggers).
  - Never hold the bus while waiting on long device states (e.g., `BUSY` polling for e-paper refresh).

## Initialization placement

- Initialize shared buses (SPI/I2C) once in top-level code (e.g., `app_main`).
- Device/UI init helpers assume the bus is already initialized and should not call `spi_bus_initialize` / `i2c_new_master_bus`.

## C++ usage

- Keep control-flow simple; avoid clever constructs.
- Do not use lambdas for simple delays or small helpers in application code.
  Prefer named functions:

- Never use the ternary operator (`?:`). Use `if`/`else` for clarity.

```cpp
static void step_pause(void) {
  vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
}
```

Avoid:

```cpp
const bool pressed = active_low ? (level == 0) : (level != 0);
```

Prefer:

```cpp
bool pressed = false;
if (active_low) {
  pressed = (level == 0);
} else {
  pressed = (level != 0);
}
```

## Comments

- Comments should explain non-obvious intent, not restate what the code already says.
- Keep comments short and accurate.
