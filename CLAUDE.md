# EyeN

ESP32 digital eyes: dual GC9A01 round LCDs track people via an HLK-LD2450 mmWave radar sensor. ESP-IDF 5.5.2, C++ (targeting C++26 long-term).

## Prerequisites

```bash
sudo apt install clang-format-18 cppcheck   # CI lint tools
```

## Build

```bash
source ~/esp/esp-idf/export.sh
idf.py build
idf.py flash monitor        # requires board connected via USB
idf.py fullclean && idf.py build   # clean rebuild (needed after structural changes)
```

## Architecture

ESP-IDF component-based layout. Each component has its own `CMakeLists.txt`, public headers in `include/`, and declared dependencies.

```
components/
  config/     Header-only shared configuration (pins, tuning constants)
  types/      Header-only shared data types (Target, ModeFrame, colors)
  drivers/    Hardware abstraction: display (SPI + GC9A01), ld2450 (UART radar)
  radar/      Sensor processing: target filtering, multi-target attention, gaze computation
  rtos/       Header-only C++ wrappers for FreeRTOS static primitives (Task, Queue, Mutex)
  ui/         Display mode renderers: eye tracking, radar PPI sweep
main/         Application entry point (app_main, task spawning, control plane)
docs/         Log message reference
tools/        Python visualization/tuning tool (not part of firmware)
```

Dependency graph: `config ← types ← drivers ← radar ← main`, `config ← types ← ui ← main`, `rtos ← radar`, `rtos ← main`.

The radar component has two backends selected via `RADAR_SRC` in `components/radar/CMakeLists.txt`:
- `radar_stack_single.cpp` (default) — single sensor + potentiometer vertical
- `radar_stack_multi.cpp` (archived) — pitch-stacked multi-sensor fusion

## Conventions

- C++ source files (`.cpp`), C-compatible headers (`.h` with `extern "C"` guards)
- `clang-format-18` enforced (LLVM-based, see `.clang-format`); run before committing
- Concurrent: radar task (core 1, pri 6), render task (core 0, pri 5), app_main control plane (core 0, pri 1)
- Zero runtime heap allocation: all FreeRTOS objects use `*Static` variants via `rtos::` wrappers
- Inter-task communication via `rtos::Queue` (typed, statically allocated) and `std::atomic`
- All hardware pin assignments and tuning parameters live in `components/config/include/config.h`
- Display modes implement the `DisplayMode` vtable interface from `components/ui/include/mode.h`
- ESP-IDF managed components handle external dependencies (`idf_component.yml` in `components/drivers/` and `main/`)

## Common tasks

- **Add a display mode**: create `components/ui/mode_*.cpp`, implement `display_mode_t`, add to `SRCS` in `components/ui/CMakeLists.txt`, register in `s_modes[]` in `main/main.cpp`
- **Change pin assignments**: edit `components/config/include/config.h`
- **Tune radar filtering**: adjust `CFG_FILTER_*` constants in config.h, or use `tools/replay.py --live` for real-time tuning
- **Switch radar backend**: change `RADAR_SRC` in `components/radar/CMakeLists.txt`
