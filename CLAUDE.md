# EyeN

ESP32 digital eyes: dual GC9A01 round LCDs track people via an HLK-LD2450 mmWave radar sensor. ESP-IDF 5.5.2, C++ (targeting C++26 long-term).

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
  drivers/    Hardware abstraction: display (SPI + GC9A01), ld2450 (UART radar)
  radar/      Sensor processing: target filtering, multi-target attention, gaze computation
  ui/         Display mode renderers: eye tracking, radar PPI sweep
main/         Application entry point (app_main, main loop, mode switching)
tools/        Python visualization/tuning tool (not part of firmware)
```

Dependency graph: `config ← drivers ← radar ← main`, `config ← drivers ← ui ← main`.

The radar component has two backends selected via `RADAR_SRC` in `components/radar/CMakeLists.txt`:
- `radar_stack_single.cpp` (default) — single sensor + potentiometer vertical
- `radar_stack_multi.cpp` (archived) — pitch-stacked multi-sensor fusion

## Conventions

- C++ source files (`.cpp`), C-compatible headers (`.h` with `extern "C"` guards)
- `clang-format` enforced (LLVM-based, see `.clang-format`); run before committing
- Single-threaded: everything runs in `app_main` on the default FreeRTOS task
- All hardware pin assignments and tuning parameters live in `components/config/include/config.h`
- Display modes implement the `display_mode_t` vtable interface from `components/ui/include/mode.h`
- ESP-IDF managed components handle external dependencies (`idf_component.yml` in `components/drivers/` and `main/`)

## Common tasks

- **Add a display mode**: create `components/ui/mode_*.cpp`, implement `display_mode_t`, add to `SRCS` in `components/ui/CMakeLists.txt`, register in `s_modes[]` in `main/main.cpp`
- **Change pin assignments**: edit `components/config/include/config.h`
- **Tune radar filtering**: adjust `CFG_FILTER_*` constants in config.h, or use `tools/replay.py --live` for real-time tuning
- **Switch radar backend**: change `RADAR_SRC` in `components/radar/CMakeLists.txt`
