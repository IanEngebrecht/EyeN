# Software Design

## Layered Architecture

The firmware is organized into four layers. Dependencies flow strictly downward — a component may only depend on components in the same layer or below. This means, for example, the LD2450 UART driver cannot depend on the UI module, and neither `radar` nor `ui` can depend on each other.

```mermaid
graph TD
    subgraph APP["Application"]
        main
    end

    subgraph LOGIC["Application Logic"]
        radar
        ui
    end

    subgraph HAL["Hardware Abstraction"]
        drivers
    end

    subgraph FOUND["Foundation (header-only)"]
        config
        types
        rtos
    end

    main --> radar & ui
    radar --> drivers
    radar --> rtos
    drivers --> config & types
    ui --> config & types
```

| Layer | Component | Responsibility |
|-------|-----------|----------------|
| **Application** | `main` | Entry point: initializes hardware, spawns FreeRTOS tasks, runs the control-plane loop (button polling, UART commands, health logging). Depends on both application-logic components but they do not depend on it. |
| **Application Logic** | `radar` | Sensor reading, target filtering, multi-target attention selection, gaze computation (azimuth + elevation). Owns the sensor read loop. |
| | `ui` | `DisplayMode` interface and concrete renderers (`EyeMode`, `RadarMode`). Pure rendering — no sensor or hardware knowledge. Peer to `radar`; the two cannot depend on each other and communicate only through queues wired by `main`. |
| **Hardware Abstraction** | `drivers` | `Display` (SPI bus + dual GC9A01 panels) and `Ld2450` (UART radar protocol). Hardware I/O only — no processing logic. |
| **Foundation** | `config` | Pin assignments, tuning constants, sensor array definition. Single file to edit for hardware changes. |
| | `types` | `Target`, `ModeFrame`, color constants. Shared vocabulary types with no logic. |
| | `rtos` | `rtos::Task`, `rtos::Queue`, `rtos::Mutex` — C++ wrappers enforcing static allocation of FreeRTOS primitives. Any layer may depend on foundation components. |

## Concurrency Model

Three execution contexts run concurrently. The radar task is pinned to core 1 for uninterrupted UART reads; rendering and the control plane share core 0, separated by priority.

```mermaid
graph LR
    CTRL["app_main · pri 1"]:::core0
    RENDER["render · pri 5 · 4 KB"]:::core0
    RADAR["radar · pri 6 · 4 KB"]:::core1

    RADAR -->|frame_q| RENDER
    CTRL -->|cmd_q| RADAR
    CTRL -->|mode_q| RENDER

    classDef core0 fill:#3b82f6,stroke:#1e40af,color:#fff
    classDef core1 fill:#f59e0b,stroke:#b45309,color:#fff
```

Legend: <span style="color:#3b82f6">**blue = core 0**</span>, <span style="color:#f59e0b">**amber = core 1**</span>

| Channel | Type | Direction | Purpose |
|---------|------|-----------|---------|
| `frame_q` | `Queue<ModeFrame, 2>` | radar → render | Processed sensor frames with gaze vector |
| `cmd_q` | `Queue<DevCommand, 4>` | app_main → radar | Live tuning commands from UART0 |
| `mode_q` | `Queue<ModeSwitch, 1>` | app_main → render | Display mode change requests |
| `pot_frac` | `std::atomic<float>` | radar → (shared) | Smoothed potentiometer reading (0.0–1.0) |

All FreeRTOS objects use `*Static` creation variants via the `rtos::` wrappers — zero runtime heap allocation.

### Task Details

**radar** (core 1, priority 6) — tight loop: drain `cmd_q`, call `Ld2450::read_frame()` (blocks up to 100 ms on UART), filter targets, select attention target, compute gaze, build `ModeFrame`, send to `frame_q`. Logs frames at 1 Hz.

**render** (core 0, priority 5) — loop: check `mode_q` for mode switches, receive from `frame_q` (10 ms timeout), apply azimuth coasting, call `DisplayMode::render()`. When no frame is available, the loop spins on `frame_q` to stay responsive.

**app_main** (core 0, priority 1) — 20 ms poll loop: reads UART0 for `$SET` commands, polls the button with debounce, emits `$HEALTH` logs every 60 s (heap, stack high-water marks).

## Data Types

```mermaid
classDiagram
    class Target {
        +int16_t x_mm
        +int16_t y_mm
        +int16_t speed_cm_s
        +uint16_t distance_mm
        +bool valid
    }

    class ModeFrame {
        +Target targets[3]
        +int target_count
        +int primary_idx
        +Target primary
        +float azimuth_deg
        +float elevation_norm
        +bool human
        +uint32_t frame_id
    }

    ModeFrame o-- "3" Target : contains

    class Gaze {
        +bool human
        +float azimuth_deg
        +float elevation_norm
        +Target primary
        +uint32_t frame_id
        +int total_targets
        +SlotInfo slots[4]
    }

    class SlotInfo {
        +const char* name
        +int target_count
        +Target targets[3]
    }

    Gaze o-- "4" SlotInfo : contains
    SlotInfo o-- "3" Target : contains

    Gaze ..> ModeFrame : "build_mode_frame()"
```

`Target` is the raw per-target output from the LD2450: position, speed, and a validity flag. The radar component produces a `Gaze` internally (carrying slot-level detail for multi-sensor support), then converts it to a `ModeFrame` — the simplified struct that crosses the queue to the render task.

## Display Mode Interface

```mermaid
classDiagram
    class DisplayMode {
        <<abstract>>
        +name()* const char*
        +enter(left, right, scanline)*
        +render(left, right, frame)*
        +leave()*
    }

    class EyeMode {
        +name() "eye"
        -cur_x_, cur_y_, cur_r_ : float
        -dot_x_, dot_y_, dot_r_ : int
        -move_dot_on_panel()
        -draw_filled_circle()
    }

    class RadarMode {
        +name() "radar"
        -RadarTarget targets_[3]
        -dist_lut_[240][240] : uint8_t
        -ang_lut_[240][240] : uint8_t
        -render_left(sweep_deg)
        -render_right(frame)
    }

    DisplayMode <|-- EyeMode
    DisplayMode <|-- RadarMode
```

Modes are singletons returned by factory functions (`eye_mode()`, `radar_mode()`). The render task calls `enter()` on activation, `render()` each frame, and `leave()` on mode switch. Modes receive panel handles and a shared scanline buffer — they own no hardware resources.

**EyeMode** — lerps pupil position toward the gaze target each frame, redraws only the bounding box of old + new pupil positions (minimal SPI traffic). Pupil radius shrinks with target distance.

**RadarMode** — left eye: full-framebuffer PPI sweep with precomputed angle/distance LUTs, target blips that fade between sweeps. Right eye: monospace text readout, diff-updated (only redraws changed lines).

## Driver Classes

```mermaid
classDiagram
    class Display {
        +init() esp_err_t
        +left() esp_lcd_panel_handle_t
        +right() esp_lcd_panel_handle_t
        +fill(panel, color)
        +scanline_buf() span~uint16_t, 240~
        -dma_line_[240] : uint16_t
        -create_panel(host, cs, rotation, out)$
        -pulse_shared_reset()$
    }

    class Ld2450 {
        +create(cfg, out)$ esp_err_t
        +read_frame(targets, timeout) esp_err_t
        +set_sensitivity(val)
        +set_energy_threshold(val)
        +set_speed_filter(val)
        +set_hold_time(val)
        +restart()
        +read_firmware_version(buf)
        +unstick()
        -uart_num_ : uart_port_t
        -valid_ : bool
        -enter_config()
        -end_config()
        -send_config_cmd(payload, len)
    }
```

`Display` manages the shared SPI bus and both GC9A01 panels. It exposes raw `esp_lcd_panel_handle_t` handles — the modes call ESP-IDF panel APIs directly. A single DMA-aligned scanline buffer is shared across all rendering to avoid per-mode allocation.

`Ld2450` wraps the HLK-LD2450 UART protocol: frame parsing, config commands (sensitivity, energy threshold, speed filter, hold time), and firmware version queries. Move-only; created via a static factory.

## RTOS Wrappers

```mermaid
classDiagram
    class Task~StackBytes~ {
        +create(name, priority, fn, arg, core)
        +handle() TaskHandle_t
        -stack_[StackBytes] : StackType_t
        -tcb_ : StaticTask_t
    }

    class Queue~T, Depth~ {
        +create()
        +send(item, timeout) bool
        +receive(item, timeout) bool
        +overwrite(item) bool
        -storage_[Depth * sizeof T]
        -control_ : StaticQueue_t
    }

    class Mutex {
        +create()
        +lock(timeout) bool
        +unlock()
        -storage_ : StaticSemaphore_t
    }

    class LockGuard {
        +LockGuard(Mutex&)
        +~LockGuard()
    }

    LockGuard --> Mutex : RAII lock
```

All wrappers embed the backing storage as member arrays, so the FreeRTOS objects live in static (BSS) memory with zero heap allocation. Template parameters control stack size and queue depth at compile time.

## Sequence: Startup

```mermaid
sequenceDiagram
    participant M as app_main
    participant D as Display
    participant R as radar::init
    participant RT as radar task
    participant RD as render task

    M->>M: init_uart0_rx()
    M->>M: button_init()
    M->>D: init()
    D->>D: pulse_shared_reset()
    D->>D: create_panel(cs_left)
    D->>D: create_panel(cs_right)
    M->>R: init()
    R->>R: Ld2450::create(UART2)
    R->>R: pot_init(ADC, GPIO34)
    M->>M: create frame_q, cmd_q, mode_q

    M->>RD: spawn (core 0, pri 5)
    activate RD
    RD->>RD: eye_mode().enter()

    M->>RT: spawn (core 1, pri 6)
    activate RT
    RT->>RT: enter radar loop

    M->>M: enter control loop (20 ms poll)
```

## Sequence: Frame Pipeline (Steady State)

```mermaid
sequenceDiagram
    participant S as LD2450
    participant RT as radar task
    participant Q as frame_q
    participant RD as render task
    participant LCD as GC9A01 displays

    loop every ~100 ms (sensor 10 Hz)
        S->>RT: UART frame (3 target slots)
        RT->>RT: filter_targets(min_speed, dist, persist)
        RT->>RT: select_attention_target()
        RT->>RT: azimuth = atan2(x, y)
        RT->>RT: elevation = atan2(height_delta, range)
        RT->>RT: build_mode_frame(gaze → ModeFrame)
        RT->>Q: send(ModeFrame)
    end

    loop render (as fast as frames arrive)
        Q->>RD: receive(ModeFrame, 10 ms timeout)
        RD->>RD: aim_azimuth_deg() — rate smoothing + coast
        RD->>LCD: mode→render(left, right, frame)
    end
```

## Sequence: Mode Switch

```mermaid
sequenceDiagram
    participant BTN as Button (D21)
    participant M as app_main
    participant Q as mode_q
    participant RD as render task
    participant LCD as GC9A01 displays

    BTN->>M: GPIO low (press)
    M->>M: debounce (200 ms guard)
    M->>M: next = (current + 1) % mode_count
    M->>Q: send(ModeSwitch{next})
    Q->>RD: receive(ModeSwitch)
    RD->>RD: current_mode→leave()
    RD->>RD: new_mode→enter(left, right, scanline)
    RD->>LCD: clear + initial draw
    RD->>RD: reset_azimuth_coast()
    Note over M: app_main continues polling
```

## Sequence: Live Tuning Command

The Python tool (`tools/replay.py --live`) sends `$SET` commands over USB serial (UART0). These reach the radar task asynchronously via the command queue.

```mermaid
sequenceDiagram
    participant PY as Python tool
    participant M as app_main
    participant Q as cmd_q
    participant RT as radar task
    participant HW as LD2450

    PY->>M: "$SET sensitivity 7\n" (UART0)
    M->>M: parse → DevCommand{hw_sensitivity, 7}
    M->>Q: send(DevCommand)
    M->>PY: "$ACK sensitivity 7 OK" (log)

    Note over RT: next loop iteration
    Q->>RT: receive(DevCommand)
    RT->>HW: Ld2450::set_sensitivity(7)
```
