# EyeN

ESP32 digital eyes: dual GC9A01 round LCDs track people via a **pitch-stacked** pair of HLK-LD2450 radars (same roll, different pitch). Vertical gaze comes from which sensors see the person; horizontal gaze from averaged azimuth.

**Board:** ELEGOO ESP32-WROOM-32

Silk labels:  
`VIN GND D13 D12 D14 D27 D26 D25 D33 D32 D35 D34 VN VP EN 3V3` ·  
`GND D15 D2 D4 RX2 TX2 D5 D18 D19 D21 RX0 TX0 D22 D23`

## Wiring

Common **GND**. Displays → **3V3**. Radars → **VIN** (~5V when USB-powered).

### GC9A01 (both eyes — shared SPI, 7-pin)

| Display | ESP32 silk |
|---------|------------|
| VCC | **3V3** |
| GND | **GND** |
| SCL | **D18** |
| SDA | **D23** |
| DC | **D27** |
| CS left / right | **D5** / **D15** |
| RST | **D4** (shared) |

### Pitch-stack LD2450s

| Sensor | Mount pitch | Sensor TX → ESP | ESP → Sensor RX |
|--------|-------------|-----------------|-----------------|
| **Lower** | ~−20° | **RX2** (GPIO 16) | **TX2** (GPIO 17) |
| **Upper** | ~+20° | **D21** (GPIO 21) | **D22** (GPIO 22) |

Both: VCC→**VIN**, GND→**GND**. Copper faces the walkway with **4 pads bottom / 2 pads top**; only the up/down nod (pitch) differs between lower and upper.

The **TX wiring** (ESP → Sensor RX) is needed for live tuning of sensor parameters from the Python tool. If you only need tracking without tuning, the TX wires can be left unconnected.

Do **not** use `RX0`/`TX0` (USB serial). Baud **256000** 8N1.

### Later: third (mid) sensor + 74HC4051

Software already uses an N-slot table with a reserved `mux_channel`. When the mux arrives, add a mid slot (~0° pitch) and optionally move all three TX lines through the 4051 into one UART. See comments in `pinout.h` / `radar_stack.c`.

## Vertical bands (2 sensors)

| Who sees the person | Band | Pupil |
|---------------------|------|-------|
| Upper only | 2 | top |
| Both | 1 | center |
| Lower only | 0 | bottom |

## Orientation

Point each module’s **copper-patch face** at the walkway. The plug-in adhesive antenna is **Bluetooth only** — not for tracking.

**Roll (critical for left/right):** hold the board so the copper face is toward you with **four pads on the bottom and two on the top**. That is the orientation where sensor `x` tracks left/right well. Do **not** mount it “landscape” with pads left/right of each other — left/right gaze will feel wrong or weak.

**Pitch (for the stack):** keep that same roll on both sensors, then nod them differently (~−20° lower, ~+20° upper) so their elevation FOVs cover different heights. Copper faces still point into the room.

## Build / flash

```bash
source ~/esp/esp-idf/export.sh
cd ~/esp/EyeN
idf.py set-target esp32
idf.py build flash monitor
```

Logs: `$FRAME t=… eye=… sensor=count:targets` or `$LOST t=…`.

## Visualization / live tuning

Replay a log file:
```bash
python3 tools/replay.py log.txt --speed 0        # step with spacebar
```

Live mode with sensor tuning sliders (requires TX pins wired):
```bash
python3 tools/replay.py --live /dev/ttyUSB0
```

Sliders control LD2450 parameters in real time:
- **Sensitivity** (0–9): lower = fewer false positives, reduced range
- **Energy threshold** (100–10000): higher = ignores weak reflections / ghosts
- **Min speed filter** (0–100 cm/s): ignores static targets below threshold
- **Hold time** (0–60 s): how long a target persists after leaving FOV

Settings are sent to *all* sensors and persist across power cycles on the LD2450.
