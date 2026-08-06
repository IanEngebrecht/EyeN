# Log Messages

EyeN emits structured log lines over UART0 (the USB serial monitor). All
machine-readable messages use a `$` prefix so they can be parsed by
`tools/replay.py` or filtered with `grep`.

Connect with `idf.py monitor` or any serial terminal at 115200 baud.

## `$MODE`

Emitted when the active display mode changes (button press or startup).

```
I (1080) EyeN: $MODE eye
I (7542) EyeN: $MODE radar
```

| Field | Description |
|-------|-------------|
| name  | Active display mode: `eye` or `radar` |

## `$FRAME`

Emitted every 10th radar frame (~once per second at 10 Hz). Shows the
current state of all sensor slots and their detected targets.

```
I (2655) radar_single: $FRAME t=2655 main=2:6,-521,521,8|-853,-1740,1937,8|-
```

| Field | Description |
|-------|-------------|
| `t`   | Timestamp in milliseconds since boot |
| `<slot>=<n>:` | Slot name and number of valid targets after filtering |
| target entry | `x_mm,y_mm,distance_mm,speed_cm_s` for each target |
| `-`   | Empty target slot (no detection) |

Targets are separated by `|`. The LD2450 reports up to 3 targets per
sensor. A target entry of `-` means that slot had no detection.

## `$SKIP`

Warning emitted when radar frame IDs are non-sequential, meaning one or
more frames were dropped between reads.

```
W (12345) radar_single: $SKIP frames=2 total=5
```

| Field | Description |
|-------|-------------|
| `frames` | Number of frames skipped in this gap |
| `total`  | Cumulative skipped frames since boot |

If you see `$SKIP` lines, the radar task is being starved. This should
not occur under normal operation with concurrency enabled.

## `$LOST`

Emitted on the transition from human-detected to no-human-detected (the
gaze target disappeared from the sensor's field of view).

```
I (56226) radar_single: $LOST t=56226
```

| Field | Description |
|-------|-------------|
| `t`   | Timestamp in milliseconds since boot |

## `$HEALTH`

Periodic system health check. First emitted 30 seconds after boot, then
every 60 seconds.

```
I (90999) EyeN: $HEALTH heap=163796 delta=-308 radar_stk=1308 render_stk=2068
```

| Field | Description |
|-------|-------------|
| `heap`       | Current free heap in bytes |
| `delta`      | Change from startup heap in bytes (negative = less free) |
| `radar_stk`  | Radar task minimum unused stack bytes (high water mark) |
| `render_stk` | Render task minimum unused stack bytes (high water mark) |

**Reading the values:**

- `delta` should stabilize near zero. A small negative value (a few
  hundred bytes) in the first minute is normal — ESP-IDF internals do
  lazy one-time allocation. A continuously decreasing value indicates a
  memory leak.
- Stack values show the *minimum* free stack ever observed. If either
  drops below ~200 bytes, increase that task's stack size in
  `main.cpp`. Values above 1000 indicate comfortable headroom.

## `$ACK`

Response to a `$SET` command received over UART0.

```
I (5000) EyeN: $ACK min_dist 500 OK
W (5100) EyeN: $ACK foobar UNKNOWN_PARAM
W (5200) EyeN: $ACK sensitivity 5 QUEUE_FULL
```

| Field | Description |
|-------|-------------|
| param | The parameter name from the `$SET` command |
| value | The value that was set (omitted for `UNKNOWN_PARAM`) |
| status | `OK`, `UNKNOWN_PARAM`, or `QUEUE_FULL` |

## `$SET` (input command)

Send commands over UART0 to change parameters at runtime. Format:

```
$SET <param> <value>
```

**Software filter parameters** (applied immediately by the radar task):

| Parameter   | Description | Default |
|-------------|-------------|---------|
| `min_speed` | Minimum \|speed\| in cm/s to count as real (0 = off) | 0 |
| `min_dist`  | Minimum distance in mm (closer = phantom) | 400 |
| `max_dist`  | Maximum distance in mm (farther = noise) | 6000 |
| `persist`   | Target must appear in N of last 3 frames | 2 |

**Hardware parameters** (sent to the LD2450 sensor):

| Parameter      | Description |
|----------------|-------------|
| `sensitivity`  | Sensor sensitivity (0-9) |
| `energy`       | Energy detection threshold |
| `speed_filter` | Hardware-side speed filter |
| `hold_time`    | Target hold time after disappearing |
| `restart`      | Restart the sensor (value ignored) |
