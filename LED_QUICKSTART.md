# LED Control System — Quickstart Guide

## Overview

The LED control system has been implemented with three core modules:

1. **`led_serial.py`** — Low-level RS485 communication, LEDStrip abstraction
2. **`led_animations.py`** — Animation classes
3. **`led_animation_controller.py`** — Thread-based animation manager (your main interface)

## Hardware Mapping

**Strip IDs and Locations:**

```
Strip 11: (-2367, 0)     Strip 12: (-2272, 0)      [Column 1]
Strip 21: (-1868, 1480)  Strip 22: (-1773, 1480)   [Column 2]
Strip 31: (-1868, -1480) Strip 32: (-1773, -1480)  [Column 3]
Strip 41: (-48, 1480)    Strip 42: (47, 1480)      [Column 4]
Strip 51: (-48, -1480)   Strip 52: (47, -1480)     [Column 5]
Strip 61: (1773, 1480)   Strip 62: (1868, 1480)    [Column 6]
Strip 71: (1773, -1480)  Strip 72: (1868, -1480)   [Column 7]
Strip 81: (2272, 0)      Strip 82: (2367, 0)       [Column 8]
```

Each strip has **28 addressable LEDs**.

---

## Basic Usage

### Option 1: Simple Direct Control

```python
from led_animation_controller import LEDAnimationController
from led_serial import RED, BLUE, GREEN

controller = LEDAnimationController(serial_port='COM3')
controller.start()

# Set a single strip to red
controller.set_strip_color(11, RED)

# Set all strips to blue
controller.set_all_strips_color(BLUE)

controller.stop()
```

### Option 2: Queue Animations (Recommended)

```python
from led_animation_controller import LEDAnimationController
from led_animations import FillAnimation, ColorAnimation
from led_serial import RED, BLUE, GREEN, OFF

controller = LEDAnimationController()
controller.start()

# Queue a fill animation (0→28 LEDs over 2 seconds)
anim = FillAnimation(
    strip_ids=[11, 12],          # Animate both strips
    target_leds=28,              # Fill all 28 LEDs
    duration_ms=2000,            # Over 2 seconds
    color=GREEN,                 # In green
)
controller.queue_animation(anim)

# Queue a color change after
next_anim = ColorAnimation([11, 12], RED, duration_ms=500)
controller.queue_animation(next_anim)

# Wait for animations to play
time.sleep(3)

controller.stop()
```

### Option 3: Standalone LEDSystem (Lower Level)

```python
from led_serial import LEDSystem, RED, BLUE

system = LEDSystem()
system.start()

# Direct control without animation queue
system.set_strip_color(11, RED)
system.set_strip_fill(12, 14, BLUE)  # Fill first 14 LEDs in strip 12

system.stop()
```

---

## Integration with GameController

The `LEDAnimationController` is auto-integrated into `GameController`:

```python
from game_controller import GameController
from game_settings import GameSettings
from led_animations import FillAnimation
from led_serial import RED, BLUE

settings = GameSettings()
controller = GameController(settings)
controller.start()

# Access LED display via property
led_display = controller.led_display

# Queue animations during gameplay
if game_state == "GameOn":
    score_height = int((team1_score / max_score) * 28)
    
    anim = FillAnimation(
        strip_ids=[11, 12, 21, 22],  # Team 1 columns
        target_leds=score_height,
        duration_ms=500,
        color=RED,
    )
    led_display.queue_animation(anim)

controller.stop()
```

---

## Available Animations

### 1. FillAnimation
Progressively fill LEDs from 0 to num_leds.

**Use case:** Score visualization

```python
anim = FillAnimation(
    strip_ids=[11, 12],
    target_leds=28,           # Fill all LEDs
    duration_ms=2000,         # Over 2 seconds
    color=GREEN,
    off_color=OFF,            # Color for unfilled LEDs
)
```

### 2. ColorAnimation
Set strips to a fixed color instantly.

**Use case:** Static color display

```python
anim = ColorAnimation(
    strip_ids=[11, 12],
    color=RED,
    duration_ms=100,  # How long to hold
)
```

### 3. PulseAnimation
Brightness oscillates between two colors.

**Use case:** Attention effects, alerts

```python
anim = PulseAnimation(
    strip_ids=[11, 12],
    color1=RED,
    color2=OFF,
    duration_ms=1000,  # Cycle time
)
```

---

## Testing

A complete test suite is provided in `test_led_system.py`:

```bash
python src/test_led_system.py
```

This runs four tests:
1. **Basic Colors** — Static color setting
2. **Fill Animation** — Progressive fill
3. **Animation Controller** — Queued animations
4. **Independent Strips** — Per-strip control

---

## Hardware Setup (Minimal Configuration)

For testing with limited hardware:

**Required:**
- 1 RS485 USB adapter (CH340 or similar)
- 1 LED controller (RS485 address 0x01)
- 2 WS2811 LED strips (12V, ~1m, 28 LEDs each)

**Wiring:**
```
USB RS485 Adapter
├─ A (green) → Controller A
├─ B (blue)  → Controller B
└─ GND       → Controller GND

Controller (Address 0x01)
├─ VIN → +12V
├─ GND → GND
├─ Channel 1 (DAT) → Strip 11 Data line
├─ Channel 1 (CLK) → Strip 11 Data line (both identical for WS2811)
├─ Channel 2 (DAT) → Strip 12 Data line
└─ Channel 2 (CLK) → Strip 12 Data line
```

---

## Color Reference

```python
from led_serial import RED, GREEN, BLUE, WHITE, OFF, Color

# Built-in colors
RED        # Color(255, 0, 0)
GREEN      # Color(0, 255, 0)
BLUE       # Color(0, 0, 255)
WHITE      # Color(255, 255, 255)
OFF        # Color(0, 0, 0)

# Custom colors
yellow = Color(255, 255, 0)
purple = Color(128, 0, 255)

# From hex
color = Color.from_hex("#FF00FF")
```

---

## Troubleshooting

### No Serial Connection
- Check USB RS485 adapter is plugged in
- Verify device appears in Device Manager (COM port)
- Logs will show "WARNING: Could not connect to hardware. Running in simulation mode."

### Animations Not Playing
- Ensure `controller.start()` was called
- Check that animations are enqueued (not just created)
- Verify strip IDs are valid (11–82)

### LEDs Not Lighting
- Check power supply (12V) to LED strips
- Verify RS485 wiring (A=green, B=blue to controller)
- Test with basic color animations first

---

## Next Steps

1. **Wave Effect** — Animate across strips using XY coordinates
2. **Team Visualization** — Alternate colors for red/blue teams
3. **Score Bar** — Dynamic fill based on game score
4. **Collision Indicator** — Flash strips when collisions detected

See comments in `led_animations.py` for extension points.
