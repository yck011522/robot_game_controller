# Robot Game Scoreboard

Multi-display NeoPixel scoreboard with **independent text and particle-physics layers**,
controllable over **OSC/UDP**, **USB-Serial**, **RS485**, or a built-in **WiFi web interface**.
Built on ESP32-S3 (M5Stack AtomS3) with PlatformIO / Arduino.

```
┌──────────┐  ┌──────────┐       ┌──────────┐
│  32 × 8  │→ │  32 × 8  │→ … → │  32 × 8  │   (1 to N daisy-chained tiles)
│ Display 1│  │ Display 2│       │ Display N│
└──────────┘  └──────────┘       └──────────┘
      ↑ NeoPixel data (single GPIO)
  ┌────────┐
  │ AtomS3 │◄── OSC / UDP   (WiFi STA or Ethernet PoE)
  │ ESP32  │◄── Web UI      (WiFi AP, button-toggled)
  │        │◄── USB-Serial   (text commands)
  │        │◄── RS485        (text commands, optional)
  └────────┘
```

### Key features

- **Two independent layers per display** — text (foreground) and particles (background),
  each with its own enable toggle, brightness (0–255), and colour.
- **Particle physics engine** — gravity (IMU-driven on AtomS3), inter-particle collisions,
  attraction, Langevin temperature, configurable damping and elasticity.
- **Multiple render styles** — point, square, circle, text-on-particle, Gaussian glow
  with optional wave interference, and speed-to-colour heat map.
- **Text → Particles** — convert rendered text into particles that can be frozen,
  animated, or cleared. Physics can be paused/resumed independently.
- **View transform** — OpenGL-style render-time rotation, scale, and translation
  of the particle layer (does not affect physics positions).
- **Text stack** — per-display stack of up to 8 strings; scroll modes cycle through them.
- **Scroll animation** — instant, scroll-up, or scroll-down with configurable speed
  and continuous auto-cycle mode.
- **Four command interfaces** — OSC/UDP, USB-Serial, RS485, and HTTP (web UI) —
  all sharing the same command syntax.
- **WiFi web control panel** — full HTML/JS page served by the ESP32 in AP mode,
  toggled by pressing the AtomS3 button.
- **Desktop GUI** — `test/gui_control.py` (Python/tkinter) over USB-Serial,
  with two-column layout, JSON preset save/load, and per-display state tracking.
- **NVS persistence** — all display settings survive power cycles
  (text, colours, brightness, layer enables, particle config, scroll state).
- **Dead-LED compensation** — configurable skip index for a bypassed LED.

---

## Hardware

| Part                                          | Role            | Notes                                              |
| --------------------------------------------- | --------------- | -------------------------------------------------- |
| M5Stack **AtomS3** (ESP32-S3)                 | Microcontroller | Any ESP32 board works (see build envs)             |
| 1–N × WS2812B **32×8 NeoPixel** matrices      | Displays        | Daisy-chained; set `NUM_DISPLAYS` in config        |
| M5Stack **Ethernet Unit with PoE** (optional) | Wired network   | W5500 SPI, for reliable production use             |
| M5Stack **Atomic RS485 Base** (optional)      | RS485 bus       | SP3485EE, auto direction control                   |
| **5 V PSU** (≥ 5 A recommended)               | LED power       | ~10 mA/LED theoretical; text @ brightness 10 ≈ 1 A |

> **Power note:** Never power the matrices from USB alone.
> Text at low brightness draws ~1–2 A total; particle glow mode can draw more.
> A 5–10 A supply is safe for up to 6 tiles.

See [docs/hardware_setup.md](docs/hardware_setup.md) for wiring details.

---

## Quick start

### 1. Prerequisites

- [PlatformIO CLI](https://docs.platformio.org/en/latest/core/installation.html)
  or the **PlatformIO IDE** VS Code extension.
- Python 3 + `python-osc` for test scripts: `pip install python-osc`
- Python 3 + `tkinter` + `pyserial` for the GUI: `pip install pyserial`

### 2. Clone & configure

```bash
git clone https://github.com/alvarohub/robot_game_scoreboard.git
cd robot_game_scoreboard

# Create your WiFi credentials file (not committed to git)
cp src/credentials.h.example src/credentials.h
# Edit src/credentials.h with your SSID and password
```

### 3. Build & flash

```bash
# AtomS3 + WiFi (default)
pio run -e atoms3-wifi -t upload

# AtomS3 + Ethernet PoE
pio run -e atoms3-ethernet -t upload

# AtomS3 + WiFi + RS485
pio run -e atoms3-wifi-rs485 -t upload

# Generic ESP32 DevKit + WiFi
pio run -e esp32dev-wifi -t upload
```

### 4. Monitor serial output

```bash
pio device monitor
```

On boot the board will:

1. Run a colour self-test across all displays.
2. Attempt WiFi/Ethernet connection and display the IP for 4 seconds.
3. Show "Scoreboard" on the AtomS3 LCD with "tap to start AP" hint.
4. Start listening for commands on all enabled interfaces.

### 5. Send a test score

```bash
cd test
python test_osc_send.py 192.168.1.42   # replace with the board's IP
```

### 6. Use the web control panel

1. **Tap the AtomS3 button** — the device starts a WiFi access point.
2. Connect your phone/laptop to SSID **`Scoreboard`**, password **`12345678`**.
3. Open **http://192.168.4.1** in a browser — the full control panel loads.
4. Tap the button again to stop the AP and reconnect to your normal WiFi.

### 7. Use the desktop GUI

```bash
cd test
python gui_control.py          # auto-detects the serial port
```

The GUI provides sliders, colour pickers, preset save/load (JSON), and all the
same controls as the web interface.

---

## Command interfaces

All four interfaces accept the **same command syntax** — an OSC-style address
followed by arguments. Commands are delivered as:

| Interface      | Transport                           | Notes                          |
| -------------- | ----------------------------------- | ------------------------------ |
| **OSC/UDP**    | WiFi STA or Ethernet, port 9000     | Standard OSC binary messages   |
| **USB-Serial** | USB-CDC, 115200 baud                | Plain text, newline-terminated |
| **RS485**      | Serial2, 115200 baud (configurable) | Plain text, newline-terminated |
| **Web UI**     | HTTP GET to `/cmd?c=…`              | URL-encoded text commands      |

---

## Command reference

Display numbers are **1-based** in commands (mapped to 0-based internally).

### Per-display commands (`/display/<N>/…`)

| Address                                 | Arguments             | Description                                      |
| --------------------------------------- | --------------------- | ------------------------------------------------ |
| `/display/<N>`                          | `string` or `int`     | Set display text                                 |
| `/display/<N>/text`                     | `string`              | Set display text (alias)                         |
| `/display/<N>/mode`                     | `int` (0/1/2)         | Display mode: 0=text, 1=scroll up, 2=scroll down |
| `/display/<N>/color`                    | `int int int` (R G B) | Text colour                                      |
| `/display/<N>/brightness`               | `int` (0–255)         | Display brightness                               |
| `/display/<N>/clear`                    | —                     | Clear display                                    |
| `/display/<N>/scroll`                   | `int` (0/1/2)         | Scroll mode: 0=instant, 1=up, 2=down             |
| `/display/<N>/clearqueue`               | —                     | Discard pending scroll queue                     |
| `/display/<N>/text/enable`              | `int` (0/1)           | Enable/disable text layer                        |
| `/display/<N>/text/brightness`          | `int` (0–255)         | Text layer brightness                            |
| `/display/<N>/text/push`                | `string`              | Push text onto the text stack                    |
| `/display/<N>/text/pop`                 | —                     | Pop last text stack entry                        |
| `/display/<N>/text/set`                 | `int string`          | Set stack entry at index                         |
| `/display/<N>/text/clear`               | —                     | Clear text stack                                 |
| `/display/<N>/text/list`                | —                     | Print text stack to serial                       |
| `/display/<N>/particles/enable`         | `int` (0/1)           | Enable/disable particle layer                    |
| `/display/<N>/particles/brightness`     | `int` (0–255)         | Particle layer brightness                        |
| `/display/<N>/particles/color`          | `int int int` (R G B) | Particle colour                                  |
| `/display/<N>/particles`                | up to 16 args         | Full particle config (see below)                 |
| `/display/<N>/text2particles`           | —                     | Convert rendered text into particles             |
| `/display/<N>/particles/pause`          | `int` (0/1)           | Pause/resume particle physics                    |
| `/display/<N>/particles/transform`      | up to 5 floats        | Set view transform (angle° scaleX scaleY tx ty)  |
| `/display/<N>/particles/rotate`         | `float` (degrees)     | Set view rotation angle                          |
| `/display/<N>/particles/scale`          | 1 or 2 floats         | Set view scale (uniform or X Y)                  |
| `/display/<N>/particles/translate`      | `float float`         | Set view translation (tx ty)                     |
| `/display/<N>/particles/resettransform` | —                     | Reset view transform to identity                 |

### Global commands (apply to all displays)

| Address                     | Arguments             | Description                           |
| --------------------------- | --------------------- | ------------------------------------- |
| `/brightness`               | `int` (0–255)         | Global brightness                     |
| `/mode`                     | `int` (0/1/2)         | Set mode for all displays             |
| `/scroll`                   | `int` (0/1/2)         | Scroll mode for all displays          |
| `/scrollspeed`              | `int` (ms)            | Scroll speed per pixel step           |
| `/scrollcontinuous`         | `int` (0/1)           | Auto-cycle text stack in scroll mode  |
| `/text/enable`              | `int` (0/1)           | Text layer on all displays            |
| `/text/brightness`          | `int` (0–255)         | Text layer brightness (all)           |
| `/text/push`                | `string`              | Push to text stack (all)              |
| `/text/pop`                 | —                     | Pop text stack (all)                  |
| `/text/set`                 | `int string`          | Set stack entry (all)                 |
| `/text/clear`               | —                     | Clear text stack (all)                |
| `/text/list`                | —                     | Print text stacks to serial           |
| `/particles/enable`         | `int` (0/1)           | Particle layer on all displays        |
| `/particles/brightness`     | `int` (0–255)         | Particle brightness (all)             |
| `/particles/color`          | `int int int` (R G B) | Particle colour (all)                 |
| `/text2particles`           | —                     | Text → particles (all displays)       |
| `/particles/pause`          | `int` (0/1)           | Pause/resume physics (all)            |
| `/particles/transform`      | up to 5 floats        | Set view transform (all)              |
| `/particles/rotate`         | `float` (degrees)     | Set view rotation (all)               |
| `/particles/scale`          | 1 or 2 floats         | Set view scale (all)                  |
| `/particles/translate`      | `float float`         | Set view translation (all)            |
| `/particles/resettransform` | —                     | Reset view transform (all)            |
| `/defaults`                 | —                     | Reset all params to compiled defaults |
| `/clearqueue`               | —                     | Discard scroll queues (all)           |
| `/clearall` or `/clear`     | —                     | Clear every display                   |
| `/status`                   | —                     | Reply `ANIMATING 0` or `ANIMATING 1`  |
| `/save`                     | —                     | Force NVS save                        |
| `/rasterscan`               | —                     | Diagnostic raster scan pattern        |

### Particle configuration

The `/display/<N>/particles` command accepts up to 16 positional arguments.
Missing arguments keep their current value:

| #   | Type  | Parameter         | Default  |
| --- | ----- | ----------------- | -------- |
| 1   | int   | `count`           | 6        |
| 2   | int   | `renderMs`        | 20       |
| 3   | float | `gravityScale`    | 18.0     |
| 4   | float | `elasticity`      | 0.92     |
| 5   | float | `wallElasticity`  | 0.78     |
| 6   | float | `radius`          | 0.45     |
| 7   | int   | `renderStyle`     | 4 (glow) |
| 8   | float | `glowSigma`       | 1.2      |
| 9   | float | `temperature`     | 0.0      |
| 10  | float | `attractStrength` | 0.0      |
| 11  | float | `attractRange`    | 3.0      |
| 12  | int   | `gravityEnabled`  | 1        |
| 13  | int   | `substepMs`       | 20       |
| 14  | float | `damping`         | 0.9998   |
| 15  | float | `glowWavelength`  | 0.0      |
| 16  | int   | `speedColor`      | 0        |

**Render styles:** 0=point, 1=square, 2=circle, 3=text, 4=glow

Full command reference: [docs/command_reference.md](docs/command_reference.md)  
OSC-specific protocol details: [docs/osc_protocol.md](docs/osc_protocol.md)

---

## Layer architecture

Each display composites two independent layers:

```
┌─────────────────────────────┐
│  Text layer (foreground)    │  enable, brightness, colour
├─────────────────────────────┤
│  Particle layer (background)│  enable, brightness, colour
└─────────────────────────────┘
```

- **Particles** render first, clearing the canvas.
- **Text** overlays on top — only non-black text pixels are drawn.
- Each layer has its own **enable** toggle, **brightness** (0–255), and **colour**.
- Disabling a layer makes it fully transparent (zero cost when off).
- All layer settings are saved to NVS and restored on boot.

---

## Configuration

All compile-time tunables live in [src/config.h](src/config.h).
Most can be overridden via `-D` build flags in `platformio.ini`.

| Define               | Default             | Description                                 |
| -------------------- | ------------------- | ------------------------------------------- |
| `NEOPIXEL_PIN`       | `2` (env-dependent) | GPIO for NeoPixel data                      |
| `NUM_DISPLAYS`       | `6`                 | Number of 32×8 tiles                        |
| `MATRIX_TILE_WIDTH`  | `32`                | Pixels per tile (width)                     |
| `MATRIX_TILE_HEIGHT` | `8`                 | Pixels per tile (height)                    |
| `DEFAULT_BRIGHTNESS` | `10`                | Start-up brightness (0–255)                 |
| `OSC_PORT`           | `9000`              | UDP port for OSC                            |
| `SCROLL_STEP_MS`     | `50`                | Default ms per scroll step                  |
| `SCROLL_QUEUE_SIZE`  | `10`                | Max queued scroll items per display         |
| `TEXT_STACK_MAX`     | `8`                 | Max entries in each text stack              |
| `TEXT_MAX_LEN`       | `32`                | Max characters per text entry               |
| `DEAD_LED_INDEX`     | `172`               | Raw strip index of dead LED (−1 to disable) |
| `SERIAL_CMD_ENABLED` | `1`                 | Enable USB-Serial command interface         |
| `MATRIX_LAYOUT`      | see config.h        | NeoMatrix wiring flags                      |

> If your panels show garbled output, you likely need to change
> `MATRIX_LAYOUT` (row vs. column, progressive vs. zigzag).
> See the [Adafruit NeoMatrix guide](https://learn.adafruit.com/adafruit-neopixel-uberguide/neomatrix-library).

---

## Build environments

Defined in [platformio.ini](platformio.ini):

| Environment             | Board          | Network          | NeoPixel GPIO | Extras            |
| ----------------------- | -------------- | ---------------- | ------------- | ----------------- |
| `atoms3-wifi` (default) | M5Stack AtomS3 | WiFi STA         | GPIO 1        | M5Unified, IMU    |
| `atoms3-ethernet`       | M5Stack AtomS3 | Ethernet W5500   | GPIO 5        | PoE support       |
| `atoms3-wifi-rs485`     | M5Stack AtomS3 | WiFi STA + RS485 | GPIO 1        | RS485 on GPIO 5/6 |
| `esp32dev-wifi`         | Generic ESP32  | WiFi STA         | GPIO 2        | No IMU/LCD        |

---

## NVS persistence

Display settings are automatically saved to ESP32 Non-Volatile Storage (NVS)
and restored on boot. Saved state includes (per display):

- Text stack contents
- Text colour and particle colour
- Text brightness and particle brightness
- Text layer enable and particle layer enable
- Display mode and scroll settings
- Particle physics configuration

The `/save` command forces an immediate NVS write. Settings are also saved
automatically when changed. The storage uses a versioned format (currently v4).

---

## Project structure

```
robot_game_scoreboard/
├── platformio.ini              Build configuration (4 environments)
├── README.md                   This file
├── experiments/
│   ├── journal/
│   │   └── Journal.md          Development journal
│   └── validation/             Ad hoc validation and hardware-check scripts
├── animations/                 Authored `.anim` demo and preset source files
├── src/
│   ├── main.cpp                Entry point — setup, loop, AP toggle, LCD screens
│   ├── config.h                All compile-time settings
│   ├── credentials.h.example   Template for WiFi credentials
│   ├── credentials.h           Your WiFi credentials (git-ignored)
│   ├── DisplayManager.h/cpp    NeoMatrix wrapper, per-display state, NVS save/load
│   ├── OSCHandler.h/cpp        OSC/UDP + Serial + RS485 + Web command dispatch
│   ├── VirtualDisplay.h/cpp    Display modes, scroll, text stack, layer compositing
│   ├── ParticleSystem.h/cpp    2D particle physics engine (Velocity Verlet)
│   ├── Vec2f.h                 2D vector math utilities
│   └── WebInterface.h/cpp      WiFi AP mode, HTTP server, HTML control page
├── docs/
│   ├── hardware_setup.md       Wiring diagrams & power notes
│   ├── osc_protocol.md         OSC protocol details
│   └── command_reference.md    Full command reference
├── test/
│   ├── gui_control.py          Desktop GUI (Python/tkinter over serial)
│   ├── test_osc_send.py        OSC test script
│   ├── test_serial_send.py     Serial test script
│   ├── demo_countdown.py       Countdown demo script
│   ├── default_config.json     Default preset for GUI
│   ├── interference1.json      Wave interference preset
│   └── liquid1.json            Liquid simulation preset
├── include/                    (reserved for shared headers)
└── lib/                        (reserved for local libraries)
```

---

## Future ideas

- [ ] mDNS (`scoreboard.local`)
- [ ] OTA firmware updates
- [ ] Custom large-pixel digit font (full 8 px height)
- [ ] Additional particle render styles (trails, springs)
- [ ] Multi-device synchronization over RS485

---

## License

MIT
