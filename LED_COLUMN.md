# LED Arena System Documentation

## 1. Overview

This project consists of an LED-based visualization system for a robotic game arena.

- 8 vertical light columns arranged around an arena
- Each column contains 4 LED strips
- LED strips are controlled via RS485 LED controllers
- Each controller drives 2 outputs (DAT + CLK used as identical outputs)

Primary use:
- Real-time team visualization (Red vs Blue)
- Score display via vertical fill animations
- Dynamic animations (waves, wipes, growth effects)

---

## 2. Physical Layout

### Column Numbering (Spatial Mapping)

The columns are arranged around the arena and numbered for **logical animation flow (left → right)**:

[2] Top Left [4] Top Center [6] Top Right
[1] Middle Left [ ] Arena [8] Middle Right
[3] Bottom Left [5] Bottom Center [7] Bottom Right


| Column ID | Position        |
|----------|-----------------|
| 1        | Middle Left     |
| 2        | Top Left        |
| 3        | Bottom Left     |
| 4        | Top Center      |
| 5        | Bottom Center   |
| 6        | Top Right       |
| 7        | Bottom Right    |
| 8        | Middle Right    |

This ordering is intentional to support:
- Left → Right wave effects
- Sequential animation logic

---

## 3. LED Strip Configuration

Each column:

- 4 LED strips total
- Grouped into **2 logical pairs**
- Each pair is driven by **one controller (2 outputs)**

### Important Hardware Detail

Although the controller has:
- DAT (data)
- CLK (clock)

For WS2811 strips:

> DAT and CLK outputs are electrically identical (confirmed by testing)

Therefore:

- Treat both outputs as **independent identical data channels**
- Ignore "clock" semantics (not used in WS2811)

---

## 4. Digital Numbering Scheme

To simplify animation logic, LED strips are numbered sequentially:

### Per Column

Each column has:

Column N:
Strip N1 → Controller channel 1 (DAT)
Strip N2 → Controller channel 2 (CLK)


Example:

| Column | Strip IDs | Controller |
|--------|----------|------------|
| 1      | 11, 12   | Controller 1 |
| 2      | 21, 22   | Controller 2 |
| ...    | ...      | ... |
| 8      | 81, 82   | Controller 8 |

This enables:

- Easy iteration: `for column in 1..8`
- Clean animation mapping
- Predictable spatial sequencing

---

## 5. LED Characteristics

- Type: WS2811 (12V)
- Length: ~1.0 m per strip
- Addressable units: **28 segments per strip**
- The 12V WS2811 strips group LEDs in sets of 3.

---

### LED Orientation

- First LED is located at the **bottom of the column**
- Data flows **bottom → top**

This is critical for:

- Growth animations
- Score visualization

---

## 6. Power Wiring

### LED Power (4 Strips)

- Red → +12V
- White → GND

### Controller Power

- VIN → +12V
- GND → GND

### Notes

- Each 1 meter LED strips consumes 20W when fully turned on. Each column (4 strips) requires a **80W power supply** (12A 6.7A)

---

## 7. Controller Wiring

### LED Output (Controller → LED Strip)

Example 

Channel 1 DAT → Strip 11 (Green)
Channel 1 CLK → Strip 11 (Green)
Channel 2 DAT → Strip 12 (Green)
Channel 2 CLK → Strip 12 (Green)


Even though labeled differently:

> Both outputs behave identically

---

### RS485 Communication Buses (Dual-Bus Architecture)

The 8 controllers are split across **two RS485 buses** for improved throughput.
Each bus is served by its own USB-to-RS485 adapter on the control PC.

```
Control PC
├── USB → RS485 Adapter A  (Bus A)
│   ├── Controller 1
│   ├── Controller 2
│   ├── Controller 3
│   └── Controller 4
│
└── USB → RS485 Adapter B  (Bus B)
    ├── Controller 5
    ├── Controller 6
    ├── Controller 7
    └── Controller 8
```

> The exact grouping is flexible.  The software auto-discovers which
> controllers are on which bus by probing device addresses on each port.

#### Auto-Discovery

The discovery system tracks which controller addresses (1–8) are
**missing** and actively searches for them:

1. On start-up, scan all candidate USB-RS485 COM ports (by VID/PID)
   and send a **query-device-address** command (function `0x94`) to
   addresses 1–8 with a 300 ms read timeout.
2. A background thread runs every 5 seconds:
   - **Quick-probe** missing addresses on already-open buses (20 ms
     timeout — does not block the animation sender thread).
   - If still missing, look for newly-plugged USB-RS485 adapters,
     open them, and full-probe for the missing addresses only.
   - Close any bus that ends up with zero controllers.
3. Ports are only kept open while they have at least one controller.
4. If a controller is moved to a different bus, the next probe round
   detects it on the new port and updates the mapping automatically.

#### Probe Command (Function 0x94 — Query Device Address)

```
DD 55 EE  00 00  00 <addr>  00  94  02  00 00  00 03  00 01  00 00 00  AA BB
```

- 21 bytes, same frame structure as display commands.
- `<addr>` = device address to query (01–08).
- Reply when present (15 bytes, two lines):
  ```
  RecvEnd\r\n
  0001\r\n
  ```
  The second line echoes the 4-digit hex device address.
- No reply when absent (timeout).
- Any response containing `\r\n` is treated as proof of a live
  controller; bus noise (stray `0xFF`) never contains `\r\n`.

See also: *LED Communication Manual.pdf*, section 2.5.10 (function 0x94).

#### Supported USB-RS485 Adapter Chipsets

| Chip            | VID    | PID    |
|-----------------|--------|--------|
| FTDI FT232R     | 0x0403 | 0x6001 |
| FTDI FT-X       | 0x0403 | 0x6015 |
| Silicon Labs CP210x | 0x10C4 | 0xEA60 |
| Prolific PL2303 | 0x067B | 0x2303 |
| CH340           | 0x1A86 | 0x7522 |
| CH341           | 0x1A86 | 0x7523 |
| CH343           | 0x1A86 | 0x55D3 |

> CH340/CH341 adapters overlap with the haptic ESP32 controllers.
> The software differentiates them by probing the LED protocol.


### Wiring Convention

| Signal | Cable Color |
|--------|------------|
| A      | Green      |
| B      | Blue       |

---

## 8. Addressing Strategy

Each controller must have a **unique device address** (across all buses).

| Column | Controller Address |
|--------|-------------------|
| 1      | 1 |
| 2      | 2 |
| ...    | ... |
| 8      | 8 |

This allows:

- Individual control per column
- Broadcast control (all columns)
- Group-based animations (future extension)
- **Automatic bus routing** — the software discovers which address
  lives on which bus and routes commands accordingly

---

## 9. Logical Control Model

### Hierarchy

Arena
├── Column (1–8)
│ ├── Strip A (x2)
│ └── Strip B (x2)
│ 	└── LED index (0–27)

## 10. Planned Visual Behaviors

| Behavior Name        | Description                                      | Visual Pattern                     | Parameters                         | Use Case                          |
|---------------------|--------------------------------------------------|------------------------------------|------------------------------------|-----------------------------------|
| Full Column Color   | Entire column is set to a single color           | Bottom → Top uniform               | column_id, color                   | Team indication (Red / Blue)      |
| Partial Fill        | Column fills from bottom to a certain height     | Bottom → Top proportional fill     | column_id, height, color           | Score visualization               |
| Split Column        | Column divided into two color regions            | Bottom = color A, Top = color B    | column_id, split_index, color A/B  | Compare teams in same column      |
| Growth Animation    | Color grows upward over time                     | Animated bottom → top progression  | column_id, speed, target_height    | Score counting animation          |
| Wave Effect         | Color propagates across columns                  | Column 1 → 8 sequential activation | color, direction, speed            | Game start / transitions          |
| Color Wipe          | One color replaces another over time             | Progressive overwrite              | from_color, to_color, speed        | Win/lose transitions              |

## 14. Command Examples (RS485 Protocol)

This section documents example commands generated using the manufacturer’s software.
These can be used as reference for implementing custom control software.

---

### 14.1 Command Structure Reminder

Each command follows the format:
[Header][Group][Device][Port][Function][LED Type][Reserved][Length][Repeat][Color Data][Tail]

Key fields:

| Field         | Value (Example) | Meaning                         |
|--------------|----------------|---------------------------------|
| Header        | DD 55 EE       | Fixed start bytes               |
| Group Addr    | 00 00          | Broadcast to all groups         |
| Device Addr   | 00 01          | Target controller (ID = 1)      |
| Port          | 01 / 02        | Channel selection               |
| Function      | 99             | Display color data              |
| LED Type      | 02             | WS2811 (confirmed from tool)    |
| Length        | 00 54          | 84 bytes (28 LEDs × 3 RGB)      |
| Tail          | AA BB          | Fixed end bytes                 |

---

### 14.2 Example: Channel 1 → Full Red

This command sets **all LEDs on Channel 1 (Port 01)** to red.

DD 55 EE
00 00
00 01
01
99
02
00 00
00 54
00 01
FF 00 00 FF 00 00 FF 00 00 FF 00 00 FF 00 00
FF 00 00 FF 00 00 FF 00 00 FF 00 00 FF 00 00
FF 00 00 FF 00 00 FF 00 00 FF 00 00 FF 00 00
FF 00 00 FF 00 00 FF 00 00 FF 00 00 FF 00 00
FF 00 00 FF 00 00 FF 00 00 FF 00 00
AA BB

#### Interpretation

- Device: **Controller 1**
- Port: **Channel 1 (DAT output)**
- Color: **Red (R=255, G=0, B=0)**
- LEDs: **28 (all set uniformly)**


### 14.3 Example: Channel 2 → Full Blue

This command sets **all LEDs on Channel 2 (Port 02)** to blue.

DD 55 EE
00 00
00 01
02
99
02
00 00
00 54
00 01
00 00 FF 00 00 FF 00 00 FF 00 00 FF 00 00 FF
00 00 FF 00 00 FF 00 00 FF 00 00 FF 00 00 FF
00 00 FF 00 00 FF 00 00 FF 00 00 FF 00 00 FF
00 00 FF 00 00 FF 00 00 FF 00 00 FF 00 00 FF
00 00 FF 00 00 FF 00 00 FF 00 00 FF
AA BB

#### Interpretation

- Device: **Controller 1**
- Port: **Channel 2 (CLK output used as data)**
- Color: **Blue (R=0, G=0, B=255)**
- LEDs: **28 (all set uniformly)**

---

## 15. Validated Timing Constraints (Windows + RS485)

These values were validated on the current test setup (single controller, strips 11/12):

- `baudrate = 115200` works reliably.
- Flash tests pass reliably with inter-frame delay `>= 7 ms`.
- At `6 ms`, flashing becomes unreliable (frames appear dropped/ignored).

### Why 6 ms Can Fail

Each full-strip command is approximately 102 bytes on the wire.

- UART 8N1 sends 10 bits per byte.
- Wire time per frame is therefore:

`102 bytes * 10 bits / 115200 bps = 8.85 ms`

So practical command spacing should include:

- `~8.85 ms` transmission time
- plus extra controller parse/processing margin

This makes `>= 7 ms` additional delay a practical floor on this setup, and `10 ms` a safer default.

### Animation Planning Implication

For per-command delay `d` milliseconds:

`command_period_ms ~= 8.85 + d`

At `d = 10 ms`, command period is about `18.85 ms`, giving:

- maximum command rate `~53 Hz`
- for 2 strips updated each frame, max frame rate `~26 FPS`

Recommendation:

- Use `10 ms` inter-command delay as default for reliability.
- Avoid assuming 50 FPS for multi-strip updates at 115200 baud.
- Prefer updating only changed strips per frame where possible.

### Note on Practical 35 FPS Observation

In tests, around 35 FPS was observed for dual-strip animation despite the conservative
`~26.5 FPS` estimate above.

Reason: there are two pacing models:

- **between-all-commands** (safer): delay is enforced after every command
	- For 2 strips with 10 ms gap: about `26.5 FPS` max
- **between-strips-only** (faster): delay only between A/B commands within one frame
	- For 2 strips with 10 ms gap: about `35.9 FPS` max

The higher observed value matches the **between-strips-only** model.
For production reliability, prefer **between-all-commands** unless additional testing
proves long-run stability at the faster mode.

