# Ultrasonic sensing in `car_Llevel.ino` (discussion notes)

This document describes **how the three HC-SR04-style ultrasonics are read today**: async, round-robin, no blocking `pulseIn`-style loop in the main PID path.

---

## 1. Big picture

- **Hardware**: Three sensors (Right, Left, Front). Each has a **TRIG** output from the MCU and an **ECHO** input. The MCU pulls TRIG high for a short time; the module sends a pulse on **ECHO** whose length is proportional to round-trip time → distance.
- **Software idea**: A small **finite state machine (FSM)** runs inside **`ultrasonic_service()`**. You call that function often (from **`myDelayMs()`**, **`goBackwardMs()`**, obstacle wait loops, etc.). It advances **one step per call** (non-blocking).
- **Only one sensor is “active” at a time** (one measurement in flight). That reduces electrical/acoustic crosstalk between modules.
- **Results** are stored in three buffers: last distance (cm) and last sample time (ms) per channel. The rest of the code **reads those buffers**; it does not trigger a fresh blocking read.

---

## 2. Pin / data model

### `UltrasonicSensor` struct

Each sensor is described by:

| Field       | Meaning |
|------------|---------|
| `trigPort` | Port register used to drive TRIG (e.g. `PORTD`, `PORTC`) |
| `echoPin`  | Input register to read ECHO (e.g. `PIND`, `PINC`) |
| `trigBit`  | Which bit on that port is TRIG |
| `echoBit`  | Which bit is ECHO |

The three constants are **`SENSOR_RIGHT`**, **`SENSOR_LEFT`**, **`SENSOR_FRONT`** (see the top of the `.ino` for exact Arduino pins).

### Channel index (important for reading)

| Index | Sensor | Typical use in code |
|-------|--------|---------------------|
| `0`   | Right  | `us_last_cm[0]`, wall on the right |
| `1`   | Left   | `us_last_cm[1]` |
| `2`   | Front  | `us_last_cm[2]`, obstacle ahead |

---

## 3. Constants (tuning knobs)

| Name | Value | Role |
|------|--------|------|
| `US_NUM_CH` | 3 | Number of sensors |
| `US_COOLDOWN_MS` | 20 ms | After a channel finishes (success or timeout), that channel **cannot start a new ping** until this much time has passed. Gives the module “breathing room” (HC-SR04 often quoted ~60 ms max rate; 20 ms is a softer spacing). |
| `US_ECHO_TIMEOUT_US` | 30 000 µs (30 ms) | If the echo line does not behave as expected within this time, the measurement **fails** and distance is stored as **0**. |
| `SPEED_OF_SOUND` | 0.0343 (cm/µs) | Converts echo pulse width (µs) to **cm** (one-way distance uses half of round-trip). |

---

## 4. What each function does

### `us_sensor_for_ch(ch)`

- **Input**: `ch` = 0, 1, or 2.
- **Output**: Pointer to `SENSOR_RIGHT`, `SENSOR_LEFT`, or `SENSOR_FRONT`.
- **Purpose**: Map channel index → hardware descriptor.

### `ultrasonic_init()`

- Called once from **`setup()`** (after timers are configured).
- Resets FSM to **IDLE**, clears `usm_cfg`, zeroes **`us_last_cm[]`**, **`us_last_sample_ms[]`**, **`us_next_fire_ms[]`**, and sets round-robin pointer **`usm_rr`** to start at channel 0.

### `usm_complete(ch, cm)`

- **Internal** helper when a measurement **finishes** (good or bad).
- Writes **`us_last_cm[ch] = cm`** (0 on timeout / error).
- Sets **`us_last_sample_ms[ch]`** to current **`myMillis()`** (so other code can tell how fresh the reading is).
- Sets **`us_next_fire_ms[ch] = now + US_COOLDOWN_MS`** so that channel waits before the next ping.
- Advances **`usm_rr = (ch + 1) % 3`** so the next preferred channel rotates **Right → Left → Front → Right → …**
- Puts FSM back to **IDLE** and clears `usm_cfg`.

### `usm_try_start()`

- Called only when FSM is **IDLE**.
- Tries to start a new measurement on **one** channel:
  - It looks at channels **`usm_rr`, `usm_rr+1`, `usm_rr+2`** (mod 3) in order.
  - Picks the **first** whose **`us_next_fire_ms`** has passed (cooldown finished).
- If none is ready yet, it does nothing (stays IDLE until a later **`ultrasonic_service()`** call).
- If it starts: saves **`usm_active_ch`**, **`usm_cfg`**, **`usm_echo_mask`**, pulls **TRIG low**, enters **`USM_TRIG_LOW_HOLD`**, and sets a **2 µs** deadline using **`myMicros()`**.

### `ultrasonic_service()`

- **This is the heart of the driver.** Call it as often as possible while the CPU would otherwise spin in delays.
- **One call** = at most **one transition** or **one check** of the current phase (no long blocking wait inside).

#### FSM phases (in order for one successful ping)

1. **`USM_IDLE`**  
   Calls **`usm_try_start()`** to maybe begin a new ping.

2. **`USM_TRIG_LOW_HOLD`**  
   Wait until **≥ 2 µs** have passed with TRIG low (HC-SR04 wants a clean low before the pulse).

3. **`USM_TRIG_HIGH_HOLD`**  
   TRIG high; wait until **≥ 10 µs** (minimum trigger pulse width in the datasheet sense).

4. **`USM_WAIT_ECHO_PRELOW`**  
   Wait until the **ECHO line is LOW** (clears any leftover high from a previous pulse). If it stays HIGH longer than **30 ms** → timeout, **`usm_complete(..., 0)`**.

5. **`USM_WAIT_ECHO_HIGH`**  
   Wait until **ECHO goes HIGH** (start of measured pulse). If it never does within **30 ms** → timeout, distance 0.

6. **`USM_WAIT_ECHO_FALL`**  
   Wait until **ECHO goes LOW** again (end of pulse). Record **`dur = myMicros() - pulse_start`**, convert to **cm**, **`usm_complete(ch, cm)`**. If HIGH lasts too long → timeout, 0 cm.

Trigger timing uses **`myMicros()`** deadlines (not spinning on `TCNT1L` alone), so it stays consistent with **Timer1** used for PWM and timekeeping.

### `myDelayMs(ms)` (relevant part)

- While waiting for **ms** milliseconds, it repeatedly calls **`ultrasonic_service()`** in the wait loop.
- So **any** code that uses **`myDelayMs`** (warmup, PID period, turns, setup banner delay, etc.) **keeps advancing the ultrasonic FSM** instead of freezing sensor updates.

### `delayUsMyMicros(us)` (related, not the FSM)

- Short busy-wait using **`myMicros()`** (e.g. motor direction settle in **`goBackwardMs`**). Not used for the HC-SR04 trigger path in the FSM.

---

## 5. How the rest of the program uses the data

| Location | Behaviour |
|----------|-----------|
| **Warmup** | Copies **`us_last_cm[0]`** / **`[1]`** into **`lastDistR`** / **`lastDistL`** when positive, inside loops that use **`myDelayMs(10)`** (which services the FSM). |
| **Main PID loop** | Reads **`distRight = us_last_cm[0]`**, **`distLeft = us_last_cm[1]`**, **`distFront = us_last_cm[2]`** — these are the **latest completed** measurements, not necessarily from the same millisecond. |
| **Obstacle re-check** | After stopping and a settle delay, sets a **`mark`** time and spins (up to ~350 ms) calling **`ultrasonic_service()`** until **all three** **`us_last_sample_ms[i] >= mark`** (fresh triplet) or timeout; then reads **`us_last_cm[]`** again. |
| **`goBackwardMs`** | Long reverse wait loop also calls **`ultrasonic_service()`** so sensors can still update during long reverse. |

---

## 6. Round-robin in one sentence

After each finished measurement on channel **`ch`**, the preferred next channel is **`(ch + 1) % 3`**; when starting from IDLE, the code picks the **first channel in that rotation whose cooldown has expired**, so no sensor is starved forever if one channel is waiting out its 20 ms.

---

## 7. What “0 cm” means

- **Timeout** on any echo phase, or **no valid pulse** → stored distance is **0**.
- PID / logic often treats **`> 0`** as “reading valid for that check” (same idea as before with blocking reads).

---

## 8. Time bases (for the discussion)

- **`myMicros()`**: from **Timer1** (overflow ISR + counter); used for **µs** trigger deadlines and echo pulse width.
- **`myMillis()`**: from **Timer0** CTC 1 ms interrupt; used for **cooldowns** and **sample timestamps**.

---

## 9. Optional discussion points

- **Staleness**: R, L, F are not guaranteed sampled in the same microsecond; they are **last completed** values, spaced by measurement time + cooldown + round order. For “synchronous snapshot” you would need a different design (e.g. one-shot all TRIGs together — not what this code does).
- **Tuning**: Increase **`US_COOLDOWN_MS`** if you see crosstalk or unstable readings; decrease carefully (HC-SR04 max rate limits).
- **UART**: Blocking **`uart_read_blocking`** does **not** call **`ultrasonic_service()`**; long serial handling can delay FSM progress slightly compared to delay loops.

---

*File: [`car_Llevel.ino`](car_Llevel.ino) — ultrasonic block is under the section comment `ULTRASONIC (async round-robin, non-blocking FSM)`.*
