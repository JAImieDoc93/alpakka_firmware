# Adaptive gyro stabiliser (Alpakka v0, custom firmware)

An optional One Euro filter that reduces hand shake in gyro mouse aiming. It is a
small change on top of upstream firmware **1.2.0** (commit
`6bfa7755910abe1715ff5a5a8fd0f9176d798358`), on branch `gyro-stabiliser`.

> **Not hardware-tested.** Every number below comes from host simulations
> (`tests/`). The firmware compiles cleanly but has not run on a real controller.

## What it does

- **Where it runs:** after gyro calibration and the dual-IMU blend, and before
  upstream's low-speed curve (`hssnf`), per-axis sensitivity and HID reporting.
  Calibration, sensitivity presets, axis scaling, buttons and HID reports are all
  unchanged.
- **How it filters:** each gyro axis (X, Y, Z) has its own One Euro filter. The
  filter works on the camera *angle* and uses the gyro reading as the speed.
  - The cutoff is `min_cutoff + beta × speed`. Hand shake (slow) is smoothed
    hard; deliberate movement (fast) is smoothed less.
  - `speed` is |angular velocity| low-passed at `d_cutoff`.
  - Each step uses the measured time step, clamped to 2–8 ms.
  - There is no sample buffer. Each axis stores 3 values.
- **Total rotation is preserved.** The filter delays rotation but never removes
  it: any rotation held back is output as the motion slows down. Your RWS and 360°
  distance stay the same. (A velocity-domain version was tried first and lost
  0.5–7% of rotation after flicks, so it was rejected.)
- **Fast-motion bypass:** between 75% and 100% of the bypass speed, the output
  blends linearly into the raw gyro. The test runs show no jump entering or leaving
  bypass. Tests showed the bypass is **necessary**: without it a 90° flick lags up
  to 7–10° mid-flick.
- **Engage and disengage:** when gyro aiming turns on, the filter starts fresh from
  the current motion. When it turns off, any rotation still held back is
  **discarded**, so the camera never moves after you release. The cost is up to
  about 0.6° (BALANCED) of rotation lost if you release mid-pan (STRONG: about 0.003°).
  - The filter also resets if the gyro isn't read for more than 20 ms (for
    example, while Home is held) and when you change mode.
- **OFF mode** follows the original code path exactly.

| Parameter | LOW_LATENCY | BALANCED | STRONG |
|---|---|---|---|
| min_cutoff | 8 Hz | 5 Hz | 2 Hz |
| beta | 0.10 Hz per °/s | 0.08 Hz per °/s | 50 Hz per °/s (of gated speed) |
| d_cutoff | 2 Hz | 1 Hz | 10 Hz |
| bypass | 120 °/s | 120 °/s | 180 °/s |
| tremor_k | 0 (off) | 0 (off) | 3.0 |
| threshold_dps | – | – | 0 |
| micro deadzone | none | none | none |

These parameters are in `src/gyro_filter.c` (`params[]`).

**STRONG uses a tremor gate (since `v1.2.0-gyrostab.2`; retuned in `.3`).**
The old STRONG (3 Hz / 0.05 / 1 Hz) added about 31 ms to small corrections.
The new one:
- drives the cutoff from the low-passed **signed** velocity (10 Hz). Tremor
  swings back and forth and averages out; a deliberate correction does not.
- measures your tremor level (RMS of the velocity around that average, 2 Hz,
  each sample capped at 30 °/s) and subtracts 3 × that. So the gate adapts to
  how shaky your hand is, and shake alone doesn't open it.
- protects the tremor level from flicks: it remembers the level from the last
  moment you were below 30 °/s. Above 90 °/s it restores that level and holds
  it for 150 ms after. So a flick doesn't make the next correction slow, but a
  button-press jolt (under 90 °/s) still raises it and gets smoothed.
- Result: 2 Hz cutoff (heavy smoothing) while holding still; the cutoff jumps
  to tens of Hz within a tick or two once a real correction starts.
- The constants are `TREMOR_*` in `gyro_filter.c`.
- **gyrostab.2 → .3:** .2 (1 Hz / 6.84 / 4 Hz, k 1, threshold 1) still lagged
  0.3–0.5° corrections by 19–27 ms. .3 gets every correction under 15 ms,
  at the cost of less still-hand smoothing (−65% instead of −78%).

## Switching modes

**On the controller (runtime):**
1. Hold **Home**.
2. Long-press the **D-Hat push** to open the stabiliser menu.
3. While still holding Home, scroll the **wheel** to change mode. Scrolling up
   steps OFF → LOW_LATENCY → BALANCED → STRONG; scrolling down goes back.

The LEDs show the current mode:

| Mode | LEDs |
|---|---|
| OFF | bottom LED blinks |
| LOW_LATENCY | top LED blinks |
| BALANCED | left and right LEDs blink |
| STRONG | all four LEDs blink |

- The choice is saved, so it survives a power cycle.
- A short press of D-Hat push in the home menu still sends its original key, but
  it now fires on release instead of on press.

**Build-time default:** the setting has a default that applies until you pick a
mode on the controller. There is one `.uf2` per default:

- `alpakka_v0_1.2.0_gyrostab_default-off.uf2` (recommended: behaves like stock
  until you turn the stabiliser on)
- `..._default-low-latency.uf2`
- `..._default-balanced.uf2`
- `..._default-strong.uf2`

**How the setting is stored:**
- It is one byte (`Config.gyro_stab`, offset 173) inside the config block's unused
  padding, which already holds 0. 0 means "use the build default", so no config
  version change was needed.
- Stock firmware ignores this byte.
- The home menu entry is added in RAM when the home profile loads, and only if
  D-Hat push has no secondary action. Your stored profiles are not rewritten. On
  newly reset home profiles, `profiles/home.c` adds the same entry.

## Test results (host simulation, 250 Hz, same `gyro_filter.c` as the firmware)

Full tables are in `tests/results.md`. To reproduce: `make -C tests run`.

| Metric | OFF (stock) | BALANCED | STRONG | LOW_LATENCY |
|---|---|---|---|---|
| Still-hand angle jitter RMS (4–12 Hz tremor) | 0.041° | 0.025° (−38%) | 0.014° (−65%) | 0.031° (−25%) |
| Jitter reduction at 8 °/s tremor (very shaky) | – | −36% | −65% | −23% |
| Button-press jolt, peak camera excursion | 0.63° | 0.36° (−43%) | 0.20° (−68%) | 0.44° (−30%) |
| Micro-adjust (1.5° in 150 ms): delay | 0 ms | 22 ms | 0–1 ms | 15 ms |
| Corrections 1°–5°: delay | 0 ms | 22–28 ms | 0–1.5 ms | 15–17 ms |
| Tiny corrections 0.3° / 0.5°: delay | 0 ms | 24 / 22 ms | 10 / 3 ms | 19 / 14 ms |
| 1.5° correction 50–400 ms after a 90° flick: delay | 0 ms | 9–19 ms | 0 ms | 9–15 ms |
| Slow tracking: delay / RMS angle error | 13 ms† / 0.08° | 38 ms / 0.30° | 14 ms / 0.09° | 29 ms / 0.21° |
| Velocity step 0→20 °/s, 63% rise | 4 ms | 32 ms | 28 ms | 24 ms |
| Velocity step 0→200 °/s (bypass) | 4 ms | 4 ms | 4 ms | 4 ms |
| 90° flick in 200 ms: max lag / lag after 0.5 s | 0 | 0.60° / 0.009° | 0.57° / 0.017° | 0.53° / 0.006° |
| Total rotation, 60 s of mixed aiming | 0 | 0.0000% | 0.0000% | 0.0000% |
| Rotation lost on release mid-pan at 30 °/s | 0 | 0.65° | 0.003° | 0.43° |
| Small-signal delay at 1 / 10 Hz (2 °/s sine) | 0 | 31 / 16 ms | 3 / 20 ms | 20 / 13 ms |
| Slow steady pan 1–8 °/s: lag (2 / 8 °/s tremor) | 0 | 29–36 / 30–47 ms | 1–71 / 32–97 ms | 19–24 / 20–36 ms |
| Fuzz (NaN, Inf, bad dt, 200k samples) | – | PASS | PASS | PASS |

The old STRONG (v1) is still in `tests/results.md` as "STRONG v1 (old)" for
comparison: −54% jitter, 31 ms micro-adjust delay, 54 ms tracking delay.

† The tremor in the "intent" signal is uncorrelated, so even raw shows a small
measured delay.

**CPU cost (estimate, not measured):** per tick, about 3 × 35 software float
operations plus 12 double operations (STRONG: about 3 × 50 plus 3 `sqrtf`).
That is roughly 7–11k cycles, or about 60–90 µs of the 4,000 µs tick (~2%).
The 128 SPI gyro reads per tick are far more expensive. No heap is used; the
state is 3 × 20 bytes plus 16 bytes.

**What the numbers mean.** The filter is not free.
- BALANCED adds about 20–40 ms of delay to slow, small movements. STRONG adds
  under 15 ms to every correction tested (0.3°–5°, worst 10 ms), and nothing
  to corrections right after a flick.
- STRONG's cost is **very slow, steady pans while shaking**: a 1–2 °/s pan
  trails by 40–100 ms (old STRONG: 54–67 ms), because shake hides it and the
  2 Hz floor smooths it. Faster pans (4–8 °/s) trail much less. Raise
  `min_cutoff` (3 Hz) if slow tracking feels behind; that cuts smoothing to
  about −55%.
- Fast movements pass through with essentially no delay.
- Delay at small movements is the cost of shake reduction: a first-order filter
  that cuts 8–10 Hz tremor by half delays low-frequency motion by
  1/(2π·fc), about 30 ms at 5 Hz.
- If BALANCED feels floaty, LOW_LATENCY keeps about two thirds of the benefit
  with about half the delay.

## COD tuning guide (RWS 6)

The stabiliser doesn't change sensitivity, so keep RWS 6 and your in-game
settings. Then:

1. Start with **LOW_LATENCY**. Hold on a distant, still target and compare the
   wobble against OFF (switch with Home + D-Hat push + wheel).
2. **Still shaky:** step up to BALANCED. **Aim feels late or floaty** on
   micro-corrections: step back down, or raise `min_cutoff` (7–10 Hz) in
   `params[]` and rebuild.
3. **Still too shaky** when ADS-holding: try STRONG. With the tremor gate it
   should respond to corrections about as fast as OFF. If it still wobbles,
   lower `min_cutoff` (1.5 Hz: −71%, slow pans trail up to ~125 ms). If slow
   tracking feels behind, raise it (3 Hz). If corrections feel sticky, lower
   `tremor_k` (2.5).
4. **Flicks feel sticky or rubbery:** lower `bypass_dps` (for example 90). If
   tracking strafing targets feels jumpy, raise it.
5. Change one parameter at a time. Retest the same drill (still hold, slow strafe
   track, 90° turn) in the firing range.

**Button-press disturbance:** the filter cuts the camera kick from a 25 ms jolt by
40–60%. It doesn't delay the button itself: buttons are handled by separate code
that this change doesn't touch.

## Backup (before flashing)

- **Flashing a `.uf2` doesn't erase settings.** Profiles, calibration and config
  live in a separate flash area (NVM) that a `.uf2` doesn't overwrite. A backup is
  still recommended.
- **Simple backup:** in the Ctrl app, note or screenshot each custom profile and
  your settings (mouse sensitivity, touch sensitivity, deadzone, protocol).
- **Full flash backup (exact):**
  1. Put the controller in BOOTSEL mode (Ctrl app → Tools → Commands → Restart in
     bootsel mode, or hold **Home + Select** while plugging in USB).
  2. Run `picotool save -a alpakka_backup.bin`. This saves firmware, profiles,
     config and calibration. (`picotool` is from Raspberry Pi; builds exist for
     Windows, macOS and Linux.)
  3. To restore: `picotool load alpakka_backup.bin -o 0x10000000`, then
     `picotool reboot`.

## Installing (only when you decide to)

1. Back up as described above.
2. Enter BOOTSEL mode. The `RPI-RP2` drive appears.
3. Copy `alpakka_v0_1.2.0_gyrostab_default-off.uf2` to the drive. The controller
   reboots.
4. Check that gyro aiming behaves exactly as before (stabiliser OFF).
5. Then enable BALANCED from the home menu.

## Rollback to official firmware

1. Enter BOOTSEL mode (Home + Select while plugging in, or the Ctrl app command).
   If the firmware won't boot: on v0, remove the rear cover and hold the white
   BOOTSEL button on the Pico while plugging in USB.
2. Copy the official `alpakka_pico_1_2_0.uf2` to `RPI-RP2`. Get it from
   https://github.com/inputlabs/alpakka_firmware/releases/tag/1.2.0
   (sha256 `9087ea5dad1abb10ea7b2013949aca23b5e1d5acff9177d719a0c8a59b6855d8`).
3. Profiles and calibration carry over. The extra config byte is ignored by stock
   firmware.

**Restoring profiles and config:**
- Use the Ctrl app to re-enter any changed profile, or restore the full
  `picotool` backup.
- If the LEDs show the calibration warning (horizontal and vertical blink), run
  calibration as described in the manual.
- Last resort: Home menu factory reset, then re-create profiles.

## Troubleshooting

| Symptom | Fix |
|---|---|
| Aim feels delayed | Use OFF or lower the strength. Delay is the cost of smoothing (see the tables above). |
| Camera keeps moving slightly just after you stop a fast turn | That's the held-back rotation catching up (about 0.01–0.3° in tests). Lower the strength or the bypass speed. |
| Short rotation missing when releasing gyro touch mid-motion | This is by design: held-back rotation is discarded so the camera never moves after release. |
| Home + D-Hat push long-press does nothing | Your home profile already has a secondary action on D-Hat push. Reset the home profile, or ask for the shortcut to be rebound. |
| Behaves like stock in every mode | Check the LEDs in the stabiliser menu. With the `default-off` build, the mode is OFF until you change it. |

## Known risks and limitations

- **Never run on hardware.** These are untested on a physical controller: timing,
  the LED menu, the home-menu patch, and the D-Hat push behaving as a long-press.
- **Your firmware version:** the screenshot showed the Ctrl app version (1.2.0),
  which is not necessarily the controller's firmware version. Check it before
  flashing.
- **Not byte-identical to official builds:** the unmodified build from this
  sandbox (aarch64 host) differs in about 3 KB from the official 1.2.0 binary,
  most likely because of the build machine. Use the official `.uf2` for rollback.
- **Simulated tremor only:** the tremor and jolt models are synthetic. Real
  improvement depends on your hand's tremor frequency.
- **Unit assumption:** gyro units assume 17.5 m°/s per LSB (LSM6DS at 500 dps).
  This only affects how the °/s thresholds are interpreted, not sensitivity.
- **Absolute (steering) gyro mode is not filtered.**
- **CPU overhead is an estimate,** not a measurement.
- **New action ID:** `PROC_TUNE_GYRO_STAB` uses ID 246. A future upstream release
  could assign that ID to something else.
