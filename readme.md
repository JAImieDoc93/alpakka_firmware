# Alpakka Firmware

> **Unofficial experimental fork: adaptive gyro stabiliser (Alpakka v0 only).**
> Adds a One Euro-style filter that smooths hand tremor on the gyro without adding noticeable delay, with four modes (OFF, LOW_LATENCY, BALANCED, STRONG). OFF behaves like stock.
> Not made or supported by Input Labs. Use at your own risk. Roll back with the official 1.2.0 `.uf2`.
> **Testers:** grab a `.uf2` from the [latest release](../../releases/latest), read [docs/gyro_stabiliser.md](docs/gyro_stabiliser.md), and tell us how it feels via [Issues](../../issues).
> Status: simulation results are in `tests/results.md`. On real hardware only STRONG has been tried, by one user, and it felt good. Everything else needs testers.

*Alpakka controller reference firmware (for Raspberry Pi Pico)*
## Project links
- [Alpakka Manual](https://inputlabs.io/devices/alpakka/manual).
- [Alpakka Firmware](https://github.com/inputlabs/alpakka_firmware). _(you are here)_
- [Alpakka PCB](https://github.com/inputlabs/alpakka_pcb).
- [Alpakka 3D-print](https://github.com/inputlabs/alpakka_case).
- [Input Labs Roadmap](https://github.com/orgs/inputlabs/projects/2/views/2).

## Supported developer operative systems
- GNU/Linux (and MacOS) - See [Development in Linux](https://inputlabs.io/devices/alpakka/manual/dev_unix).
- Windows - See [Development in Windows](https://inputlabs.io/devices/alpakka/manual/dev_windows).

## System dependencies
With `apt`, `rpm`, `pacman`, `brew`, or the equivalent package manager of your system, install:
- **gcc**
- **git**
- **cmake**

## Project dependencies
- `make install`: Download and configure dependencies automatically.

## Compilation targets
- `alpakka_v0`: Alpakka v0.x.x with Raspberry Pico.
- `alpakka_v1`: Alpakka v1.x.x with Marmota module.
- `dongle`: Wireless dongle.
- `llama`: ESP wireless module flasher (ESP-LLAMA repository).

Example usage:
```
DEVICE=alpakka_v1 make
```

## Development commands
- `DEVICE=<device> make`: Build compilation environment and build executables.
- `make rebuild`: Build executables again using cache (faster).
- `make load`: Load built .uf2 file into the Pico (requires bootsel mode or active session).
- `make reload`: Do both `rebuild` and `load` commands (for dev convenience).
- `make clean`: Delete previous build files.
- `make session`: Connect to UART serial stdio, and display controller log.

While having an active session:
- `make restart`: Restart the controller.
- `make bootsel`: Put the controller in bootsel mode.
- `make calibrate`: Calibrate thumbstick and IMUs.
- `make format`: Format NVM sector and reset to initial values.
- `make test`: Start a semi-manual testing procedure for the buttons and axis.

## Devkit button
- Single press: Restart the controller.
- Double press: Put the controller in bootsel mode.

Read the [Alpakka developer manual](https://inputlabs.io/devices/alpakka/manual/dev) for details about the **devkit** and more.
