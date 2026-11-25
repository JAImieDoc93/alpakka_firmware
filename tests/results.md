# Gyro stabiliser host test results

Fs=250 Hz (v0 tick). Velocities in deg/s, angles in deg (camera rotation).

## 1. Stationary hand jitter (tremor 4-12 Hz, 2 deg/s RMS, 10 s)

| Mode | vel RMS | angle jitter RMS (deg) | angle peak-peak (deg) | jitter reduction |
|---|---|---|---|---|
| RAW (off) | 1.889 | 0.0409 | 0.2030 | 0% |
| BALANCED | 0.987 | 0.0252 | 0.1332 | 38% |
| STRONG | 0.479 | 0.0142 | 0.0776 | 65% |
| BALANCED no-bypass | 0.987 | 0.0252 | 0.1332 | 38% |
| STRONG v1 (old) | 0.687 | 0.0188 | 0.1016 | 54% |
| LOW_LATENCY | 1.265 | 0.0309 | 0.1577 | 25% |

## 2. Micro-adjustment (1.5 deg in 150 ms, + 1 deg/s tremor)

| Mode | delay (ms) | angle err RMS during move (deg) | final angle err (deg) | 90% reached (ms after start) |
|---|---|---|---|---|
| RAW (off) | 0.0 | 0.0236 | 0.0002 | 108 |
| BALANCED | 21.8 | 0.2478 | -0.0198 | 148 |
| STRONG | 0.0 | 0.0405 | -0.0209 | 108 |
| BALANCED no-bypass | 21.8 | 0.2478 | -0.0198 | 148 |
| STRONG v1 (old) | 30.5 | 0.3654 | -0.0209 | 192 |
| LOW_LATENCY | 14.9 | 0.1652 | -0.0171 | 132 |

## 3. Slow target tracking (5 deg/s pan + 15 deg/s 0.3 Hz weave, + 2 deg/s tremor, 10 s)

| Mode | delay vs intent (ms) | angle tracking err RMS (deg) | angle err RMS vs RAW incl. tremor (deg) |
|---|---|---|---|
| RAW (off) | 12.7 | 0.0788 | 0.0000 |
| BALANCED | 38.0 | 0.3032 | 0.2901 |
| STRONG | 13.5 | 0.0878 | 0.0430 |
| BALANCED no-bypass | 38.0 | 0.3032 | 0.2901 |
| STRONG v1 (old) | 54.3 | 0.4884 | 0.4787 |
| LOW_LATENCY | 29.1 | 0.2070 | 0.1884 |

## 4. Fast rotations (minimum-jerk, + 2 deg/s tremor)

| Mode | case | peak deg/s | angle lag at move end (deg) | max angle lag (deg) | final angle err after 0.5 s (deg) |
|---|---|---|---|---|---|
| RAW (off) | 90 deg/200 ms | 844 | 0.000 | 0.000 | 0.0000 |
| BALANCED | 90 deg/200 ms | 844 | 0.091 | 0.603 | 0.0092 |
| STRONG | 90 deg/200 ms | 844 | 0.000 | 0.573 | 0.0170 |
| BALANCED no-bypass | 90 deg/200 ms | 844 | 0.103 | 7.044 | 0.0092 |
| STRONG v1 (old) | 90 deg/200 ms | 844 | 0.268 | 1.214 | 0.0146 |
| LOW_LATENCY | 90 deg/200 ms | 844 | 0.038 | 0.531 | 0.0062 |
| RAW (off) | 180 deg/300 ms | 1125 | 0.000 | 0.000 | 0.0000 |
| BALANCED | 180 deg/300 ms | 1125 | 0.019 | 0.743 | -0.0395 |
| STRONG | 180 deg/300 ms | 1125 | -0.000 | 0.633 | -0.0419 |
| BALANCED no-bypass | 180 deg/300 ms | 1125 | 0.017 | 6.895 | -0.0395 |
| STRONG v1 (old) | 180 deg/300 ms | 1125 | 0.076 | 1.501 | -0.0449 |
| LOW_LATENCY | 180 deg/300 ms | 1125 | 0.008 | 0.639 | -0.0361 |
| RAW (off) | 90 deg/400 ms | 422 | 0.000 | 0.000 | 0.0000 |
| BALANCED | 90 deg/400 ms | 422 | 0.023 | 1.351 | 0.0282 |
| STRONG | 90 deg/400 ms | 422 | -0.001 | 0.369 | 0.0564 |
| BALANCED no-bypass | 90 deg/400 ms | 422 | 0.023 | 4.093 | 0.0282 |
| STRONG v1 (old) | 90 deg/400 ms | 422 | 0.118 | 2.837 | 0.0436 |
| LOW_LATENCY | 90 deg/400 ms | 422 | 0.009 | 1.023 | 0.0177 |

## 5. Button-press disturbance (40 deg/s 25 ms jolt + 50% rebound, while still)

| Mode | peak angle excursion (deg) | reduction |
|---|---|---|
| RAW (off) | 0.6307 | 0% |
| BALANCED | 0.3586 | 43% |
| STRONG | 0.2022 | 68% |
| BALANCED no-bypass | 0.3586 | 43% |
| STRONG v1 (old) | 0.2731 | 57% |
| LOW_LATENCY | 0.4387 | 30% |

## 6. Engage / disengage (pan at 30 deg/s; release at 1.0 s; firmware drops filter state)

| Mode | first output after engage (deg/s) | rotation lost at release (deg) | output after release |
|---|---|---|---|
| RAW (off) | 30.00 | 0.0000 | 0 (by construction) |
| BALANCED | 30.00 | 0.6452 | 0 (by construction) |
| STRONG | 30.00 | 0.0032 | 0 (by construction) |
| BALANCED no-bypass | 30.00 | 0.6452 | 0 (by construction) |
| STRONG v1 (old) | 30.00 | 1.0610 | 0 (by construction) |
| LOW_LATENCY | 30.00 | 0.4341 | 0 (by construction) |

Release 20 ms after a 90 deg / 200 ms flick ends (residual in filter dropped):

| Mode | rotation lost (deg) |
|---|---|
| RAW (off) | 0.0000 |
| BALANCED | 0.0106 |
| STRONG | 0.0000 |
| BALANCED no-bypass | 0.0121 |
| STRONG v1 (old) | 0.0625 |
| LOW_LATENCY | 0.0024 |

## 7. Total integrated rotation error (60 s random aiming + flicks, filter allowed to settle)

| Mode | total in (deg) | error (deg) | error % |
|---|---|---|---|
| RAW (off) | 792.20 | 0.0000 | 0.0000% |
| BALANCED | 792.20 | -0.0000 | -0.0000% |
| STRONG | 792.20 | -0.0000 | -0.0000% |
| BALANCED no-bypass | 792.20 | 0.0000 | 0.0000% |
| STRONG v1 (old) | 792.20 | -0.0000 | -0.0000% |
| LOW_LATENCY | 792.20 | -0.0000 | -0.0000% |

## 8. Small-signal frequency response (2 deg/s sine, no motion)

| Mode | 1 Hz gain / delay | 3 Hz | 6 Hz | 8 Hz | 10 Hz | 12 Hz |
|---|---|---|---|---|---|---|
| RAW (off) | 1.00 / -0.0 ms | 1.00 / -0.0 ms | 1.00 / -0.0 ms | 1.00 / -0.0 ms | 1.00 / -0.0 ms | 1.00 / -0.0 ms |
| BALANCED | 0.98 / 30.7 ms | 0.85 / 27.7 ms | 0.63 / 21.8 ms | 0.52 / 18.5 ms | 0.43 / 15.9 ms | 0.37 / 13.8 ms |
| STRONG | 1.00 / 3.0 ms | 1.02 / 10.0 ms | 0.31 / 31.3 ms | 0.24 / 24.5 ms | 0.19 / 19.9 ms | 0.16 / 16.7 ms |
| BALANCED no-bypass | 0.98 / 30.7 ms | 0.85 / 27.7 ms | 0.63 / 21.8 ms | 0.52 / 18.5 ms | 0.43 / 15.9 ms | 0.37 / 13.8 ms |
| STRONG v1 (old) | 0.95 / 49.9 ms | 0.70 / 40.1 ms | 0.44 / 27.6 ms | 0.35 / 22.2 ms | 0.28 / 18.5 ms | 0.24 / 15.7 ms |
| LOW_LATENCY | 0.99 / 19.4 ms | 0.93 / 18.5 ms | 0.78 / 16.1 ms | 0.68 / 14.5 ms | 0.60 / 12.9 ms | 0.53 / 11.6 ms |

## 9. Velocity step response (63% / 90% rise, ms)

| Mode | 0->20 deg/s | 0->60 deg/s | 0->200 deg/s |
|---|---|---|---|
| RAW (off) | 4 / 4 | 4 / 4 | 4 / 4 |
| BALANCED | 32 / 68 | 32 / 56 | 4 / 4 |
| STRONG | 28 / 28 | 20 / 20 | 4 / 4 |
| BALANCED no-bypass | 32 / 68 | 32 / 56 | 24 / 40 |
| STRONG v1 (old) | 52 / 104 | 44 / 80 | 4 / 4 |
| LOW_LATENCY | 24 / 44 | 20 / 40 | 4 / 4 |

## 10. Bypass transition continuity (ramp 0->300->0 deg/s over 3 s, + tremor)

| Mode | max |out step| (deg/s per tick) | max |raw step| | max |out-raw| (deg/s) |
|---|---|---|---|
| RAW (off) | 3.71 | 3.71 | 0.00 |
| BALANCED | 3.71 | 3.71 | 8.75 |
| STRONG | 7.53 | 3.71 | 7.30 |
| BALANCED no-bypass | 2.56 | 3.71 | 8.18 |
| STRONG v1 (old) | 3.71 | 3.71 | 12.03 |
| LOW_LATENCY | 3.71 | 3.71 | 5.38 |

## 11. Still-hand jitter reduction vs tremor amplitude

| Mode | 1 deg/s | 2 deg/s | 4 deg/s | 8 deg/s |
|---|---|---|---|---|
| BALANCED | 39% | 38% | 38% | 36% |
| STRONG | 65% | 65% | 65% | 65% |
| BALANCED no-bypass | 39% | 38% | 38% | 36% |
| STRONG v1 (old) | 54% | 54% | 53% | 52% |
| LOW_LATENCY | 25% | 25% | 24% | 23% |

## 12. Delay (ms) vs correction size (deg in ms, + 1 deg/s tremor, 5 seeds)

| Mode | 0.3/100 | 0.5/200 | 1.0/400 | 1.0/150 | 1.5/150 | 3.0/200 | 5.0/250 |
|---|---|---|---|---|---|---|---|
| BALANCED | 24.4 | 22.4 | 27.5 | 22.5 | 22.4 | 23.2 | 23.5 |
| STRONG | 10.0 | 3.3 | 1.3 | 1.5 | 0.7 | 0.0 | 0.0 |
| BALANCED no-bypass | 24.4 | 22.4 | 27.5 | 22.5 | 22.4 | 23.2 | 23.5 |
| STRONG v1 (old) | 31.0 | 33.8 | 43.7 | 31.2 | 31.1 | 33.8 | 35.0 |
| LOW_LATENCY | 18.7 | 13.9 | 17.1 | 15.6 | 15.3 | 15.0 | 14.9 |

## 13. Delay (ms) of a 1.5 deg correction made shortly after a 90 deg flick (5 seeds)

| Mode | +50 ms | +100 ms | +200 ms | +400 ms |
|---|---|---|---|---|
| BALANCED | 8.6 | 10.4 | 13.9 | 19.1 |
| STRONG | 0.0 | 0.0 | 0.0 | 0.0 |
| BALANCED no-bypass | 8.6 | 10.4 | 13.9 | 19.1 |
| STRONG v1 (old) | 13.0 | 15.4 | 20.2 | 27.1 |
| LOW_LATENCY | 8.7 | 10.9 | 13.9 | 15.4 |

## 14. Slow steady pan under tremor: lag behind intent (ms-equivalent = angle lag / pan speed)

| Mode | 2 deg/s tremor, 1 / 2 / 4 / 8 deg/s pan | 4 deg/s tremor | 8 deg/s tremor |
|---|---|---|---|
| BALANCED | 36 / 33 / 31 / 29 | 39 / 35 / 32 / 29 | 47 / 38 / 33 / 30 |
| STRONG | 71 / 40 / 9 / 1 | 82 / 70 / 35 / 7 | 97 / 79 / 69 / 32 |
| BALANCED no-bypass | 36 / 33 / 31 / 29 | 39 / 35 / 32 / 29 | 47 / 38 / 33 / 30 |
| STRONG v1 (old) | 56 / 54 / 51 / 47 | 60 / 55 / 52 / 48 | 67 / 57 / 52 / 48 |
| LOW_LATENCY | 24 / 22 / 20 / 19 | 28 / 23 / 21 / 19 | 36 / 27 / 23 / 20 |

## 15. Numerical robustness

Random/NaN/Inf/bad-dt fuzz (200k samples/mode), bounded output, DC convergence: PASS
