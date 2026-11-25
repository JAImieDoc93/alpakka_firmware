// Host-side test harness for src/gyro_filter.c (same source as firmware).
// Build: make -C tests   Run: tests/gyro_filter_test
// All signals are angular velocity in deg/s sampled at the Alpakka v0 tick (250 Hz).
// "Angle" = integrated velocity = camera rotation (gain-independent of sens/RWS).

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "gyro_filter.h"

#define FS 250.0
#define DT (1.0 / FS)
#define MAXN 20000

typedef struct { const char *name; GyroFilterParams p; bool raw; } Cfg;

static Cfg cfgs[] = {
    {"RAW (off)",            {0, 0, 0, 0, 0, 0}, true},
    {"BALANCED",             {5.0f, 0.08f, 1.0f, 120.0f, 0, 0}, false},
    {"STRONG",               {0, 0, 0, 0, 0, 0}, false},  // Filled from gyro_filter_params() in main.
    {"BALANCED no-bypass",   {5.0f, 0.08f, 1.0f, 0.0f, 0, 0}, false},
    {"STRONG v1 (old)",      {3.0f, 0.05f, 1.0f, 180.0f, 0, 0}, false},
    {"LOW_LATENCY",          {8.0f, 0.10f, 2.0f, 120.0f, 0, 0}, false},
};
#define NCFG (int)(sizeof(cfgs)/sizeof(cfgs[0]))

// Deterministic RNG (xorshift) + Gaussian.
static unsigned long long rng = 88172645463325252ULL;
static double urand(void) { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return (rng >> 11) * (1.0 / 9007199254740992.0); }
static double grand(void) { double u = urand() + 1e-12, v = urand(); return sqrt(-2 * log(u)) * cos(2 * M_PI * v); }

// Hand tremor model: physiological tremor band 4-12 Hz with random phases,
// plus sensor noise. Amplitude in deg/s RMS.
static void tremor(double *v, int n, double rms, unsigned seed) {
    rng = 88172645463325252ULL ^ (seed * 2654435761ULL);
    double f[] = {4.5, 6.0, 8.0, 9.5, 11.0}, a[] = {0.5, 0.8, 1.0, 0.8, 0.5}, ph[5], s2 = 0;
    for (int k = 0; k < 5; k++) { ph[k] = urand() * 2 * M_PI; s2 += a[k] * a[k] / 2; }
    double norm = rms * 0.9 / sqrt(s2);
    for (int i = 0; i < n; i++) {
        double t = i * DT, x = 0;
        for (int k = 0; k < 5; k++) x += a[k] * sin(2 * M_PI * f[k] * t + ph[k]);
        v[i] += x * norm + grand() * rms * 0.3;  // ~0.3 rms broadband noise
    }
}

// Minimum-jerk velocity profile for a rotation of `deg` over `dur` seconds.
static void minjerk(double *v, int n, double t0, double dur, double deg) {
    for (int i = 0; i < n; i++) {
        double s = (i * DT - t0) / dur;
        if (s < 0 || s > 1) continue;
        v[i] += deg / dur * 30 * s * s * (1 - s) * (1 - s);
    }
}

static void run(const Cfg *c, const double *in, double *out, int n) {
    GyroFilterAxis ax; gyro_filter_reset(&ax);
    for (int i = 0; i < n; i++) out[i] = c->raw ? in[i] : gyro_filter_step(&ax, &c->p, (float)in[i], (float)DT);
}

static void integrate(const double *v, double *a, int n) { double s = 0; for (int i = 0; i < n; i++) { s += v[i] * DT; a[i] = s; } }
static double rms_diff(const double *a, const double *b, int i0, int i1) { double s = 0; for (int i = i0; i < i1; i++) s += (a[i] - b[i]) * (a[i] - b[i]); return sqrt(s / (i1 - i0)); }

// Delay (ms) of y relative to x by cross-correlation with parabolic interpolation.
static double xcorr_delay(const double *x, const double *y, int n) {
    double best = -1e300; int bk = 0; double c[60];
    for (int k = 0; k < 60; k++) {
        double s = 0; for (int i = 60; i < n; i++) s += x[i - k] * y[i];
        c[k] = s; if (s > best) { best = s; bk = k; }
    }
    double frac = 0;
    if (bk > 0 && bk < 59) { double d = c[bk - 1] - 2 * c[bk] + c[bk + 1]; if (d != 0) frac = 0.5 * (c[bk - 1] - c[bk + 1]) / d; }
    return (bk + frac) * DT * 1000;
}

static double in[MAXN], clean[MAXN], out[MAXN], ai[MAXN], ao[MAXN], ac[MAXN], o2[MAXN], cm[MAXN];

// Still-hand angle jitter RMS (1 s moving mean removed) for tremor of `amp` deg/s RMS.
static double jitter(const Cfg *c, double amp) {
    int n = 2500; memset(in, 0, sizeof(in)); tremor(in, n, amp, 1); run(c, in, out, n); integrate(out, ao, n);
    double s = 0; int cnt = 0;
    for (int i = 250; i < n - 125; i++) { double m = 0; for (int k = -125; k < 125; k++) m += ao[i + k]; m /= 250; s += (ao[i] - m) * (ao[i] - m); cnt++; }
    return sqrt(s / cnt);
}

int main(void) {
    int n;
    cfgs[2].p = *gyro_filter_params(GYRO_STAB_STRONG);
    // Guard: the table above must match the firmware for the classic modes.
    if (memcmp(&cfgs[1].p, gyro_filter_params(GYRO_STAB_BALANCED), sizeof(GyroFilterParams)) ||
        memcmp(&cfgs[5].p, gyro_filter_params(GYRO_STAB_LOW_LATENCY), sizeof(GyroFilterParams))) {
        printf("Test table out of sync with gyro_filter.c params[]\n"); return 1;
    }
    printf("# Gyro stabiliser host test results\n\nFs=%.0f Hz (v0 tick). Velocities in deg/s, angles in deg (camera rotation).\n", FS);

    // 1. Stationary jitter.
    printf("\n## 1. Stationary hand jitter (tremor 4-12 Hz, 2 deg/s RMS, 10 s)\n\n| Mode | vel RMS | angle jitter RMS (deg) | angle peak-peak (deg) | jitter reduction |\n|---|---|---|---|---|\n");
    n = 2500; double base = 0;
    for (int c = 0; c < NCFG; c++) {
        memset(in, 0, sizeof(in)); tremor(in, n, 2.0, 1); run(&cfgs[c], in, out, n);
        // Angle jitter: integrate, remove 1 s moving mean (slow drift is not "shake").
        integrate(out, ao, n);
        double s = 0, v = 0, mn = 1e9, mx = -1e9; int cnt = 0;
        for (int i = 250; i < n - 125; i++) {
            double m = 0; for (int k = -125; k < 125; k++) m += ao[i + k]; m /= 250;
            double d = ao[i] - m; s += d * d; cnt++; if (d < mn) mn = d; if (d > mx) mx = d;
        }
        for (int i = 250; i < n; i++) v += out[i] * out[i];
        double r = sqrt(s / cnt); if (c == 0) base = r;
        printf("| %s | %.3f | %.4f | %.4f | %.0f%% |\n", cfgs[c].name, sqrt(v / (n - 250)), r, mx - mn, 100 * (1 - r / base));
    }

    // 2. Micro-adjustment: 1.5 deg over 150 ms with tremor.
    printf("\n## 2. Micro-adjustment (1.5 deg in 150 ms, + 1 deg/s tremor)\n\n| Mode | delay (ms) | angle err RMS during move (deg) | final angle err (deg) | 90%% reached (ms after start) |\n|---|---|---|---|---|\n");
    n = 500;
    for (int c = 0; c < NCFG; c++) {
        memset(clean, 0, sizeof(clean)); minjerk(clean, n, 0.4, 0.15, 1.5);
        memcpy(in, clean, sizeof(double) * n); tremor(in, n, 1.0, 2); run(&cfgs[c], in, out, n);
        integrate(clean, ac, n); integrate(out, ao, n);
        int t90 = -1; for (int i = 100; i < n; i++) if (ao[i] - ao[99] >= 0.9 * 1.5) { t90 = i - 100; break; }
        printf("| %s | %.1f | %.4f | %.4f | %.0f |\n", cfgs[c].name, xcorr_delay(clean, out, n), rms_diff(ac, ao, 100, 160), ao[n - 1] - ac[n - 1], t90 * DT * 1000);
    }

    // 3. Slow tracking: 0.3 Hz sinusoid 15 deg/s peak + 5 deg/s pan, + tremor.
    printf("\n## 3. Slow target tracking (5 deg/s pan + 15 deg/s 0.3 Hz weave, + 2 deg/s tremor, 10 s)\n\n| Mode | delay vs intent (ms) | angle tracking err RMS (deg) | angle err RMS vs RAW incl. tremor (deg) |\n|---|---|---|---|\n");
    n = 2500;
    for (int c = 0; c < NCFG; c++) {
        for (int i = 0; i < n; i++) clean[i] = 5 + 15 * sin(2 * M_PI * 0.3 * i * DT);
        memcpy(in, clean, sizeof(double) * n); tremor(in, n, 2.0, 3); run(&cfgs[c], in, out, n);
        integrate(clean, ac, n); integrate(out, ao, n); integrate(in, ai, n);
        printf("| %s | %.1f | %.4f | %.4f |\n", cfgs[c].name, xcorr_delay(clean, out, n), rms_diff(ac, ao, 250, n), rms_diff(ai, ao, 250, n));
    }

    // 4. Fast rotations.
    printf("\n## 4. Fast rotations (minimum-jerk, + 2 deg/s tremor)\n\n| Mode | case | peak deg/s | angle lag at move end (deg) | max angle lag (deg) | final angle err after 0.5 s (deg) |\n|---|---|---|---|---|---|\n");
    double rot[][2] = {{90, 0.20}, {180, 0.30}, {90, 0.40}};
    for (int r = 0; r < 3; r++) for (int c = 0; c < NCFG; c++) {
        n = 500; memset(clean, 0, sizeof(clean)); minjerk(clean, n, 0.5, rot[r][1], rot[r][0]);
        memcpy(in, clean, sizeof(double) * n); tremor(in, n, 2.0, 4); run(&cfgs[c], in, out, n);
        integrate(in, ai, n); integrate(out, ao, n);
        int iend = (int)((0.5 + rot[r][1]) * FS), ifin = iend + 125; double ml = 0;
        for (int i = 0; i < ifin; i++) if (fabs(ai[i] - ao[i]) > ml) ml = fabs(ai[i] - ao[i]);
        printf("| %s | %.0f deg/%.0f ms | %.0f | %.3f | %.3f | %.4f |\n", cfgs[c].name, rot[r][0], rot[r][1] * 1000, rot[r][0] / rot[r][1] * 1.875,
               ai[iend] - ao[iend], ml, ai[ifin] - ao[ifin]);
    }

    // 5. Button press disturbance: 25 ms half-sine 40 deg/s jolt followed by rebound.
    printf("\n## 5. Button-press disturbance (40 deg/s 25 ms jolt + 50%% rebound, while still)\n\n| Mode | peak angle excursion (deg) | reduction |\n|---|---|---|\n");
    n = 500; double bp = 0;
    for (int c = 0; c < NCFG; c++) {
        memset(in, 0, sizeof(in));
        for (int i = 0; i < n; i++) { double t = i * DT - 1.0;
            if (t >= 0 && t < 0.025) in[i] = 40 * sin(M_PI * t / 0.025);
            else if (t >= 0.025 && t < 0.075) in[i] = -20 * sin(M_PI * (t - 0.025) / 0.05); }
        run(&cfgs[c], in, out, n); integrate(out, ao, n);
        double m = 0; for (int i = 0; i < n; i++) if (fabs(ao[i]) > m) m = fabs(ao[i]);
        if (c == 0) bp = m;
        printf("| %s | %.4f | %.0f%% |\n", cfgs[c].name, m, 100 * (1 - m / bp));
    }

    // 6. Engage/disengage: 30 deg/s pan, gyro disengaged mid-motion at 1.0 s.
    printf("\n## 6. Engage / disengage (pan at 30 deg/s; release at 1.0 s; firmware drops filter state)\n\n| Mode | first output after engage (deg/s) | rotation lost at release (deg) | output after release |\n|---|---|---|---|\n");
    n = 250;
    for (int c = 0; c < NCFG; c++) {
        for (int i = 0; i < n; i++) in[i] = 30;
        run(&cfgs[c], in, out, n);  // engaged 0..1 s, then state is reset, nothing more emitted
        double si = 0, so = 0; for (int i = 0; i < n; i++) { si += in[i] * DT; so += out[i] * DT; }
        printf("| %s | %.2f | %.4f | 0 (by construction) |\n", cfgs[c].name, out[0], si - so);
    }
    // Worst realistic case: release right after a stop (deceleration).
    printf("\nRelease 20 ms after a 90 deg / 200 ms flick ends (residual in filter dropped):\n\n| Mode | rotation lost (deg) |\n|---|---|\n");
    for (int c = 0; c < NCFG; c++) {
        n = 300; memset(in, 0, sizeof(in)); minjerk(in, n, 0.1, 0.2, 90); n = (int)(0.32 * FS);
        run(&cfgs[c], in, out, n);
        double si = 0, so = 0; for (int i = 0; i < n; i++) { si += in[i] * DT; so += out[i] * DT; }
        printf("| %s | %.4f |\n", cfgs[c].name, si - so);
    }

    // 7. Total integrated rotation (no release): long random walk.
    printf("\n## 7. Total integrated rotation error (60 s random aiming + flicks, filter allowed to settle)\n\n| Mode | total in (deg) | error (deg) | error %% |\n|---|---|---|---|\n");
    n = 15000 + 250;
    for (int c = 0; c < NCFG; c++) {
        memset(in, 0, sizeof(in)); rng = 12345;
        for (int k = 0; k < 40; k++) { double t0 = urand() * 58, d = 0.1 + urand() * 0.4, deg = (urand() - 0.4) * 120; minjerk(in, 15000, t0, d, deg); }
        tremor(in, 15000, 2.0, 7);
        run(&cfgs[c], in, out, n);
        double si = 0, so = 0; for (int i = 0; i < n; i++) { si += in[i] * DT; so += out[i] * DT; }
        printf("| %s | %.2f | %.4f | %.4f%% |\n", cfgs[c].name, si, so - si, 100 * (so - si) / si);
    }

    // 8. Frequency response at small amplitude (stationary) and phase delay.
    printf("\n## 8. Small-signal frequency response (2 deg/s sine, no motion)\n\n| Mode | 1 Hz gain / delay | 3 Hz | 6 Hz | 8 Hz | 10 Hz | 12 Hz |\n|---|---|---|---|---|---|---|\n");
    double fr[] = {1, 3, 6, 8, 10, 12};
    for (int c = 0; c < NCFG; c++) {
        printf("| %s |", cfgs[c].name);
        for (int k = 0; k < 6; k++) {
            n = 5000; for (int i = 0; i < n; i++) in[i] = 2 * sin(2 * M_PI * fr[k] * i * DT);
            run(&cfgs[c], in, out, n);
            double re = 0, im = 0, ri = 0, ii = 0;
            for (int i = 2500; i < n; i++) { double w = 2 * M_PI * fr[k] * i * DT; re += out[i] * cos(w); im += out[i] * sin(w); ri += in[i] * cos(w); ii += in[i] * sin(w); }
            double g = sqrt(re * re + im * im) / sqrt(ri * ri + ii * ii), ph = atan2(re, im) - atan2(ri, ii);
            while (ph > 0) ph -= 2 * M_PI;
            printf(" %.2f / %.1f ms |", g, -ph / (2 * M_PI * fr[k]) * 1000);
        }
        printf("\n");
    }

    // 9. Step response.
    printf("\n## 9. Velocity step response (63%% / 90%% rise, ms)\n\n| Mode | 0->20 deg/s | 0->60 deg/s | 0->200 deg/s |\n|---|---|---|---|\n");
    double st[] = {20, 60, 200};
    for (int c = 0; c < NCFG; c++) {
        printf("| %s |", cfgs[c].name);
        for (int k = 0; k < 3; k++) {
            n = 500; for (int i = 0; i < n; i++) in[i] = i >= 100 ? st[k] : 0;
            run(&cfgs[c], in, out, n);
            int a = -1, b = -1; for (int i = 100; i < n; i++) { if (a < 0 && out[i] >= 0.63 * st[k]) a = i - 100; if (b < 0 && out[i] >= 0.9 * st[k]) { b = i - 100; break; } }
            printf(" %.0f / %.0f |", (a + 1) * DT * 1000, (b + 1) * DT * 1000);
        }
        printf("\n");
    }

    // 10. Bypass continuity: slow ramp through the bypass band and back.
    printf("\n## 10. Bypass transition continuity (ramp 0->300->0 deg/s over 3 s, + tremor)\n\n| Mode | max |out step| (deg/s per tick) | max |raw step| | max |out-raw| (deg/s) |\n|---|---|---|---|\n");
    n = 750;
    for (int c = 0; c < NCFG; c++) {
        for (int i = 0; i < n; i++) in[i] = 300 * (i < n / 2 ? (double)i / (n / 2) : (double)(n - i) / (n / 2));
        tremor(in, n, 2.0, 9); run(&cfgs[c], in, out, n);
        double mo = 0, mr = 0, md = 0;
        for (int i = 1; i < n; i++) { if (fabs(out[i] - out[i - 1]) > mo) mo = fabs(out[i] - out[i - 1]); if (fabs(in[i] - in[i - 1]) > mr) mr = fabs(in[i] - in[i - 1]); if (fabs(out[i] - in[i]) > md) md = fabs(out[i] - in[i]); }
        printf("| %s | %.2f | %.2f | %.2f |\n", cfgs[c].name, mo, mr, md);
    }

    // 11. Jitter vs tremor amplitude (shaky hands).
    printf("\n## 11. Still-hand jitter reduction vs tremor amplitude\n\n| Mode | 1 deg/s | 2 deg/s | 4 deg/s | 8 deg/s |\n|---|---|---|---|---|\n");
    double amps[] = {1, 2, 4, 8}, j0[4];
    for (int k = 0; k < 4; k++) j0[k] = jitter(&cfgs[0], amps[k]);
    for (int c = 1; c < NCFG; c++) {
        printf("| %s |", cfgs[c].name);
        for (int k = 0; k < 4; k++) printf(" %.0f%% |", 100 * (1 - jitter(&cfgs[c], amps[k]) / j0[k]));
        printf("\n");
    }

    // 12. Delay vs correction size (minimum-jerk, + 1 deg/s tremor, mean of 5 seeds).
    double mv[][2] = {{0.3, 0.10}, {0.5, 0.20}, {1, 0.40}, {1, 0.15}, {1.5, 0.15}, {3, 0.20}, {5, 0.25}};
    printf("\n## 12. Delay (ms) vs correction size (deg in ms, + 1 deg/s tremor, 5 seeds)\n\n| Mode |");
    for (int m = 0; m < 7; m++) printf(" %.1f/%.0f |", mv[m][0], mv[m][1] * 1000);
    printf("\n|---|---|---|---|---|---|---|---|\n");
    for (int c = 1; c < NCFG; c++) {
        printf("| %s |", cfgs[c].name);
        for (int m = 0; m < 7; m++) {
            double d = 0; n = 500;
            for (int sd = 0; sd < 5; sd++) {
                memset(clean, 0, sizeof(clean)); minjerk(clean, n, 0.4, mv[m][1], mv[m][0]);
                memcpy(in, clean, sizeof(double) * n); tremor(in, n, 1.0, 2 + sd); run(&cfgs[c], in, out, n);
                d += xcorr_delay(clean, out, n) / 5;
            }
            printf(" %.1f |", d);
        }
        printf("\n");
    }

    // 13. Correction right after a flick: response to a 1.5 deg / 150 ms correction
    // made `gap` after a 90 deg / 200 ms flick (run with minus run without it).
    printf("\n## 13. Delay (ms) of a 1.5 deg correction made shortly after a 90 deg flick (5 seeds)\n\n| Mode | +50 ms | +100 ms | +200 ms | +400 ms |\n|---|---|---|---|---|\n");
    for (int c = 1; c < NCFG; c++) {
        printf("| %s |", cfgs[c].name);
        for (double gap = 0.05; gap <= 0.41; gap *= 2) {
            double d = 0; n = 1000;
            for (int sd = 0; sd < 5; sd++) {
                memset(in, 0, sizeof(in)); minjerk(in, n, 0.3, 0.2, 90); tremor(in, n, 1.0, 2 + sd); run(&cfgs[c], in, o2, n);
                memset(cm, 0, sizeof(cm)); minjerk(cm, n, 0.5 + gap, 0.15, 1.5);
                for (int i = 0; i < n; i++) in[i] += cm[i];
                run(&cfgs[c], in, out, n);
                for (int i = 0; i < n; i++) out[i] -= o2[i];
                int s0 = (int)((0.5 + gap) * FS) - 60;
                d += xcorr_delay(cm + s0, out + s0, 250) / 5;
            }
            printf(" %.1f |", d);
        }
        printf("\n");
    }

    // 14. Slow steady pan under tremor: mean angle lag, as ms of the pan speed.
    printf("\n## 14. Slow steady pan under tremor: lag behind intent (ms-equivalent = angle lag / pan speed)\n\n| Mode | 2 deg/s tremor, 1 / 2 / 4 / 8 deg/s pan | 4 deg/s tremor | 8 deg/s tremor |\n|---|---|---|---|\n");
    double pans[] = {1, 2, 4, 8}, trs[] = {2, 4, 8};
    for (int c = 1; c < NCFG; c++) {
        printf("| %s |", cfgs[c].name);
        for (int t = 0; t < 3; t++) {
            for (int q = 0; q < 4; q++) {
                n = 2500; for (int i = 0; i < n; i++) in[i] = pans[q];
                tremor(in, n, trs[t], 5); run(&cfgs[c], in, out, n);
                double si = 0, so = 0, l = 0;
                for (int i = 0; i < n; i++) { si += pans[q] * DT; so += out[i] * DT; if (i > 500) l += si - so; }
                printf("%s%.0f", q ? " / " : " ", l / (n - 501) / pans[q] * 1000);
            }
            printf(" |");
        }
        printf("\n");
    }

    // 15. Robustness.
    printf("\n## 15. Numerical robustness\n\n");
    int fails = 0;
    for (int c = 1; c < NCFG; c++) {
        GyroFilterAxis ax; gyro_filter_reset(&ax); rng = 99;
        for (int i = 0; i < 200000; i++) {
            float x = (float)((urand() - 0.5) * 4000), dt = (float)(urand() * 0.02);
            if (i % 997 == 0) x = NAN;
            if (i % 1009 == 0) dt = 0;
            if (i % 1013 == 0) dt = -1;
            if (i % 1019 == 0) x = INFINITY;
            float y = gyro_filter_step(&ax, &cfgs[c].p, x, dt);
            if (!isfinite(y) || fabsf(y) > 1e5f) { fails++; break; }
        }
        // Constant input must converge exactly-ish.
        gyro_filter_reset(&ax); float y = 0;
        for (int i = 0; i < 2000; i++) y = gyro_filter_step(&ax, &cfgs[c].p, i < 10 ? 0 : 7.5f, 0.004f);
        if (fabsf(y - 7.5f) > 1e-4f) fails++;
    }
    printf("Random/NaN/Inf/bad-dt fuzz (200k samples/mode), bounded output, DC convergence: %s\n", fails ? "FAIL" : "PASS");
    return fails ? 1 : 0;
}
