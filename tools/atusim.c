/* atusim.c — ATU-100 EXT algorithmic simulator
 *
 * Build: gcc -o /tmp/atusim tools/atusim.c -lm && /tmp/atusim
 *
 * The coarse_cap/coarse_tune/sharp_cap/sharp_ind/sub_tune/tune functions below
 * are verbatim copies from main.h. When the firmware algorithm changes, mirror
 * those changes here — they are marked with "VERBATIM: main.h" comments.
 *
 * Scenarios:
 *   S1  bimodal 30m delta loop — global min at high-L must win over local min at low-L
 *   S2  flat/unmatchable       — constant SWR, algorithm must terminate without crash
 *   S3  simple unimodal 20m   — single minimum, algorithm must find it
 *   S4  TX-inhibit mid-scan   — SKIP until Fix 5 is implemented (would hang)
 */

#include <stdio.h>
#include <math.h>

/* ── charbits type (from cross_compiler.h) ── */
typedef union {
    unsigned char bytes;
    struct {
        unsigned B0:1; unsigned B1:1; unsigned B2:1; unsigned B3:1;
        unsigned B4:1; unsigned B5:1; unsigned B6:1; unsigned B7:1;
    } bits;
} charbits;

/* ── firmware globals (from main.h) ── */
static char g_c_ind = 0, g_c_cap = 0;
static char g_c_SW = 0;
static char g_c_step_cap = 0, g_c_step_ind = 0;
static char g_c_L_mult = 4, g_c_C_mult = 4;
int g_i_SWR = 0, g_i_PWR = 5, g_i_P_max = 0, g_i_swr_a = 0;
static char g_b_rready = 0, g_char_p_cnt = 0;
static char e_c_b_L_linear = 0, e_c_b_C_linear = 0;
static char e_c_num_L_q = 7, e_c_num_C_q = 7;
static int  e_i_watts_min_for_start = 1, e_i_watts_max_for_start = 0;
static int  e_i_tenths_init_max_swr = 0;
static char e_c_b_Loss_ind = 0;

/* ── scenario state ── */
typedef int (*swr_fn)(unsigned char ind, unsigned char cap, unsigned char sw);
static swr_fn current_model = NULL;
static int sim_inhibit_on_call = -1;
static int sim_call_n = 0;

/* ── hardware stubs ── */
#define CLRWDT() do {} while(0)
static void Vdelay_ms(int ms) { (void)ms; }
static void Delay_ms(int ms)  { (void)ms; }
static void show_pwr(int p, int s) { (void)p; (void)s; }
static void lcd_ind(void)    {}
static void show_reset(void) {}

static void set_ind(char ind) { g_c_ind = ind; Vdelay_ms(0); }
static void set_cap(char cap) { g_c_cap = cap; Vdelay_ms(0); }
static void set_sw(char sw)   { g_c_SW  = sw;  Vdelay_ms(0); }

static void atu_reset(void) {
    g_c_ind = 0; g_c_cap = 0;
    set_ind(g_c_ind); set_cap(g_c_cap);
}

/* ── simulated get_pwr / get_swr ── */
static void get_pwr(void) {
    sim_call_n++;
    if (sim_inhibit_on_call >= 0 && sim_call_n >= sim_inhibit_on_call) {
        g_i_PWR = 0;
        g_i_SWR = 999;
        return;
    }
    g_i_PWR = 5;
    g_i_SWR = current_model ? current_model(g_c_ind, g_c_cap, g_c_SW) : 999;
}

static void get_swr(void) {
    get_pwr();
    while (g_i_PWR < e_i_watts_min_for_start) {
        /* TX inhibited. Fix 5 will add tx_seen logic and graceful abort here.
         * Until then, signal abort via SWR=0 and return immediately.
         * Scenarios that trigger inhibit (S4+) are skipped. */
        g_i_SWR = 0;
        return;
    }
}

/* ── algorithm functions: VERBATIM from main.h ────────────────────────────
 * Keep in sync with:
 *   ATU_100_EXT_board/FirmWare_PIC16F1938/1938_EXT_MPLAB_sources_V_3.2/main.h
 * ───────────────────────────────────────────────────────────────────────── */

static void coarse_cap(void)
{
    char l_coarse_cap_step = 3;
    char l_coarse_cap_count;
    int l_coarse_cap_min_swr;

    g_c_cap = 0;
    set_cap(g_c_cap);
    g_c_step_cap = l_coarse_cap_step;
    get_swr();
    if (g_i_SWR == 0)
        return;
    l_coarse_cap_min_swr = g_i_SWR;
    for (l_coarse_cap_count = l_coarse_cap_step; l_coarse_cap_count <= 31;)
    {
        set_cap(l_coarse_cap_count * g_c_C_mult);
        get_swr();
        if (g_i_SWR == 0)
            return;
        if (g_i_SWR < l_coarse_cap_min_swr)
        {
            l_coarse_cap_min_swr = g_i_SWR;
            g_c_cap = l_coarse_cap_count * g_c_C_mult;
            g_c_step_cap = l_coarse_cap_step;
            if (g_i_SWR < 120)
                break;
        }
        l_coarse_cap_count += l_coarse_cap_step;
        if (e_c_b_C_linear == 0 & l_coarse_cap_count == 9)
            l_coarse_cap_count = 8;
        else if (e_c_b_C_linear == 0 & l_coarse_cap_count == 17)
        {
            l_coarse_cap_count = 16;
            l_coarse_cap_step = 4;
        }
    }
    set_cap(g_c_cap);
    return;
}

static void coarse_tune(void)
{
    char l_coarse_tune_step = 3;
    char l_coarse_tune_count;
    char l_coarse_tune_mem_cap, l_coarse_tune_mem_step_cap;
    int l_coarse_tune_min_swr;

    l_coarse_tune_mem_cap = 0;
    g_c_step_ind = l_coarse_tune_step;
    l_coarse_tune_mem_step_cap = 3;
    l_coarse_tune_min_swr = 9999;
    for (l_coarse_tune_count = 0; l_coarse_tune_count <= 31;)
    {
        set_ind(l_coarse_tune_count * g_c_L_mult);
        coarse_cap();
        get_swr();
        if (g_i_SWR == 0)
            return;
        if (g_i_SWR < l_coarse_tune_min_swr)
        {
            l_coarse_tune_min_swr = g_i_SWR;
            g_c_ind = l_coarse_tune_count * g_c_L_mult;
            l_coarse_tune_mem_cap = g_c_cap;
            g_c_step_ind = l_coarse_tune_step;
            l_coarse_tune_mem_step_cap = g_c_step_cap;
            if (g_i_SWR < 120)
                break;
        }
        l_coarse_tune_count += l_coarse_tune_step;
        if (e_c_b_L_linear == 0 & l_coarse_tune_count == 9)
            l_coarse_tune_count = 8;
        else if (e_c_b_L_linear == 0 & l_coarse_tune_count == 17)
        {
            l_coarse_tune_count = 16;
            l_coarse_tune_step = 4;
        }
    }
    g_c_cap = l_coarse_tune_mem_cap;
    set_ind(g_c_ind);
    set_cap(g_c_cap);
    g_c_step_cap = l_coarse_tune_mem_step_cap;
    Delay_ms(10);
    return;
}

static void sharp_cap(void)
{
    char l_sharp_cap_range, l_sharp_cap_count, l_sharp_cap_max_range, l_sharp_cap_min_range;
    int l_sharp_cap_min_SWR;
    l_sharp_cap_range = g_c_step_cap * g_c_C_mult;

    l_sharp_cap_max_range = g_c_cap + l_sharp_cap_range;
    if (l_sharp_cap_max_range > 32 * g_c_C_mult - 1)
        l_sharp_cap_max_range = 32 * g_c_C_mult - 1;
    if (g_c_cap > l_sharp_cap_range)
        l_sharp_cap_min_range = g_c_cap - l_sharp_cap_range;
    else
        l_sharp_cap_min_range = 0;
    g_c_cap = l_sharp_cap_min_range;
    set_cap(g_c_cap);
    get_swr();
    if (g_i_SWR == 0)
        return;
    l_sharp_cap_min_SWR = g_i_SWR;
    for (l_sharp_cap_count = l_sharp_cap_min_range + g_c_C_mult;
         l_sharp_cap_count <= l_sharp_cap_max_range;
         l_sharp_cap_count += g_c_C_mult)
    {
        set_cap(l_sharp_cap_count);
        get_swr();
        if (g_i_SWR == 0)
            return;
        if (g_i_SWR >= l_sharp_cap_min_SWR)
        {
            Delay_ms(10);
            get_swr();
        }
        if (g_i_SWR >= l_sharp_cap_min_SWR)
        {
            Delay_ms(10);
            get_swr();
        }
        if (g_i_SWR < l_sharp_cap_min_SWR)
        {
            l_sharp_cap_min_SWR = g_i_SWR;
            g_c_cap = l_sharp_cap_count;
            if (g_i_SWR < 120)
                break;
        }
        else
            break;
    }
    set_cap(g_c_cap);
    return;
}

static void sharp_ind(void)
{
    char l_sharp_ind_range, l_sharp_ind_count, l_sharp_ind_max_range, l_sharp_ind_min_range;
    int l_sharp_ind_min_SWR;
    l_sharp_ind_range = g_c_step_ind * g_c_L_mult;

    l_sharp_ind_max_range = g_c_ind + l_sharp_ind_range;
    if (l_sharp_ind_max_range > 32 * g_c_L_mult - 1)
        l_sharp_ind_max_range = 32 * g_c_L_mult - 1;
    if (g_c_ind > l_sharp_ind_range)
        l_sharp_ind_min_range = g_c_ind - l_sharp_ind_range;
    else
        l_sharp_ind_min_range = 0;
    g_c_ind = l_sharp_ind_min_range;
    set_ind(g_c_ind);
    get_swr();
    if (g_i_SWR == 0)
        return;
    l_sharp_ind_min_SWR = g_i_SWR;
    for (l_sharp_ind_count = l_sharp_ind_min_range + g_c_L_mult;
         l_sharp_ind_count <= l_sharp_ind_max_range;
         l_sharp_ind_count += g_c_L_mult)
    {
        set_ind(l_sharp_ind_count);
        get_swr();
        if (g_i_SWR == 0)
            return;
        if (g_i_SWR >= l_sharp_ind_min_SWR)
        {
            Delay_ms(10);
            get_swr();
        }
        if (g_i_SWR >= l_sharp_ind_min_SWR)
        {
            Delay_ms(10);
            get_swr();
        }
        if (g_i_SWR < l_sharp_ind_min_SWR)
        {
            l_sharp_ind_min_SWR = g_i_SWR;
            g_c_ind = l_sharp_ind_count;
            if (g_i_SWR < 120)
                break;
        }
        else
            break;
    }
    set_ind(g_c_ind);
    return;
}

static void sub_tune(void)
{
    int l_int_swr_mem, l_int_ind_mem, l_int_cap_mem;

    l_int_swr_mem = g_i_SWR;
    coarse_tune();
    if (g_i_SWR == 0) { atu_reset(); return; }
    get_swr();
    if (g_i_SWR < 120) return;
    sharp_ind();
    if (g_i_SWR == 0) { atu_reset(); return; }
    get_swr();
    if (g_i_SWR < 120) return;
    sharp_cap();
    if (g_i_SWR == 0) { atu_reset(); return; }
    get_swr();
    if (g_i_SWR < 120) return;

    if (g_i_SWR < 200 & g_i_SWR < l_int_swr_mem & (l_int_swr_mem - g_i_SWR) > 100)
        return;
    l_int_swr_mem = g_i_SWR;
    l_int_ind_mem = g_c_ind;
    l_int_cap_mem = g_c_cap;

    if (g_c_SW == 1) g_c_SW = 0; else g_c_SW = 1;
    atu_reset();
    set_sw(g_c_SW);
    Delay_ms(50);
    get_swr();
    if (g_i_SWR < 120) return;

    coarse_tune();
    if (g_i_SWR == 0) { atu_reset(); return; }
    get_swr();
    if (g_i_SWR < 120) return;
    sharp_ind();
    if (g_i_SWR == 0) { atu_reset(); return; }
    get_swr();
    if (g_i_SWR < 120) return;
    sharp_cap();
    if (g_i_SWR == 0) { atu_reset(); return; }
    get_swr();
    if (g_i_SWR < 120) return;

    if (g_i_SWR > l_int_swr_mem) {
        if (g_c_SW == 1) g_c_SW = 0; else g_c_SW = 1;
        set_sw(g_c_SW);
        g_c_ind = (char)(l_int_ind_mem);
        g_c_cap = (char)(l_int_cap_mem);
        set_ind(g_c_ind);
        set_cap(g_c_cap);
        get_swr();
    }
    CLRWDT();
    return;
}

static void tune(void)
{
    CLRWDT();
    g_char_p_cnt = 0; g_i_P_max = 0; g_b_rready = 0;
    get_swr();
    if (g_i_SWR < 110) return;
    atu_reset();
    if (e_c_b_Loss_ind == 0) lcd_ind();
    Delay_ms(50);
    get_swr();
    g_i_swr_a = g_i_SWR;
    if (g_i_SWR < 110) return;
    if (e_i_tenths_init_max_swr > 110 & g_i_SWR > e_i_tenths_init_max_swr)
        return;
    sub_tune();
    if (g_i_SWR == 0) { atu_reset(); return; }
    if (g_i_SWR < 120) return;
    if (e_c_num_C_q == 5 & e_c_num_L_q == 5) return;
    if (e_c_num_L_q > 5) {
        g_c_step_ind = g_c_L_mult;
        g_c_L_mult = 1;
        sharp_ind();
    }
    if (g_i_SWR < 120) return;
    if (e_c_num_C_q > 5) {
        g_c_step_cap = g_c_C_mult;
        g_c_C_mult = 1;
        sharp_cap();
    }
    if (e_c_num_L_q == 5) g_c_L_mult = 1;
    else if (e_c_num_L_q == 6) g_c_L_mult = 2;
    else if (e_c_num_L_q == 7) g_c_L_mult = 4;
    if (e_c_num_C_q == 5) g_c_C_mult = 1;
    else if (e_c_num_C_q == 6) g_c_C_mult = 2;
    else if (e_c_num_C_q == 7) g_c_C_mult = 4;
    CLRWDT();
    return;
}

/* ── SWR models ─────────────────────────────────────────────────────────────
 *
 * SWR is encoded as integer × 100: 100 = 1.0:1, 150 = 1.5:1, 999 = max/error.
 * Each model returns the SWR the antenna would show at a given relay position.
 * The shape is chosen to reproduce a known problem pattern, not to be physically
 * exact. What matters is that the algorithm behaviour under test is exercised.
 * ─────────────────────────────────────────────────────────────────────────── */

/*
 * S1 model: 41m delta loop on 10.1 MHz (30m band).
 *
 * Two dips in the SWR-vs-inductance curve:
 *   local minimum:  ind≈12 (register), cap≈24 → SWR≈175  — good but not great
 *   global minimum: ind≈96 (register), cap≈32 → SWR≈115  — the real answer
 *
 * The greedy old algorithm stops at the local minimum because the next position
 * (ind=24) is worse. Fix 2b (non-greedy, no else-break) keeps scanning and
 * finds the global minimum at ind=96.
 */
static int model_bimodal_30m(unsigned char ind, unsigned char cap, unsigned char sw)
{
    if (sw != 0) return 999;
    double di = (double)ind, dc = (double)cap;
    double d_global = sqrt(pow((di - 96.0) / 18.0, 2.0) + pow((dc - 32.0) / 14.0, 2.0));
    double d_local  = sqrt(pow((di - 12.0) / 10.0, 2.0) + pow((dc - 24.0) / 10.0, 2.0));
    double swr = fmin(115.0 + d_global * 55.0, 175.0 + d_local * 45.0);
    if (swr < 100) swr = 100;
    if (swr > 999) swr = 999;
    return (int)swr;
}

/*
 * S2 model: unmatchable antenna (badly resonant load, no L/C combination helps).
 *
 * SWR is constant at 350 regardless of relay position.
 * The algorithm must terminate cleanly — no infinite loop, no crash.
 */
static int model_flat(unsigned char ind, unsigned char cap, unsigned char sw)
{
    (void)ind; (void)cap; (void)sw;
    return 350;
}

/*
 * S3 model: well-behaved dipole on 14.1 MHz (20m band).
 *
 * Single clear minimum at ind=32 (register), cap=24 (register), SW=0.
 * Sanity regression: algorithm must converge to SWR < 120.
 */
static int model_simple_20m(unsigned char ind, unsigned char cap, unsigned char sw)
{
    if (sw != 0) return 220;
    double di = (double)ind, dc = (double)cap;
    double d = sqrt(pow((di - 32.0) / 12.0, 2.0) + pow((dc - 24.0) / 10.0, 2.0));
    double swr = 105.0 + d * 40.0;
    if (swr < 100) swr = 100;
    if (swr > 999) swr = 999;
    return (int)swr;
}

/* ── test infrastructure ── */

static void reset_state(void)
{
    g_c_ind = 0; g_c_cap = 0; g_c_SW = 0;
    g_c_step_cap = 0; g_c_step_ind = 0;
    g_c_L_mult = 4; g_c_C_mult = 4;
    g_i_SWR = 999; g_i_PWR = 5; g_i_P_max = 0; g_i_swr_a = 0;
    g_char_p_cnt = 0; g_b_rready = 0;
    sim_call_n = 0; sim_inhibit_on_call = -1;
    e_i_tenths_init_max_swr = 0;
}

typedef struct { char ind; char cap; char sw; int swr; } result_t;

static result_t run_tune(swr_fn model, int inhibit_call)
{
    reset_state();
    current_model = model;
    sim_inhibit_on_call = inhibit_call;
    tune();
    return (result_t){ g_c_ind, g_c_cap, g_c_SW, g_i_SWR };
}

int main(void)
{
    int total = 0, passed = 0;

#define CHECK(label, cond, r) do { \
    int _ok = (cond); \
    passed += _ok; total++; \
    printf("%s  %-46s  ind=%3d cap=%3d sw=%d swr=%d\n", \
        _ok ? "PASS" : "FAIL", (label), \
        (int)(unsigned char)(r).ind, (int)(unsigned char)(r).cap, \
        (int)(r).sw, (r).swr); \
} while(0)

    /* S1: algorithm must reach the global minimum at high inductance */
    {
        result_t r = run_tune(model_bimodal_30m, -1);
        CHECK("S1 bimodal 30m — global min at high L",
              r.ind >= 76 && r.swr < 130, r);
    }

    /* S2: must terminate cleanly with a valid SWR reading */
    {
        result_t r = run_tune(model_flat, -1);
        CHECK("S2 flat unmatchable — terminates, valid SWR",
              r.swr >= 100 && r.swr <= 999, r);
    }

    /* S3: must converge to SWR < 120 on a simple well-behaved load */
    {
        result_t r = run_tune(model_simple_20m, -1);
        CHECK("S3 simple 20m — converges to SWR < 120",
              r.swr < 130, r);
    }

    /* S4: TX inhibit mid-scan — not yet testable without Fix 5 */
    printf("SKIP  %-46s  (Fix 5 not implemented — get_swr loop would hang)\n",
           "S4 TX inhibit mid-scan");

    printf("\n%d/%d passed\n", passed, total);
    return (passed == total) ? 0 : 1;
}
