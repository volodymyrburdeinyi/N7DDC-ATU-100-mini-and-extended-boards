Review the ATU-100 EXT board firmware (PIC16F1938 V_3.2, MPLAB XC8 port) for fix correctness and regressions.

Read both files before doing anything else:
- $REPO/ATU_100_EXT_board/FirmWare_PIC16F1938/1938_EXT_MPLAB_sources_V_3.2/main.h
- $REPO/ATU_100_EXT_board/FirmWare_PIC16F1938/1938_EXT_MPLAB_sources_V_3.2/main.c

Where $REPO is the git repo root (find it with `git rev-parse --show-toplevel`).

Note: this codebase uses WA1RCT's XC8 naming conventions — globals are prefixed g_i_ (int),
g_c_ (char), e_c_b_ (EEPROM bool), e_i_ (EEPROM int). Local variables use l_ prefix.

---

## Part 1 — Fix verification (static)

Check each fix by reading the actual code. Quote the relevant line as evidence.

**Fix 1 — lcd_swr() parameter bug**
PASS: `lcd_swr()` contains `if (swr == 0)` (lowercase parameter)
FAIL: contains `if (g_i_SWR == 0)` (global variable)

**Fix 2a — coarse_cap() non-greedy**
PASS if ALL hold:
- initializer is `l_coarse_cap_min_swr = g_i_SWR;` (not `g_i_SWR + g_i_SWR / 20`)
- update inside if-block is `l_coarse_cap_min_swr = g_i_SWR;`
- `l_coarse_cap_count += l_coarse_cap_step;` appears OUTSIDE the `if` block
- NO `else break;` after the `if (g_i_SWR < l_coarse_cap_min_swr)` closing brace

**Fix 2b — coarse_tune() non-greedy**
PASS if ALL hold:
- initializer is `l_coarse_tune_min_swr = 9999;`
- update inside if-block is `l_coarse_tune_min_swr = g_i_SWR;`
- `l_coarse_tune_count += l_coarse_tune_step;` appears OUTSIDE the `if` block
- NO `else break;` after the `if (g_i_SWR < l_coarse_tune_min_swr)` closing brace

**Fix 3 — sub_tune() SW-revert live SWR**
PASS: SW-revert block calls `get_swr();`
FAIL: that position contains `g_i_SWR = l_int_swr_mem;`

---

## Part 2 — Scenario regression matrix

Trace through the algorithm logic for each scenario. Mark PASS, REGRESSION, or NOTE.

**Scenario 1 — Delta loop 41m at 10.1 MHz (30m band)**
Profile: bimodal SWR-vs-L; global minimum at high-L, local minimum at low-L.
Check: coarse_tune() with l_coarse_tune_min_swr=9999 and no else-break walks all L positions.
First low-L local minimum recorded but not final — scan continues and overwrites with high-L global minimum.
PASS if non-greedy. REGRESSION if else-break crept back in.

**Scenario 2 — Resonant dipole at 14 MHz (20m, simple mismatch)**
Profile: unimodal SWR-vs-L; single clear minimum.
Check: Fix 2 must not produce worse result than greedy. Strict comparison still records best position.
No regression.

**Scenario 3 — EFHW 40m wire at 7 MHz (high-Z, ~2000Ω)**
Profile: monotonically improving SWR; requires max L.
Check: both greedy and non-greedy reach max-L. Equivalent result. No regression.

**Scenario 4 — Unmatchable antenna (flat SWR landscape)**
Profile: SWR same everywhere.
Check: l_coarse_tune_min_swr=9999 means first position sets min (9999→SWR). Subsequent equal values
do NOT pass strict `<` test — first position wins. Old SWR+SWR/20 init would cause last-wins. PASS.

**Scenario 5 — SW-position revert, post-revert SWR check (Fix 3)**
Profile: SW-toggle experiment ran, second position was worse, revert executes.
Check: after set_cap(g_c_cap), get_swr() called for live measurement. PASS if get_swr() present.

**Scenario 6 — No-power / QMX+ in normal TX mode (SWR≥3 inhibits)**
Profile: during tune, TRX cuts power at SWR≥3. g_i_PWR drops below e_i_watts_min_for_start.
Check: get_swr() enters wait loop. Pre-existing stall risk. Fixes 1-3 do not make this worse. NOTE.

---

## Output format

**Fix verification:**
| Fix | Status | Evidence (quoted line) |
|-----|--------|------------------------|
| Fix 1 lcd_swr | PASS/FAIL | `if (swr == 0)` at main.c:NNN |
| Fix 2a coarse_cap | PASS/FAIL | key lines quoted |
| Fix 2b coarse_tune | PASS/FAIL | key lines quoted |
| Fix 3 sub_tune revert | PASS/FAIL | `get_swr()` at main.h:NNN |

**Scenario matrix:**
| # | Scenario | Verdict | Key reasoning |
|---|----------|---------|---------------|
| 1 | Delta loop 41m 30m | PASS/REGRESSION | one sentence |
| 2 | Resonant dipole 20m | PASS/REGRESSION | one sentence |
| 3 | EFHW 40m high-Z | PASS/REGRESSION | one sentence |
| 4 | Flat SWR unmatchable | PASS/REGRESSION | one sentence |
| 5 | SW-revert Fix 3 | PASS/REGRESSION | one sentence |
| 6 | No-power stall | PASS/NOTE | one sentence |

**Overall: PASS** (all fixes intact, no regressions detected) or **FAIL** with list of issues.
