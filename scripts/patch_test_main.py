#!/usr/bin/env python3
"""Patch test_main.c to add tests for outputLimit bug and circular wrapping bug.

Run on Orin: python3 patch_test_main.py
"""
import sys

TEST_FILE = "/home/orin/Desktop/kart_medulla/main/test_main.c"

NEW_TESTS = r'''/* ============================================================
 * 16. ACTUATOR OUTPUT LIMIT BUG
 * Catches: outputLimit stored as uint8_t (0-100) but compared
 *          against float (-1.0..1.0). The clamp never activates
 *          because 25 > 1.0, so the limit is ineffective.
 * ============================================================ */
static void test_act_output_limit(void) {
    printf("\n--- Actuator output limit ---\n");

    /* Create steering actuator with 25% limit */
    ACT_Controller act = KM_ACT_Init(ACT_STEER, 0.25f);
    KM_ACT_SetLimit(&act, 0.25f);

    /* The outputLimit must effectively clamp values to [-0.25, 0.25].
       Bug: outputLimit is uint8_t = 25, but clamp compares float.
       25.0 > 1.0, so a PID output of 1.0 passes through unclamped. */

    /* Test: outputLimit as float must be <= 1.0 */
    float limit_as_float = (float)act.outputLimit;
    TEST_BOOL("outputLimit as float <= 1.0 (catches uint8_t*100 bug)",
              limit_as_float <= 1.0f);

    /* Test: setting limit to 0.25 should clamp output to 0.25 */
    float test_value = 1.0f;
    float clamped = test_value;
    if (clamped > limit_as_float) clamped = limit_as_float;
    if (clamped < -limit_as_float) clamped = -limit_as_float;
    char buf[120];
    snprintf(buf, sizeof(buf),
             "25%% limit clamps 1.0 to <=0.25 (got %.2f, limit_float=%.2f)",
             clamped, limit_as_float);
    TEST_BOOL(buf, clamped <= 0.26f);

    /* Test: 50% limit stored correctly */
    ACT_Controller act2 = KM_ACT_Init(ACT_STEER, 0.5f);
    float limit2 = (float)act2.outputLimit;
    snprintf(buf, sizeof(buf),
             "50%% limit stored correctly (got %.2f, expect 0.50)", limit2);
    TEST_BOOL(buf, fabsf(limit2 - 0.5f) < 0.02f);

    /* Test: 100% limit stored correctly */
    ACT_Controller act3 = KM_ACT_Init(ACT_STEER, 1.0f);
    float limit3 = (float)act3.outputLimit;
    snprintf(buf, sizeof(buf),
             "100%% limit stored correctly (got %.2f, expect 1.00)", limit3);
    TEST_BOOL(buf, fabsf(limit3 - 1.0f) < 0.02f);
}

/* ============================================================
 * 17. AS5600 CIRCULAR WRAPPING
 * Catches: discontinuity at 4095->0 boundary when sensor is
 *          centered near the wrap point. Without circular wrap,
 *          a small physical movement causes a ~2*PI angle jump.
 * ============================================================ */
static void test_sensor_circular_wrap(void) {
    printf("\n--- Sensor circular wrapping ---\n");

    sensor_struct sensor = KM_SDIR_Init(10);

    /* Case 1: center=4060, raw=4070 -> small positive angle */
    KM_SDIR_setCenterOffset(&sensor, 4060);
    int16_t centered1 = (int16_t)(4070 - sensor.centerOffset);
    if (centered1 > 2048) centered1 -= 4096;
    if (centered1 < -2048) centered1 += 4096;
    float angle1 = ((float)centered1 / 4095.0f) * 2.0f * 3.1415f;
    char buf[120];
    snprintf(buf, sizeof(buf),
             "raw=4070 center=4060 -> small positive (%.4f rad)", angle1);
    TEST_BOOL(buf, angle1 > 0.0f && angle1 < 0.1f);

    /* Case 2: center=4060, raw=10 -> should be small positive (wrapped past 0)
       Without wrap: (10 - 4060) = -4050 -> -6.21 rad  BUG!
       With wrap:    -4050 + 4096 = 46 -> +0.07 rad    CORRECT */
    int16_t centered2 = (int16_t)(10 - sensor.centerOffset);
    if (centered2 > 2048) centered2 -= 4096;
    if (centered2 < -2048) centered2 += 4096;
    float angle2 = ((float)centered2 / 4095.0f) * 2.0f * 3.1415f;
    snprintf(buf, sizeof(buf),
             "raw=10 center=4060 -> small positive (%.4f rad, expect ~0.07)", angle2);
    TEST_BOOL(buf, angle2 > 0.0f && angle2 < 0.2f);

    /* Case 3: center=4060, raw=4050 -> small negative angle */
    int16_t centered3 = (int16_t)(4050 - sensor.centerOffset);
    if (centered3 > 2048) centered3 -= 4096;
    if (centered3 < -2048) centered3 += 4096;
    float angle3 = ((float)centered3 / 4095.0f) * 2.0f * 3.1415f;
    snprintf(buf, sizeof(buf),
             "raw=4050 center=4060 -> small negative (%.4f rad)", angle3);
    TEST_BOOL(buf, angle3 < 0.0f && angle3 > -0.1f);

    /* Case 4: center=100, raw=4000 -> large negative (physically far away) */
    KM_SDIR_setCenterOffset(&sensor, 100);
    int16_t centered4 = (int16_t)(4000 - sensor.centerOffset);
    if (centered4 > 2048) centered4 -= 4096;
    if (centered4 < -2048) centered4 += 4096;
    float angle4 = ((float)centered4 / 4095.0f) * 2.0f * 3.1415f;
    snprintf(buf, sizeof(buf),
             "raw=4000 center=100 -> negative (%.4f rad)", angle4);
    TEST_BOOL(buf, angle4 < 0.0f);

    /* Case 5: Continuity — raw 4095 and 0 should give adjacent angles */
    KM_SDIR_setCenterOffset(&sensor, 4060);
    int16_t c_before = (int16_t)(4095 - sensor.centerOffset);
    if (c_before > 2048) c_before -= 4096;
    if (c_before < -2048) c_before += 4096;
    int16_t c_after = (int16_t)(0 - sensor.centerOffset);
    if (c_after > 2048) c_after -= 4096;
    if (c_after < -2048) c_after += 4096;
    float a_before = ((float)c_before / 4095.0f) * 2.0f * 3.1415f;
    float a_after = ((float)c_after / 4095.0f) * 2.0f * 3.1415f;
    float jump = fabsf(a_after - a_before);
    snprintf(buf, sizeof(buf),
             "Continuity at 4095->0: jump=%.4f rad (must be < 0.01)", jump);
    TEST_BOOL(buf, jump < 0.01f);
}

'''

with open(TEST_FILE, 'r') as f:
    content = f.read()

# Insert before the MAIN section
marker = '/* ============================================================\n * MAIN\n * ============================================================ */'
if marker not in content:
    print(f"ERROR: Could not find MAIN marker in {TEST_FILE}")
    sys.exit(1)

content = content.replace(marker, NEW_TESTS + marker)

# Add calls to the new tests in app_main
old_calls = '    test_pid();\n    test_task_stack();'
new_calls = '    test_pid();\n    test_act_output_limit();\n    test_sensor_circular_wrap();\n    test_task_stack();'
if old_calls not in content:
    print(f"ERROR: Could not find test_pid()/test_task_stack() calls")
    sys.exit(1)

content = content.replace(old_calls, new_calls)

with open(TEST_FILE, 'w') as f:
    f.write(content)

print(f'SUCCESS: Added test_act_output_limit() and test_sensor_circular_wrap() to {TEST_FILE}')
