/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef BL_TRIAL_H_
#define BL_TRIAL_H_

#include <stdbool.h>
#include <stdint.h>

#define BL_TRIAL_MAGIC        (0x54524941u)
#define BL_TRIAL_RESET_LIMIT  (3u)
#define BL_TRIAL_STATE_ADDR   (0x20001000u)

typedef struct
{
    uint32_t magic;
    uint32_t image_crc32;
    uint32_t watchdog_reset_count;
    uint32_t demo_fault;
} bl_trial_state_t;

extern volatile bl_trial_state_t g_bl_trial_state;

#define BL_TRIAL_STATE (&g_bl_trial_state)

static inline bool BL_TrialStateMatches(const volatile bl_trial_state_t *state, uint32_t image_crc32)
{
    return (state->magic == BL_TRIAL_MAGIC) && (state->image_crc32 == image_crc32);
}

static inline void BL_TrialEnableDemoFault(volatile bl_trial_state_t *state, uint32_t image_crc32)
{
    if (!BL_TrialStateMatches(state, image_crc32))
    {
        state->magic = BL_TRIAL_MAGIC;
        state->image_crc32 = image_crc32;
        state->watchdog_reset_count = 0u;
    }

    state->demo_fault = 1u;
}

static inline bool BL_TrialDemoFaultEnabled(const volatile bl_trial_state_t *state, uint32_t image_crc32)
{
    return BL_TrialStateMatches(state, image_crc32) && (state->demo_fault != 0u);
}

static inline bool BL_TrialMarkHealthy(volatile bl_trial_state_t *state, uint32_t image_crc32)
{
    if (!BL_TrialStateMatches(state, image_crc32))
    {
        return false;
    }

    state->watchdog_reset_count = 0u;
    return true;
}

#endif /* BL_TRIAL_H_ */
