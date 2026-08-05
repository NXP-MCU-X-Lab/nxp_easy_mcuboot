#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "bl_image.h"
#include "bl_trial.h"

static int expect_action(const char *name, bl_image_action_t actual, bl_image_action_t expected)
{
    if (actual != expected)
    {
        (void)printf("FAIL %s: actual=%d expected=%d\n", name, (int)actual, (int)expected);
        return 1;
    }

    return 0;
}

static int expect_bool(const char *name, bool actual, bool expected)
{
    if (actual != expected)
    {
        (void)printf("FAIL %s: actual=%d expected=%d\n", name, (int)actual, (int)expected);
        return 1;
    }

    return 0;
}

static int expect_u32(const char *name, uint32_t actual, uint32_t expected)
{
    if (actual != expected)
    {
        (void)printf("FAIL %s: actual=0x%08X expected=0x%08X\n",
                     name,
                     (unsigned int)actual,
                     (unsigned int)expected);
        return 1;
    }

    return 0;
}

int main(void)
{
    bl_image_info_t app = {0};
    bl_image_info_t backup = {0};
    bl_trial_state_t trial = {0};
    int failures = 0;

    app.version = 2u;
    app.image_size = 0x1000u;
    app.image_crc32 = 0x11111111u;
    backup.version = 1u;
    backup.image_size = 0x1000u;
    backup.image_crc32 = 0x22222222u;

    failures += expect_action("both invalid",
                              BL_ImageSelectAction(false, &app, false, &backup),
                              kBL_ImageActionStayInBootloader);
    failures += expect_action("only app valid",
                              BL_ImageSelectAction(true, &app, false, &backup),
                              kBL_ImageActionBootApp);
    failures += expect_action("only backup valid",
                              BL_ImageSelectAction(false, &app, true, &backup),
                              kBL_ImageActionRestoreBackup);
    failures += expect_action("app version newer",
                              BL_ImageSelectAction(true, &app, true, &backup),
                              kBL_ImageActionBootApp);

    app.version = 1u;
    backup.version = 2u;
    failures += expect_action("backup version newer",
                              BL_ImageSelectAction(true, &app, true, &backup),
                              kBL_ImageActionRestoreBackup);

    app.version = 2u;
    backup.version = 2u;
    app.image_crc32 = backup.image_crc32;
    failures += expect_action("identical images",
                              BL_ImageSelectAction(true, &app, true, &backup),
                              kBL_ImageActionBootApp);

    app.image_crc32 = 0x33333333u;
    failures += expect_action("same version different crc",
                              BL_ImageSelectAction(true, &app, true, &backup),
                              kBL_ImageActionRestoreBackup);

    app.image_crc32 = backup.image_crc32;
    app.image_size = 0x1010u;
    failures += expect_action("same version different size",
                              BL_ImageSelectAction(true, &app, true, &backup),
                              kBL_ImageActionRestoreBackup);

    failures += expect_bool("full app erase",
                            BL_ImageEraseRequestIsValid(0x4000u, 0x6000u),
                            true);
    failures += expect_bool("KinetisFlashTool image erase",
                            BL_ImageEraseRequestIsValid(0x4000u, 0x2050u),
                            true);
    failures += expect_bool("minimum app erase",
                            BL_ImageEraseRequestIsValid(0x4000u, 1u),
                            true);
    failures += expect_bool("zero length app erase",
                            BL_ImageEraseRequestIsValid(0x4000u, 0u),
                            false);
    failures += expect_bool("shifted app erase",
                            BL_ImageEraseRequestIsValid(0x6000u, 0x4000u),
                            false);
    failures += expect_bool("oversized app erase",
                            BL_ImageEraseRequestIsValid(0x4000u, 0x8000u),
                            false);

    failures += expect_u32("trial state address", BL_TRIAL_STATE_ADDR, 0x20001000u);
    failures += expect_u32("trial state size", (uint32_t)sizeof(trial), 16u);

    trial.magic = BL_TRIAL_MAGIC;
    trial.image_crc32 = 0x11111111u;
    trial.watchdog_reset_count = 2u;
    trial.demo_fault = 1u;
    failures += expect_bool("non-trial clears state",
                            BL_TrialShouldRestore(false, true, 0x11111111u, &trial),
                            false);
    failures += expect_u32("non-trial magic cleared", trial.magic, 0u);
    failures += expect_u32("non-trial count cleared", trial.watchdog_reset_count, 0u);
    failures += expect_u32("non-trial fault cleared", trial.demo_fault, 0u);

    trial.magic = BL_TRIAL_MAGIC;
    trial.image_crc32 = 0x11111111u;
    trial.watchdog_reset_count = 2u;
    trial.demo_fault = 1u;
    failures += expect_bool("non-watchdog reset restarts trial",
                            BL_TrialShouldRestore(true, false, 0x11111111u, &trial),
                            false);
    failures += expect_u32("non-watchdog count cleared", trial.watchdog_reset_count, 0u);
    failures += expect_u32("non-watchdog fault cleared", trial.demo_fault, 0u);

    trial.magic = BL_TRIAL_MAGIC;
    trial.image_crc32 = 0x11111111u;
    trial.watchdog_reset_count = 2u;
    trial.demo_fault = 1u;
    failures += expect_bool("new image first watchdog reset",
                            BL_TrialShouldRestore(true, true, 0x22222222u, &trial),
                            false);
    failures += expect_u32("new image crc recorded", trial.image_crc32, 0x22222222u);
    failures += expect_u32("new image count starts at one", trial.watchdog_reset_count, 1u);
    failures += expect_u32("new image fault cleared", trial.demo_fault, 0u);

    trial.demo_fault = 1u;
    failures += expect_bool("second watchdog reset keeps app",
                            BL_TrialShouldRestore(true, true, 0x22222222u, &trial),
                            false);
    failures += expect_u32("second watchdog count", trial.watchdog_reset_count, 2u);
    failures += expect_u32("watchdog keeps fault flag", trial.demo_fault, 1u);

    failures += expect_bool("third watchdog reset restores backup",
                            BL_TrialShouldRestore(true, true, 0x22222222u, &trial),
                            true);
    failures += expect_u32("rollback clears magic", trial.magic, 0u);
    failures += expect_u32("rollback clears count", trial.watchdog_reset_count, 0u);
    failures += expect_u32("rollback clears fault", trial.demo_fault, 0u);

    trial.magic = BL_TRIAL_MAGIC;
    trial.image_crc32 = 0x33333333u;
    trial.watchdog_reset_count = 1u;
    trial.demo_fault = 0u;
    BL_TrialEnableDemoFault(&trial, 0x33333333u);
    BL_TrialEnableDemoFault(&trial, 0x33333333u);
    failures += expect_u32("repeated fault keeps watchdog count", trial.watchdog_reset_count, 1u);
    failures += expect_u32("fault flag enabled", trial.demo_fault, 1u);

    trial.demo_fault = 0u;
    trial.watchdog_reset_count = 2u;
    failures += expect_bool("matching healthy state accepted",
                            BL_TrialMarkHealthy(&trial, 0x33333333u),
                            true);
    failures += expect_u32("healthy state clears watchdog count", trial.watchdog_reset_count, 0u);

    if (failures == 0)
    {
        (void)printf("PASS bl_image_policy_test\n");
    }

    return failures;
}
