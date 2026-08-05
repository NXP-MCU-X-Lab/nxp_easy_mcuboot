#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "bl_image.h"

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

int main(void)
{
    bl_image_info_t app = {0};
    bl_image_info_t backup = {0};
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
    failures += expect_bool("partial app erase",
                            BL_ImageEraseRequestIsValid(0x4000u, 0x2000u),
                            false);
    failures += expect_bool("shifted app erase",
                            BL_ImageEraseRequestIsValid(0x6000u, 0x4000u),
                            false);
    failures += expect_bool("oversized app erase",
                            BL_ImageEraseRequestIsValid(0x4000u, 0x8000u),
                            false);

    if (failures == 0)
    {
        (void)printf("PASS bl_image_policy_test\n");
    }

    return failures;
}
