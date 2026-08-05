ASSERT(_image_end <= 0x4000, "Bootloader image exceeds 16 KB")
ASSERT((_pvHeapLimit + _StackSize) <= 0x20001000, "Bootloader low RAM overlaps trial state")
ASSERT(_vStackBase >= 0x20001010, "Bootloader stack overlaps trial state")
g_bl_trial_state = ABSOLUTE(0x20001000);
