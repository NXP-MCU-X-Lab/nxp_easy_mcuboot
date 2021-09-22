/**
  ******************************************************************************
  * @file    emc.c
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.5.28
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#include "common.h"
#include "emc.h"

/*! @brief Define the chip numbers for dynamic and static memory devices. */
#define EMC_STATIC_MEMDEV_NUM (4U)
#define EMC_DYNAMIC_MEMDEV_NUM (4U)
#define EMC_ADDRMAP_SHIFT EMC_DYNAMIC_DYNAMICCONFIG_AM0_SHIFT
#define EMC_ADDRMAP_MASK (EMC_DYNAMIC_DYNAMICCONFIG_AM0_MASK | EMC_DYNAMIC_DYNAMICCONFIG_AM1_MASK)
#define EMC_ADDRMAP(x) (((uint32_t)(((uint32_t)(x)) << EMC_ADDRMAP_SHIFT)) & EMC_ADDRMAP_MASK)
#define EMC_HZ_ONEMHZ (1000000U)
#define EMC_MILLISECS_ONESEC (1000U)
#define EMC_SDRAM_MODE_CL_SHIFT (4U)
#define EMC_SDRAM_MODE_CL_MASK (0x70U)

#define EMC_HZ_ONEMHZ (1000000U)


#define EMC_REFRESH_CLOCK_PARAM (16U)
#define EMC_SDRAM_WAIT_CYCLES (2000U)
#define EMC_DYNCTL_COLUMNBASE_OFFSET (0U)
#define EMC_DYNCTL_COLUMNBASE_MASK (0x3U)
#define EMC_DYNCTL_COLUMNPLUS_OFFSET (3U)
#define EMC_DYNCTL_COLUMNPLUS_MASK (0x18U)
#define EMC_DYNCTL_BUSWIDTH_MASK (0x80U)
#define EMC_DYNCTL_BUSADDRMAP_MASK (0x20U)
#define EMC_DYNCTL_DEVBANKS_BITS_MASK (0x1cU)
#define EMC_SDRAM_BANKCS_BA0_MASK (uint32_t)(0x2000)
#define EMC_SDRAM_BANKCS_BA1_MASK (uint32_t)(0x4000)
#define EMC_SDRAM_BANKCS_BA_MASK (EMC_SDRAM_BANKCS_BA0_MASK | EMC_SDRAM_BANKCS_BA1_MASK)
#define EMC_DIV_ROUND_UP(n, m) (((n) + (m)-1) / (m))

void EMC_Init(void)
{
    SYSCON->AHBCLKCTRL[2] |= SYSCON_AHBCLKCTRL_EMC_MASK;

    /* Reset the EMC. */
    SYSCON->PRESETCTRL[2] |= SYSCON_PRESETCTRL_EMC_RESET_MASK;
    SYSCON->PRESETCTRL[2] &= ~SYSCON_PRESETCTRL_EMC_RESET_MASK;

    /* EMC clock = core clock / 2. */
    SYSCON->EMCCLKDIV = SYSCON_EMCCLKDIV_DIV(1);

    SYSCON->EMCSYSCTRL = SYSCON_EMCSYSCTRL_EMCFBCLKINSEL(0);

    /* Set the endian mode to little endian */
    EMC->CONFIG = 0;
    /* Enable the EMC module with normal memory map mode and normal work mode. */
    EMC->CONTROL = EMC_CONTROL_E_MASK;
}

static uint32_t EMC_CalculateTimerCycles(EMC_Type *base, uint32_t timer_Ns, uint32_t plus)
{
    uint32_t cycles;

    cycles = GetClock(kEMCClock) / EMC_HZ_ONEMHZ * timer_Ns;
    cycles = EMC_DIV_ROUND_UP(cycles, EMC_MILLISECS_ONESEC); /* Round up. */

    /* Decrese according to the plus. */
    if (cycles >= plus)
    {
        cycles = cycles - plus;
    }
    else
    {
        cycles = 0;
    }

    return cycles;
}


static uint32_t EMC_ModeOffset(uint32_t addrMap)
{
    uint8_t offset = 0;
    uint32_t columbase = addrMap & EMC_DYNCTL_COLUMNBASE_MASK;

    /* First calculate the column length. */
    if (columbase == 0x10)
    {
        offset = 8;
    }
    else
    {
        if (!columbase)
        {
            offset = 9;
        }
        else
        {
            offset = 8;
        }
        /* Add column length increase check. */
        if (((addrMap & EMC_DYNCTL_COLUMNPLUS_MASK) >> EMC_DYNCTL_COLUMNPLUS_OFFSET) == 1)
        {
            offset += 1;
        }
        else if (((addrMap & EMC_DYNCTL_COLUMNPLUS_MASK) >> EMC_DYNCTL_COLUMNPLUS_OFFSET) == 2)
        {
            offset += 2;
        }
        else
        {
            /* To avoid MISRA rule 14.10 error. */
        }
    }

    /* Add Buswidth/16. */
    if (addrMap & EMC_DYNCTL_BUSWIDTH_MASK)
    {
        offset += 2;
    }
    else
    {
        offset += 1;
    }

    /* Add bank select bit if the sdram address map mode is RBC(row-bank-column) mode. */
    if (!(addrMap & EMC_DYNCTL_BUSADDRMAP_MASK))
    {
        if (!(addrMap & EMC_DYNCTL_DEVBANKS_BITS_MASK))
        {
            offset += 1;
        }
        else
        {
            offset += 2;
        }
    }

    return offset;
}

void SDRAM_Init(emc_dynamic_timing_config_t *timing, emc_dynamic_chip_config_t *config)
{
    uint32_t count;
    uint8_t casLatency;
    uint32_t addr;
    uint32_t offset;
    uint32_t data;
    
    /* Setting for dynamic memory controller chip independent configuration. */
    EMC->DYNAMIC[0].DYNAMICCONFIG = EMC_DYNAMIC_DYNAMICCONFIG_MD(config->dynamicDevice) | EMC_ADDRMAP(config->devAddrMap);
            
    /* Abstract CAS latency from the sdram mode reigster setting values. */
    casLatency = (config->sdramModeReg & EMC_SDRAM_MODE_CL_MASK) >> EMC_SDRAM_MODE_CL_SHIFT;
    EMC->DYNAMIC[0].DYNAMICRASCAS = EMC_DYNAMIC_DYNAMICRASCAS_RAS(config->rAS_Nclk) | EMC_DYNAMIC_DYNAMICRASCAS_CAS(casLatency);
            
    /* Configure the Dynamic Memory controller timing/latency for all chips. */
    EMC->DYNAMICREADCONFIG = EMC_DYNAMICREADCONFIG_RD(timing->readConfig);
    EMC->DYNAMICRP = EMC_CalculateTimerCycles(EMC, timing->tRp_Ns, 1) & EMC_DYNAMICRP_TRP_MASK;
    EMC->DYNAMICRAS = EMC_CalculateTimerCycles(EMC, timing->tRas_Ns, 1) & EMC_DYNAMICRAS_TRAS_MASK;
    EMC->DYNAMICSREX = EMC_CalculateTimerCycles(EMC, timing->tSrex_Ns, 1) & EMC_DYNAMICSREX_TSREX_MASK;
    EMC->DYNAMICAPR = EMC_CalculateTimerCycles(EMC, timing->tApr_Ns, 1) & EMC_DYNAMICAPR_TAPR_MASK;
    EMC->DYNAMICDAL = EMC_CalculateTimerCycles(EMC, timing->tDal_Ns, 0) & EMC_DYNAMICDAL_TDAL_MASK;
    EMC->DYNAMICWR = EMC_CalculateTimerCycles(EMC, timing->tWr_Ns, 1) & EMC_DYNAMICWR_TWR_MASK;
    EMC->DYNAMICRC = EMC_CalculateTimerCycles(EMC, timing->tRc_Ns, 1) & EMC_DYNAMICRC_TRC_MASK;
    EMC->DYNAMICRFC = EMC_CalculateTimerCycles(EMC, timing->tRfc_Ns, 1) & EMC_DYNAMICRFC_TRFC_MASK;
    EMC->DYNAMICXSR = EMC_CalculateTimerCycles(EMC, timing->tXsr_Ns, 1) & EMC_DYNAMICXSR_TXSR_MASK;
    EMC->DYNAMICRRD = EMC_CalculateTimerCycles(EMC, timing->tRrd_Ns, 1) & EMC_DYNAMICRRD_TRRD_MASK;
    EMC->DYNAMICMRD = EMC_DYNAMICMRD_TMRD((timing->tMrd_Nclk > 0) ? timing->tMrd_Nclk - 1 : 0);

    
    /* Initialize the SDRAM.*/
    for (count = 0; count < EMC_SDRAM_WAIT_CYCLES; count++)
    {
    }
    /* Step 2. issue nop command. */
    EMC->DYNAMICCONTROL = 0x00000183;
    for (count = 0; count < EMC_SDRAM_WAIT_CYCLES; count++)
    {
    }
    
    /* Step 3. issue precharge all command. */
    EMC->DYNAMICCONTROL = 0x00000103;

    /* Step 4. issue two auto-refresh command. */
    EMC->DYNAMICREFRESH = 2;
    for (count = 0; count < EMC_SDRAM_WAIT_CYCLES / 2; count++)
    {
    }

    EMC->DYNAMICREFRESH = EMC_CalculateTimerCycles(EMC, timing->refreshPeriod_Nanosec, 0) / EMC_REFRESH_CLOCK_PARAM;

    /* Step 5. issue a mode command and set the mode value. */
    EMC->DYNAMICCONTROL = 0x00000083;
    
    /* Calculate the mode settings here and to reach the 8 auto-refresh time requirement. */
    /* Get the shift value first. */
    offset = EMC_ModeOffset(config->devAddrMap);
    addr = (EMC_DYCS0_BASE | ((uint32_t)(config->sdramModeReg & ~EMC_SDRAM_BANKCS_BA_MASK) << offset));
                
    /* Set the right mode setting value. */
    data = *(volatile uint32_t *)addr;
    data = data;
    
    /* Step 6. issue normal operation command. */
    EMC->DYNAMICCONTROL = 0x00000000; /* Issue NORMAL command */

    /* The buffer shall be disabled when do the sdram initialization and enabled after the initialization during normal opeation. */
    EMC->DYNAMIC[0].DYNAMICCONFIG |= EMC_DYNAMIC_DYNAMICCONFIG_B_MASK;
}

