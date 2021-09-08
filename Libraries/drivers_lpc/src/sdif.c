/**
  ******************************************************************************
  * @file    sdif.c
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.5.28
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#include "common.h"
#include "sdif.h"

#define kSDIF_DataTransferError (SDIF_RINTSTS_FRUN_MASK | SDIF_RINTSTS_SBE_MASK | SDIF_RINTSTS_HLE_MASK | SDIF_RINTSTS_DRTO_BDS_MASK)

void print_cmd(sdif_cmd_t *cmd)
{
    LIB_TRACE("CMD:%d\r\n", cmd->index);
    LIB_TRACE("    arg:0x%X\r\n", cmd->argument);
    LIB_TRACE("    flag:0x%X\r\n", cmd->flags);
    LIB_TRACE("    bsize:%d bcnt:%d\r\n", cmd->block_size, cmd->block_cnt);
    LIB_TRACE("    resp:0x%X 0x%X 0x%X 0x%X\r\n", cmd->response[0], cmd->response[1], cmd->response[2], cmd->response[3]);
}

static uint32_t SDIF_WaitCmdDone(sdif_cmd_t *cmd)
{
    uint32_t status = 0U;
    do
    {
        status = SDIF->RINTSTS;
        if(status & (SDIF_RINTSTS_RE_MASK | SDIF_RINTSTS_RCRC_MASK | SDIF_RINTSTS_RTO_BAR_MASK | SDIF_RINTSTS_HLE_MASK))
        {
            SDIF->RINTSTS &= SDIF_RINTSTS_RE_MASK | SDIF_RINTSTS_RCRC_MASK | SDIF_RINTSTS_RTO_BAR_MASK | SDIF_RINTSTS_HLE_MASK;
            return CH_ERR;
        }
    } while ((status & SDIF_RINTSTS_CDONE_MASK) != SDIF_RINTSTS_CDONE_MASK);

    /* clear the command done bit */
    SDIF->RINTSTS &= SDIF_RINTSTS_CDONE_MASK;
    
    cmd->response[0] = SDIF->RESP[0];
    cmd->response[1] = SDIF->RESP[1];
    cmd->response[2] = SDIF->RESP[2];
    cmd->response[3] = SDIF->RESP[3];
    return CH_OK;
}

uint32_t SDIF_SendCmd(sdif_cmd_t *cmd, uint32_t timeout)
{
    //print_cmd(cmd);
    SDIF->CMDARG = cmd->argument;

    if(cmd->flags & SDIF_CMD_DATA_EXPECTED_MASK)
    {
        SDIF->BLKSIZ = cmd->block_size;
        SDIF->BYTCNT = cmd->block_size*cmd->block_cnt;
    }
    SDIF->CMD = SDIF_CMD_CMD_INDEX(cmd->index) | SDIF_CMD_USE_HOLD_REG_MASK | SDIF_CMD_WAIT_PRVDATA_COMPLETE_MASK | SDIF_CMD_START_CMD_MASK | (cmd->flags & (~SDIF_CMD_CMD_INDEX_MASK));

    cmd->response[0] = 0;
    cmd->response[1] = 0;
    cmd->response[2] = 0;
    cmd->response[3] = 0;
    
    /* wait start_cmd bit auto clear within timeout */
    while ((SDIF->CMD & SDIF_CMD_START_CMD_MASK) == SDIF_CMD_START_CMD_MASK)
    {
        if (!timeout)
        {
            break;
        }
        --timeout;
    }

    if(timeout)
    {
        return SDIF_WaitCmdDone(cmd);
    }
    
    LIB_TRACE("SDIO command send timeout\r\n");
    return CH_ERR;
}


#define SDIF_INDENTIFICATION_MODE_SAMPLE_DELAY (0X17U)
#define SDIF_INDENTIFICATION_MODE_DRV_DELAY (0X17U)
#define SDIF_HIGHSPEED_25MHZ_SAMPLE_DELAY (0x10U)
#define SDIF_HIGHSPEED_25MHZ_DRV_DELAY (0x10U)

void SDIF_ConfigClockDelay(uint32_t target_HZ, uint32_t divider)
{
    /*config the clock delay and pharse shift
     *should config the clk_in_drv,
     *clk_in_sample to meet the min hold and
     *setup time
     */
    if (target_HZ <= 400*1000)
    {
        /*min hold time:5ns
        * min setup time: 5ns
        * delay = (x+1)*250ps
        */
        SYSCON->SDIOCLKCTRL = SYSCON_SDIOCLKCTRL_CCLK_SAMPLE_DELAY_ACTIVE_MASK |
                              SYSCON_SDIOCLKCTRL_CCLK_SAMPLE_DELAY(SDIF_INDENTIFICATION_MODE_SAMPLE_DELAY) |
                              SYSCON_SDIOCLKCTRL_CCLK_DRV_DELAY_ACTIVE_MASK |
                              SYSCON_SDIOCLKCTRL_CCLK_DRV_DELAY(SDIF_INDENTIFICATION_MODE_DRV_DELAY);
    }
    else
    {
        /*
        * user need to pay attention to this parameter
        * can be change the setting for you card and board
        * min hold time:5ns
        * min setup time: 5ns
        * delay = (x+1)*250ps
        */
        SYSCON->SDIOCLKCTRL = SYSCON_SDIOCLKCTRL_CCLK_SAMPLE_DELAY_ACTIVE_MASK |
                              SYSCON_SDIOCLKCTRL_CCLK_SAMPLE_DELAY(SDIF_HIGHSPEED_25MHZ_SAMPLE_DELAY) |
                              SYSCON_SDIOCLKCTRL_CCLK_DRV_DELAY_ACTIVE_MASK |
                              SYSCON_SDIOCLKCTRL_CCLK_DRV_DELAY(SDIF_HIGHSPEED_25MHZ_DRV_DELAY);
        /* means the input clock = 2 * card clock,
        * can use clock pharse shift tech
        */
        if (divider == 1U)
        {
            SYSCON->SDIOCLKCTRL |= SYSCON_SDIOCLKCTRL_PHASE_ACTIVE_MASK |
                                   SYSCON_SDIOCLKCTRL_CCLK_SAMPLE_PHASE(1) |
                                   SYSCON_SDIOCLKCTRL_CCLK_DRV_PHASE(1);
        }
    }
}

void SDIF_SetClock(uint32_t src_clk, uint32_t hz)
{
    uint32_t div;
    
    SDIF->CLKENA &= ~SDIF_CLKENA_CCLK_ENABLE_MASK;
    
    SDIF->CMDARG = 0x00;
    SDIF->CMD = SDIF_CMD_CMD_INDEX(0) | SDIF_CMD_START_CMD_MASK | SDIF_CMD_WAIT_PRVDATA_COMPLETE_MASK | SDIF_CMD_UPDATE_CLOCK_REGISTERS_ONLY_MASK;
    
    div = (src_clk / (SYSCON->SDIOCLKDIV + 1) / hz + 1U) / 2U;
    LIB_TRACE("SDIF->CLKDIV:0x%X\r\n", div);
    SDIF->CLKDIV = SDIF_CLKDIV_CLK_DIVIDER0(div);
    SDIF->CLKENA |= SDIF_CLKENA_CCLK_ENABLE_MASK;
    
    SDIF->CMDARG = 0x00;
    SDIF->CMD = SDIF_CMD_CMD_INDEX(0) | SDIF_CMD_START_CMD_MASK | SDIF_CMD_WAIT_PRVDATA_COMPLETE_MASK | SDIF_CMD_UPDATE_CLOCK_REGISTERS_ONLY_MASK;
    
    SDIF_ConfigClockDelay(hz, div);
}

void SDIF_SetBusWidth(uint8_t width_bit)
{
    if(width_bit == 1)
    {
        SDIF->CTYPE = 0;
    }
    
    if(width_bit == 4)
    {
        SDIF->CTYPE = 1;
    }
}

void SDIF_Init(void)
{
    /* clock setup: SDIF clock = main_clock/(SYSCON->SDIOCLKDIV+1) */
    SYSCON->AHBCLKCTRL[2] |= SYSCON_AHBCLKCTRL_SDIO_MASK;
    SYSCON->SDIOCLKSEL = 0x00;
    SYSCON->SDIOCLKDIV = 1;
    
    /* reset & power */
    SDIF->BMOD |= SDIF_BMOD_SWR_MASK;
    SDIF->PWREN |= SDIF_PWREN_POWER_ENABLE_MASK;
    
    /* clear status */
    SDIF->RINTSTS = 0xFFFFFFFF;
    SDIF->IDSTS = 0xFFFFFFFF;
    SDIF->INTMASK = 0x1FFFFU;
    SDIF_SetClock(GetClock(kCoreClock), 400*1000);
}

/* SD card driver layer */

static void SD_DecodeCid(mmc_cid_t *cid, uint32_t *rawCid)
{
    cid->manufacturerID = (uint8_t)((rawCid[3U] & 0xFF000000U) >> 24U);
    cid->applicationID = (uint16_t)((rawCid[3U] & 0xFFFF00U) >> 8U);

    cid->productName[0U] = (uint8_t)((rawCid[3U] & 0xFFU));
    cid->productName[1U] = (uint8_t)((rawCid[2U] & 0xFF000000U) >> 24U);
    cid->productName[2U] = (uint8_t)((rawCid[2U] & 0xFF0000U) >> 16U);
    cid->productName[3U] = (uint8_t)((rawCid[2U] & 0xFF00U) >> 8U);
    cid->productName[4U] = (uint8_t)((rawCid[2U] & 0xFFU));

    cid->productVersion = (uint8_t)((rawCid[1U] & 0xFF000000U) >> 24U);

    cid->productSerialNumber = (uint32_t)((rawCid[1U] & 0xFFFFFFU) << 8U);
    cid->productSerialNumber |= (uint32_t)((rawCid[0U] & 0xFF000000U) >> 24U);

    cid->manufacturerData = (uint16_t)((rawCid[0U] & 0xFFF00U) >> 8U);
}

static void SD_DecodeCsd(sd_card_t *card, uint32_t *rawCsd)
{
    sd_csd_t *csd;

    csd = &(card->csd);
    csd->csdStructure = (uint8_t)((rawCsd[3U] & 0xC0000000U) >> 30U);
    csd->dataReadAccessTime1 = (uint8_t)((rawCsd[3U] & 0xFF0000U) >> 16U);
    csd->dataReadAccessTime2 = (uint8_t)((rawCsd[3U] & 0xFF00U) >> 8U);
    csd->transferSpeed = (uint8_t)(rawCsd[3U] & 0xFFU);
    csd->cardCommandClass = (uint16_t)((rawCsd[2U] & 0xFFF00000U) >> 20U);
    csd->readBlockLength = (uint8_t)((rawCsd[2U] & 0xF0000U) >> 16U);
//    if (rawCsd[2U] & 0x8000U)
//    {
//        csd->flags |= kSD_CsdReadBlockPartialFlag;
//    }
//    if (rawCsd[2U] & 0x4000U)
//    {
//        csd->flags |= kSD_CsdReadBlockPartialFlag;
//    }
//    if (rawCsd[2U] & 0x2000U)
//    {
//        csd->flags |= kSD_CsdReadBlockMisalignFlag;
//    }
//    if (rawCsd[2U] & 0x1000U)
//    {
//        csd->flags |= kSD_CsdDsrImplementedFlag;
//    }
    switch (csd->csdStructure)
    {
        case 0:
            csd->deviceSize = (uint32_t)((rawCsd[2U] & 0x3FFU) << 2U);
            csd->deviceSize |= (uint32_t)((rawCsd[1U] & 0xC0000000U) >> 30U);
            csd->readCurrentVddMin = (uint8_t)((rawCsd[1U] & 0x38000000U) >> 27U);
            csd->readCurrentVddMax = (uint8_t)((rawCsd[1U] & 0x7000000U) >> 24U);
            csd->writeCurrentVddMin = (uint8_t)((rawCsd[1U] & 0xE00000U) >> 20U);
            csd->writeCurrentVddMax = (uint8_t)((rawCsd[1U] & 0x1C0000U) >> 18U);
            csd->deviceSizeMultiplier = (uint8_t)((rawCsd[1U] & 0x38000U) >> 15U);

            /* Get card total block count and block size. */
            card->block_cnt = ((csd->deviceSize + 1U) << (csd->deviceSizeMultiplier + 2U));
            card->block_size = (1U << (csd->readBlockLength));
            if (card->block_size != 512)
            {
                card->block_cnt = (card->block_cnt * card->block_size);
                card->block_size = 512;
                card->block_cnt = (card->block_cnt / card->block_size);
            }
            break;
        case 1:
            card->block_size = 512;

            csd->deviceSize = (uint32_t)((rawCsd[2U] & 0x3FU) << 16U);
            csd->deviceSize |= (uint32_t)((rawCsd[1U] & 0xFFFF0000U) >> 16U);
//            if (csd->deviceSize >= 0xFFFFU)
//            {
//                card->flags |= kSD_SupportSdxcFlag;
//            }

            card->block_cnt = ((csd->deviceSize + 1U) * 1024U);
            break;
        default:
            break;
    }
//    if ((uint8_t)((rawCsd[1U] & 0x4000U) >> 14U))
//    {
//        csd->flags |= kSD_CsdEraseBlockEnabledFlag;
//    }
    csd->eraseSectorSize = (uint8_t)((rawCsd[1U] & 0x3F80U) >> 7U);
    csd->writeProtectGroupSize = (uint8_t)(rawCsd[1U] & 0x7FU);
//    if ((uint8_t)(rawCsd[0U] & 0x80000000U))
//    {
//        csd->flags |= kSD_CsdWriteProtectGroupEnabledFlag;
//    }
    csd->writeSpeedFactor = (uint8_t)((rawCsd[0U] & 0x1C000000U) >> 26U);
    csd->writeBlockLength = (uint8_t)((rawCsd[0U] & 0x3C00000U) >> 22U);
//    if ((uint8_t)((rawCsd[0U] & 0x200000U) >> 21U))
//    {
//        csd->flags |= kSD_CsdWriteBlockPartialFlag;
//    }
//    if ((uint8_t)((rawCsd[0U] & 0x8000U) >> 15U))
//    {
//        csd->flags |= kSD_CsdFileFormatGroupFlag;
//    }
//    if ((uint8_t)((rawCsd[0U] & 0x4000U) >> 14U))
//    {
//        csd->flags |= kSD_CsdCopyFlag;
//    }
//    if ((uint8_t)((rawCsd[0U] & 0x2000U) >> 13U))
//    {
//        csd->flags |= kSD_CsdPermanentWriteProtectFlag;
//    }
//    if ((uint8_t)((rawCsd[0U] & 0x1000U) >> 12U))
//    {
//        csd->flags |= kSD_CsdTemporaryWriteProtectFlag;
//    }
    csd->fileFormat = (uint8_t)((rawCsd[0U] & 0xC00U) >> 10U);
}

uint32_t SDCardInit(sd_card_t *card, uint32_t hz)
{
    sdif_cmd_t cmd = {0};
    int ret;
    
    /* CMD0 -> CMD8 -> while(CMD55+ACMD41) ->CMD2 -> CMD3 ->CMD9 -> CMD7-> CMD16->(CMD55+ACMD6) */
    
    cmd.index = 0;      /* go idle */
    cmd.flags = SDIF_CMD_USE_HOLD_REG_MASK;
    cmd.argument = 0x00;
    ret = SDIF_SendCmd(&cmd, 0xFFFF);
    LIB_TRACE("ret:%s\r\n", (ret == CH_OK)?("ok"):("err"));
    
    cmd.index = 8;      /* Send Interface Condition Command */
    cmd.flags = SDIF_CMD_RESPONSE_EXPECT_MASK | SDIF_CMD_CHECK_RESPONSE_CRC_MASK;
    cmd.argument = 0x1AAU;
    ret = SDIF_SendCmd(&cmd, 0xFFFF);
    LIB_TRACE("ret:%s\r\n", (ret == CH_OK)?("ok"):("err"));
    
    /* trap if not SDHC or demaged card */
    if((cmd.response[0] & 0xAA) != 0xAA)
    {
        LIB_TRACE("bad or not SDHC card\r\n");
        return CH_ERR;
    }
    
    while(1)
    {
        cmd.index = 55; /* enable Application-Specific Command APP_CMD (CMD55) */
        cmd.flags =  SDIF_CMD_RESPONSE_EXPECT_MASK | SDIF_CMD_CHECK_RESPONSE_CRC_MASK;
        cmd.argument = 0;
        ret = SDIF_SendCmd(&cmd, 0xFFFF);
        printf("ret:%s\r\n", (ret == CH_OK)?("ok"):("err"));
        
        /* trap if APP_CMD status is not enabled */
        if((cmd.response[0] & (1<<5)) == 0)
        {
            LIB_TRACE("app cmd status still not enabled\r\n");
            return CH_ERR;
        }
        
        cmd.index = 41; /* SD_APP_OP_COND  ACMD41 */
        cmd.flags = SDIF_CMD_WAIT_PRVDATA_COMPLETE_MASK | SDIF_CMD_RESPONSE_EXPECT_MASK;
        cmd.argument = 0x40300000;
        ret = SDIF_SendCmd(&cmd, 0xFFFF);
        LIB_TRACE("ret:%s\r\n", (ret == CH_OK)?("ok"):("err"));
        
        if(cmd.response[0] & 0x80000000)
        {
            break;
        }
    }

    cmd.index = 2; /* ALL_SEND_CID */
    cmd.flags =  SDIF_CMD_RESPONSE_EXPECT_MASK | SDIF_CMD_CHECK_RESPONSE_CRC_MASK | SDIF_CMD_RESPONSE_LENGTH_MASK;
    cmd.argument = 0;
    ret = SDIF_SendCmd(&cmd, 0xFFFF);
    LIB_TRACE("ret:%s\r\n", (ret == CH_OK)?("ok"):("err"));
    if(ret)
    {
        while(1);
    }
    
    SD_DecodeCid(&card->cid, cmd.response);
    LIB_TRACE("%s\r\n", card->cid.productName);
    LIB_TRACE("%d\r\n", card->cid.manufacturerData);
    
    
    cmd.index = 3; /* SEND_RELATIVE_ADDR */
    cmd.flags =  SDIF_CMD_RESPONSE_EXPECT_MASK | SDIF_CMD_CHECK_RESPONSE_CRC_MASK;
    cmd.argument = 0;
    ret = SDIF_SendCmd(&cmd, 0xFFFF);
    LIB_TRACE("ret:%s\r\n", (ret == CH_OK)?("ok"):("err"));
    
    card->rel_addr = cmd.response[0] >> 16;
    LIB_TRACE("card rel addr:0x%X\r\n", card->rel_addr);
    
    cmd.index = 9; /* SEND_CSD */
    cmd.flags =  SDIF_CMD_RESPONSE_EXPECT_MASK | SDIF_CMD_CHECK_RESPONSE_CRC_MASK | SDIF_CMD_RESPONSE_LENGTH_MASK;
    cmd.argument = card->rel_addr << 16;
    ret = SDIF_SendCmd(&cmd, 0xFFFF);
    LIB_TRACE("ret:%s\r\n", (ret == CH_OK)?("ok"):("err"));
    
    SD_DecodeCsd(card, cmd.response);
    LIB_TRACE("block_cnt:%d\r\n", card->block_cnt);
    LIB_TRACE("block_size:%d\r\n", card->block_size);
    LIB_TRACE("total size:%dMB\r\n", (card->block_cnt / 1024 / 1024) * card->block_size);
    
    cmd.index = 7; /* SELECT/DESELECT_CARD */
    cmd.flags =  SDIF_CMD_RESPONSE_EXPECT_MASK | SDIF_CMD_CHECK_RESPONSE_CRC_MASK;
    cmd.argument = card->rel_addr << 16;
    ret = SDIF_SendCmd(&cmd, 0xFFFF);
    LIB_TRACE("ret:%s\r\n", (ret == CH_OK)?("ok"):("err"));
    
    cmd.index = 16; /* SET_BLOCKLEN */
    cmd.flags =  SDIF_CMD_RESPONSE_EXPECT_MASK | SDIF_CMD_CHECK_RESPONSE_CRC_MASK;
    cmd.argument = 512;
    ret = SDIF_SendCmd(&cmd, 0xFFFF);
    LIB_TRACE("ret:%s\r\n", (ret == CH_OK)?("ok"):("err"));

    SDIF_SetClock(GetClock(kCoreClock), hz);    
    return CH_OK;
}

uint32_t SD_SetCardDataBusWidth(sd_card_t *card, uint8_t width_in_bit)
{
    int ret;
    sdif_cmd_t cmd = {0};
    cmd.index = 55; /* enable Application-Specific Command APP_CMD (CMD55) */
    cmd.flags =  SDIF_CMD_RESPONSE_EXPECT_MASK | SDIF_CMD_CHECK_RESPONSE_CRC_MASK;
    cmd.argument = card->rel_addr << 16;
    ret = SDIF_SendCmd(&cmd, 0xFFFF);
    
    /* trap if APP_CMD status is not enabled */
    if((cmd.response[0] & (1<<5)) == 0)
    {
        LIB_TRACE("app cmd status still not enabled\r\n");
        return CH_ERR;
    }
        
    cmd.index = 6; /* SET_BUS_WIDTH  ACMD6 */
    cmd.flags = SDIF_CMD_WAIT_PRVDATA_COMPLETE_MASK | SDIF_CMD_RESPONSE_EXPECT_MASK | SDIF_CMD_CHECK_RESPONSE_CRC_MASK;
    switch(width_in_bit)
    {
        case 1:
            cmd.argument = 0;
            break;
        case 4:
            cmd.argument = 2;
            break;
        default:
            LIB_TRACE("wrong bus width\r\n");
            break;
    }
    
    SDIF_SendCmd(&cmd, 0xFFFF);
    return CH_OK;
}

bool SD_IsCardInsert(void)
{
    if(SDIF->CDETECT)
    {
        return false;
    }
    return true;
}

bool SD_IsCardIdle(sd_card_t *card)
{
    int ret;
    
    sdif_cmd_t cmd = {0};
    cmd.index = 13; /* SEND_STATUS */
    cmd.flags =  SDIF_CMD_RESPONSE_EXPECT_MASK | SDIF_CMD_CHECK_RESPONSE_CRC_MASK;
    cmd.argument = card->rel_addr << 16;
    ret = SDIF_SendCmd(&cmd, 0xFFFF);
    
    /* see SD Specifications Part 1 Physical Layer Simplified Specification Version 2.00 September 25, 2006 */
    if(ret == CH_OK && cmd.response[0] & (1<<8) && ((cmd.response[0] & 0x00001E00U) >> 9) != 7)
    {
        return true;
    }
    return false;
}

uint32_t SD_ReadBlock(sd_card_t *card, uint32_t block_addr, uint32_t block_cnt, uint8_t *buf)
{
    int i, status;
    bool transferOver = false;
    sdif_cmd_t cmd = {0};
    
    while(SD_IsCardIdle(card) == false);

    cmd.flags =  SDIF_CMD_RESPONSE_EXPECT_MASK | SDIF_CMD_WAIT_PRVDATA_COMPLETE_MASK | SDIF_CMD_CHECK_RESPONSE_CRC_MASK | SDIF_CMD_DATA_EXPECTED_MASK;
    cmd.argument = block_addr;
    cmd.block_cnt = block_cnt;
    cmd.block_size = 512;
    
    if(cmd.block_cnt > 1)
    {
        cmd.index = 18; /* READ_MULTIPLE_BLOCK  */
        cmd.flags |= SDIF_CMD_SEND_AUTO_STOP_MASK;
    }
    else
    {
        cmd.index = 17; /* READ_SINGLE_BLOCK  */
    }
    
    SDIF_SendCmd(&cmd, 0xFFFF);
    
    uint32_t *p = (uint32_t *)buf;
    
    for(i=0; i<cmd.block_cnt*cmd.block_size/(sizeof(uint32_t)); i++)
    {
        do
        {
            status = SDIF->RINTSTS;
        } while((status & (SDIF_RINTSTS_RXDR_MASK | SDIF_RINTSTS_DTO_MASK)) == 0 && !transferOver);
        
        if(status & SDIF_RINTSTS_DTO_MASK)
        {
            transferOver = true;
        }
        
        *p++ = SDIF->FIFO[0];
        
        SDIF->RINTSTS &= status;
    }
    return CH_OK;
}



uint32_t SD_WriteBlock(sd_card_t *card, uint32_t block_addr, uint32_t block_cnt, uint8_t *buf)
{
    int i;
    sdif_cmd_t cmd = {0};
    
    while(SD_IsCardIdle(card) == false);

    cmd.flags =  SDIF_CMD_RESPONSE_EXPECT_MASK | SDIF_CMD_WAIT_PRVDATA_COMPLETE_MASK | SDIF_CMD_CHECK_RESPONSE_CRC_MASK | SDIF_CMD_DATA_EXPECTED_MASK | SDIF_CMD_READ_WRITE_MASK;
    cmd.argument = block_addr;
    cmd.block_cnt = block_cnt;
    cmd.block_size = 512;
    
    if(cmd.block_cnt > 1)
    {
        cmd.index = 25; /* WRITE_MULTIPLE_BLOCK  */
        cmd.flags |= SDIF_CMD_SEND_AUTO_STOP_MASK;
    }
    else
    {
        cmd.index = 24; /* WRITE_BLOCK  */
    }
    
    SDIF_SendCmd(&cmd, 0xFFFF);
    
    uint32_t *p = (uint32_t *)buf;

    for(i=0; i<cmd.block_size*cmd.block_cnt/(sizeof(uint32_t)); i++)
    {
        /* waiting for Tx ready */
        while((SDIF->RINTSTS & SDIF_RINTSTS_TXDR_MASK) == 0);
        SDIF->RINTSTS &= SDIF_RINTSTS_TXDR_MASK;
        
        if(SDIF->RINTSTS & kSDIF_DataTransferError)
        {
            return CH_ERR;
        }
        
        SDIF->FIFO[0] = *p++;
    }
    
    /* waitting for transfer complete */
    while((SDIF->RINTSTS & SDIF_RINTSTS_DTO_MASK) == 0);
    SDIF->RINTSTS &= SDIF_RINTSTS_DTO_MASK;
    
    return CH_OK;
}

uint32_t SD_BlockTest(sd_card_t *card, uint8_t *buf, uint32_t block_addr, uint32_t block_cnt)
{
    int i;
    

    for(i=0; i<block_cnt*512; i++)
    {
        buf[i] = i & 0xFF;
    }
         
    SD_WriteBlock(card, block_addr, block_cnt, buf);

    for(i=0; i<block_cnt*512; i++)
    {
        buf[i] = 0;
    }
    SD_ReadBlock(card, block_addr, block_cnt, buf);
    
    for(i=0; i<block_cnt*512; i++)
    {
        if(buf[i] != (i & 0xFF))
        {
            LIB_TRACE("sector%d error buf[%X]:%X\r\n", block_addr, i, buf[i]);
            return CH_ERR;
        }
    }
    return CH_OK;
}


