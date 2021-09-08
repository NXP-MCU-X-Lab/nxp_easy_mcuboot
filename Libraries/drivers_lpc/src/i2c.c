/**
  ******************************************************************************
  * @file    i2c.c
  * @author  YANDLD
  * @version V3.0.0
  * @date    2015.6.21
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */

#include "common.h"
#include "i2c.h"

#define GET_MACHINE_CODE(x) ((x & I2C_STAT_MSTSTATE_MASK) >> I2C_STAT_MSTSTATE_SHIFT)


static FLEXCOMM_Type* const FLEXCOMMBases[] = FLEXCOMM_BASE_PTRS;
I2C_Type* const I2CBases[] = I2C_BASE_PTRS;
static const IRQn_Type I2C_IRQTbl[] = I2C_IRQS;

uint32_t I2C_SetClock(uint32_t instance)
{
    /* select 48 or 96M FROHFClock */
    SetFlexCommClk(instance, 1);
    return GetClock(kFROHfClock);
}

void I2C_SetBaudRate(uint32_t instance, uint32_t baud)
{
    uint32_t clk;

    clk = I2C_SetClock(instance);
    LIB_TRACE("I2C input clock:%dHz\r\n", clk);
    
    uint32_t scl, divider;
    uint32_t best_scl, best_div;
    uint32_t err, best_err;

    best_err = 0;

    for (scl = 9; scl >= 2; scl--)
    {
        /* calculated ideal divider value for given scl */
        divider = clk / (baud * scl * 2u);

        /* adjust it if it is out of range */
        divider = (divider > 0x10000u) ? 0x10000 : divider;

        /* calculate error */
        err = clk - (baud * scl * 2u * divider);
        if ((err < best_err) || (best_err == 0))
        {
            best_div = divider;
            best_scl = scl;
            best_err = err;
        }

        if ((err == 0) || (divider >= 0x10000u))
        {
            /* either exact value was found
               or divider is at its max (it would even greater in the next iteration for sure) */
            break;
        }
    }

    I2CBases[instance]->CLKDIV = I2C_CLKDIV_DIVVAL(best_div - 1);
    I2CBases[instance]->MSTTIME = I2C_MSTTIME_MSTSCLLOW(best_scl - 2u) | I2C_MSTTIME_MSTSCLHIGH(best_scl - 2u);
}


static uint32_t I2C_PendingStatusWait(uint32_t instance)
{
    uint32_t status;

    do
    {
        status = I2CBases[instance]->STAT;
    } while ((status & I2C_STAT_MSTPENDING_MASK) == 0);

    /* Clear controller state. */
    I2CBases[instance]->STAT = (I2C_STAT_MSTARBLOSS_MASK | I2C_STAT_MSTSTSTPERR_MASK);

    return status;
}

static uint32_t I2C_Start(uint32_t instance, uint8_t addr)
{
    I2C_PendingStatusWait(instance);
    
    /* Write Address and RW bit to data register */
    I2CBases[instance]->MSTDAT = addr;
    /* Start the transfer */
    I2CBases[instance]->MSTCTL = I2C_MSTCTL_MSTSTART_MASK;

    return CH_OK;
}

static uint32_t I2C_Stop(uint32_t instance)
{
    I2C_PendingStatusWait(instance);
    I2CBases[instance]->MSTCTL = I2C_MSTCTL_MSTSTOP_MASK;
    return CH_OK;
}

static uint32_t I2C_WriteBytes(uint32_t instance, uint8_t *buf, uint32_t len)
{
    int ret;
    uint32_t stat;
    
    ret = CH_OK;
    
    while(len--)
    {
        stat = I2C_PendingStatusWait(instance);
        
        if (stat & (I2C_STAT_MSTARBLOSS_MASK | I2C_STAT_MSTSTSTPERR_MASK))
        {
            return CH_ERR;
        }
        
        switch(GET_MACHINE_CODE(stat))
        {
            case 0: /* idle */
                break;
            case 2: /* tx ready */
                I2CBases[instance]->MSTDAT = *buf++;
                I2CBases[instance]->MSTCTL = I2C_MSTCTL_MSTCONTINUE_MASK;
                break;
            case 3: /* Nack */
                //LIB_TRACE("Nack in addr phase\r\n");
                ret = CH_ERR;
                break;
            case 4:
                //LIB_TRACE("NanK in data phase\r\n");
                ret = CH_ERR;
                break;
        }
        
        if(ret)
        {
            return ret;
        }
    }
    return ret;
}

uint32_t I2C_Write(uint32_t instance, uint8_t addr, uint8_t *buf, uint32_t len)
{
    int ret = CH_OK;
    
    addr <<= 1;
    
    /* start + addr(w) */
    I2C_Start(instance, (addr & (~0x01)));

    /* data */
    ret = I2C_WriteBytes(instance, buf, len);
    if(ret)
    {
        I2C_Stop(instance);
        return ret;
    }
    
    I2C_Stop(instance);
    return ret;
}

uint32_t I2C_BurstWrite(uint32_t instance ,uint8_t addr, uint32_t regAddr, uint32_t regLen, uint8_t *buf, uint32_t len)
{
    int ret = CH_OK;
    
    addr <<= 1;
    
    /* start + addr(w) */
    I2C_Start(instance, (addr & (~0x01)));

    /* reg addr phase */
    ret = I2C_WriteBytes(instance, (uint8_t *)&regAddr, regLen);
    if(ret)
    {
        I2C_Stop(instance);
        return ret;
    }
    
    /* data */
    ret = I2C_WriteBytes(instance, buf, len);
    if(ret)
    {
        I2C_Stop(instance);
        return ret;
    }
    
    I2C_Stop(instance);
    return ret;
}

uint32_t I2C_BurstRead(uint32_t instance, uint8_t addr, uint32_t regAddr, uint32_t regLen, uint8_t* buf, uint32_t len)
{
    uint32_t stat;
    int ret = CH_OK;
    
    addr <<= 1;
    
    /* start + addr(w) */
    I2C_Start(instance, (addr & (~0x01)));

    /* reg addr phase */
    ret = I2C_WriteBytes(instance, (uint8_t *)&regAddr, regLen);
    if(ret)
    {
        I2C_Stop(instance);
        return ret;
    }
    
    /* restart */
    I2C_Start(instance, (addr | 0x01));

    /* read data */
    while(len)
    {
        stat = I2C_PendingStatusWait(instance);
        
        if (stat & (I2C_STAT_MSTARBLOSS_MASK | I2C_STAT_MSTSTSTPERR_MASK))
        {
            ret = CH_ERR;
        }
        
        switch((stat & I2C_STAT_MSTSTATE_MASK) >> I2C_STAT_MSTSTATE_SHIFT)
        {
            case 0: /* idle */
                break;
            case 1: /* RX ready */
                *(buf++) = I2CBases[instance]->MSTDAT;
                (--len)?(I2CBases[instance]->MSTCTL = I2C_MSTCTL_MSTCONTINUE_MASK):(I2CBases[instance]->MSTCTL = I2C_MSTCTL_MSTSTOP_MASK);
                break;
            case 3: /* Nack */
            case 4:
                LIB_TRACE("NanK\r\n");
                ret = CH_ERR;
                break;
        }
        
        if(ret)
        {
            return ret;
        }
    }
    
    return ret;
}

uint32_t I2C_Read(uint32_t instance, uint8_t addr, uint8_t* buf, uint32_t len)
{
    uint32_t stat;
    int ret = CH_OK;
    
    addr <<= 1;
    
    /* start + addr(r) */
    I2C_Start(instance, (addr | 0x01));

    /* read data */
    while(len)
    {
        stat = I2C_PendingStatusWait(instance);
        
        if (stat & (I2C_STAT_MSTARBLOSS_MASK | I2C_STAT_MSTSTSTPERR_MASK))
        {
            ret = CH_ERR;
        }
        
        switch((stat & I2C_STAT_MSTSTATE_MASK) >> I2C_STAT_MSTSTATE_SHIFT)
        {
            case 0: /* idle */
                break;
            case 1: /* RX ready */
                *(buf++) = I2CBases[instance]->MSTDAT;
                (--len)?(I2CBases[instance]->MSTCTL = I2C_MSTCTL_MSTCONTINUE_MASK):(I2CBases[instance]->MSTCTL = I2C_MSTCTL_MSTSTOP_MASK);
                break;
            case 3: /* Nack */
            case 4:
                LIB_TRACE("NanK\r\n");
                ret = CH_ERR;
                break;
        }
        
        if(ret)
        {
            return ret;
        }
    }
    
    return ret;
}


/**
 * @brief  读取单个寄存器数值
 * @param  instance:
 *         @arg HW_I2C0 : I2C0模块
 *         @arg HW_I2C1 : I2C1模块
 *         @arg HW_I2C1 : I2C1模块
 * @param  addr: 目标设备地址
 * @param  regAddr: 寄存器地址
 * @param  buf: 读取的数据地址指针
 * @retval 0：成功；其它：错误
 */
uint32_t I2C_ReadReg(uint32_t instance, uint8_t addr, uint8_t regAddr, uint8_t* buf)
{
    return I2C_BurstRead(instance, addr, regAddr, 1, buf, 1);
}

/**
 * @brief  写单个寄存器
 * @param  instance:
 *         @arg HW_I2C0 : I2C0模块
 *         @arg HW_I2C1 : I2C1模块
 *         @arg HW_I2C1 : I2C1模块
 * @param  addr: 目标设备地址 8位地址
 * @param  regAddr: 寄存器地址
 * @param  data: 准备写入的数据
 * @retval 0：成功；其它：错误
 */
uint32_t I2C_WriteReg(uint32_t instance, uint8_t addr, uint8_t regAddr, uint8_t data)
{
    return I2C_BurstWrite(instance, addr, regAddr, 1, &data, 1);
}

uint32_t I2C_Init(uint32_t instance, uint32_t baudrate)
{
    
    SYSCON->AHBCLKCTRL[1] |= SYSCON_AHBCLKCTRL_FLEXCOMM0_MASK
                            | SYSCON_AHBCLKCTRL_FLEXCOMM1_MASK
                            | SYSCON_AHBCLKCTRL_FLEXCOMM2_MASK
                            | SYSCON_AHBCLKCTRL_FLEXCOMM3_MASK 
                            | SYSCON_AHBCLKCTRL_FLEXCOMM4_MASK
                            | SYSCON_AHBCLKCTRL_FLEXCOMM5_MASK
                            | SYSCON_AHBCLKCTRL_FLEXCOMM6_MASK
                            | SYSCON_AHBCLKCTRL_FLEXCOMM7_MASK;
    
    SYSCON->AHBCLKCTRL[2] |= SYSCON_AHBCLKCTRL_FLEXCOMM8_MASK | SYSCON_AHBCLKCTRL_FLEXCOMM9_MASK;
    
    /* select flexcomm to I2C */

    FLEXCOMMBases[instance]->PSELID &= ~FLEXCOMM_PSELID_PERSEL_MASK;
    FLEXCOMMBases[instance]->PSELID |= FLEXCOMM_PSELID_PERSEL(3);
    
    I2CBases[instance]->CFG |= I2C_CFG_MSTEN_MASK | I2C_CFG_SLVEN_MASK;
    I2C_SetBaudRate(instance, baudrate);
    
    return CH_OK;
}

uint32_t I2C_SetSalveAddr(uint32_t instance, uint8_t slot, uint8_t addr)
{
    I2CBases[instance]->SLVADR[slot] = addr;
    return CH_OK;
}

uint32_t I2C_SetIntMode(uint32_t instance, I2C_Int_t val)
{
    switch(val)
    {
        case kI2C_SlavePending:
        I2CBases[instance]->INTENSET = I2C_INTENSET_SLVPENDINGEN_MASK;
        break;
        case kI2C_SlaveDeselect:
        I2CBases[instance]->INTENSET = I2C_INTENSET_SLVDESELEN_MASK;
        break;
    }
    NVIC_EnableIRQ(I2C_IRQTbl[instance]);
    return CH_OK;
}

uint32_t I2C_Probe(uint32_t instance, uint8_t addr)
{
    uint32_t stat;
    int ret = CH_OK;

    addr <<= 1;
    
    I2C_Start(instance, addr);

    stat = I2C_PendingStatusWait(instance);
        
    if (stat & (I2C_STAT_MSTARBLOSS_MASK | I2C_STAT_MSTSTSTPERR_MASK))
    {
        return CH_ERR;
    }
        
    switch(GET_MACHINE_CODE(stat))
    {
        case 0: /* idle */
            break;
        case 2: /* tx ready */
            ret = CH_OK;
            break;
        case 3: /* Nack */
            //LIB_TRACE("Nack in addr phase\r\n");
            ret = CH_ERR;
            break;
        case 4:
            //LIB_TRACE("NanK in data phase\r\n");
            ret = CH_ERR;
            break;
    }
        
    I2C_Stop(instance);
    return ret;
}

/* 7 bit address */
void I2C_Scan(uint32_t instance)
{
    uint8_t i;
    uint8_t ret;
    for(i = 1; i < 127; i++)
    {
        ret = I2C_Probe(instance , i);
        if(!ret)
        {
            LIB_TRACE("ADDR:0x%02X(7BIT) | 0x%02X(8BIT) found!\r\n", i, i<<1);
        }
    }
}






