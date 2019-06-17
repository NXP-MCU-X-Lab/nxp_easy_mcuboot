/*
 * i2c.c
 *
 *  Created on: This day
 *      Author: The Creator
 */
#include "common.h"
#include "i2c.h"

#define I2C_MSTCTL_MSTCONTINUE_MASK              (0x1U)
#define I2C_MSTCTL_MSTCONTINUE_SHIFT             (0U)
#define I2C_MSTCTL_MSTCONTINUE(x)                (((uint32_t)(((uint32_t)(x)) << I2C_MSTCTL_MSTCONTINUE_SHIFT)) & I2C_MSTCTL_MSTCONTINUE_MASK)
#define I2C_MSTCTL_MSTSTART_MASK                 (0x2U)
#define I2C_MSTCTL_MSTSTART_SHIFT                (1U)
#define I2C_MSTCTL_MSTSTART(x)                   (((uint32_t)(((uint32_t)(x)) << I2C_MSTCTL_MSTSTART_SHIFT)) & I2C_MSTCTL_MSTSTART_MASK)
#define I2C_MSTCTL_MSTSTOP_MASK                  (0x4U)
#define I2C_MSTCTL_MSTSTOP_SHIFT                 (2U)
#define I2C_MSTCTL_MSTSTOP(x)                    (((uint32_t)(((uint32_t)(x)) << I2C_MSTCTL_MSTSTOP_SHIFT)) & I2C_MSTCTL_MSTSTOP_MASK)

#define I2C_STAT_MSTSTATE_MASK                   (0xEU)
#define I2C_STAT_MSTSTATE_SHIFT                  (1U)
#define I2C_STAT_MSTARBLOSS_MASK                 (0x10U)
#define I2C_STAT_MSTARBLOSS_SHIFT                (4U)
#define I2C_STAT_MSTSTSTPERR_MASK                (0x40U)
#define I2C_STAT_MSTSTSTPERR_SHIFT               (6U)
#define I2C_MSTTIME_MSTSCLLOW_MASK               (0x7U)
#define I2C_MSTTIME_MSTSCLLOW_SHIFT              (0U)
#define I2C_MSTTIME_MSTSCLLOW(x)                 (((uint32_t)(((uint32_t)(x)) << I2C_MSTTIME_MSTSCLLOW_SHIFT)) & I2C_MSTTIME_MSTSCLLOW_MASK)
#define I2C_MSTTIME_MSTSCLHIGH_MASK              (0x70U)
#define I2C_MSTTIME_MSTSCLHIGH_SHIFT             (4U)
#define I2C_MSTTIME_MSTSCLHIGH(x)                (((uint32_t)(((uint32_t)(x)) << I2C_MSTTIME_MSTSCLHIGH_SHIFT)) & I2C_MSTTIME_MSTSCLHIGH_MASK)
#define GET_MACHINE_CODE(x) ((x & I2C_STAT_MSTSTATE_MASK) >> I2C_STAT_MSTSTATE_SHIFT)
#define STAT_MSTPEND  		(1 << 0)

#define LPC_I2C0_Type   LPC_I2C_TypeDef

static LPC_I2C0_Type* const I2CBases[] = {LPC_I2C0, LPC_I2C1};


void I2C_SetBaudRate(uint32_t instance, uint32_t baud)
{
    uint32_t clk;

    /* i2c use main clock */
    #if defined(LPC802)
    LPC_SYSCON->I2C0CLKSEL = 1;
    #endif
    
    clk = GetClock(kCoreClock);
    
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

    I2CBases[instance]->DIV = (best_div - 1);
    I2CBases[instance]->MSTTIME = I2C_MSTTIME_MSTSCLLOW(best_scl - 2u) | I2C_MSTTIME_MSTSCLHIGH(best_scl - 2u);
}


static uint32_t I2C_PendingStatusWait(uint32_t instance)
{
    uint32_t status;

    do
    {
        status = I2CBases[instance]->STAT;
    } while ((status & STAT_MSTPEND) == 0);

    /* Clear controller state. */
    I2CBases[instance]->STAT = (1<<4) | (1<<6);

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

uint32_t I2C_Init(uint32_t MAP, uint32_t baudrate)
{
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<5);
    
    I2C_SetBaudRate(MAP, baudrate);
    
    /* master and slave enable */
    I2CBases[MAP]->CFG = (1<<0) | (1<<1);
    return CH_OK;
}

uint32_t I2C_SetSlvAddr(uint32_t instance, uint32_t slot, uint32_t addr)
{
    switch(slot)
    {
        case 0:
            I2CBases[instance]->SLVADR0 = (addr << 1) | 0;
            break;
        case 1:
            I2CBases[instance]->SLVADR1 = (addr << 1) | 0;
            break;
        case 2:
            I2CBases[instance]->SLVADR2 = (addr << 1) | 0;
            break;
        case 3:
            I2CBases[instance]->SLVADR3 = (addr << 1) | 0;
            break;
    }
    return CH_OK;
}

uint32_t I2C_SetIntMode(uint32_t instance, I2C_Int_t mode, bool val)
{
    switch(mode)
    {
        case kI2C_SlvPending:
            (val)?(I2CBases[instance]->INTENSET = (1<<8)):(I2CBases[instance]->INTENCLR = (1<<8));
            break;
        case kI2C_SlvDesel:
            (val)?(I2CBases[instance]->INTENSET = (1<<15)):(I2CBases[instance]->INTENCLR = (1<<15));
            break;
    }

    NVIC_EnableIRQ(I2C0_IRQn);
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

