/**
  ******************************************************************************
  * @file    gpio.c
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.5.28
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#include "common.h"
#include "gpio.h"


#define GPIO_BASES {LPC_GPIO_PORT};
//#define PIN_INT_BASES {LPC_PIN_INT};

//static LPC_PIN_INT_Type * const PININTBases[] = PIN_INT_BASES;


static const IRQn_Type GPIO_IrqTbl[] = 
{
    PIN_INT0_IRQn,
    PIN_INT1_IRQn,
    PIN_INT2_IRQn,
    PIN_INT3_IRQn,
//    PIN_INT4_IRQn,
//    PIN_INT5_IRQn,
//    PIN_INT6_IRQn,
//    PIN_INT7_IRQn,
};
    
/**
 * @brief  设置引脚的输入输出方向
 * @note   控制引脚是输出还是输入
 * @param  instance:
 *         @arg HW_GPIOx : GPIO端口号A-E
 * @param  pin : 引脚编号：0-31
 * @param  dir : 方向选择参数
 *         @arg 0 : 输入
 *         @arg 1 : 输出
 * @retval None
 */
void GPIO_SetPinDir(uint32_t instance, uint32_t pin, uint32_t dir)
{
    
    (dir == 1) ? (GPIO->DIR[instance] |= (1 << pin)):
    (GPIO->DIR[instance] &= ~(1 << pin));
}


 /**
 * @brief  初始化配置指定引脚的工作状态
 * @note   完成初始化一个引脚配置
 * @param  instance:
 *         @arg HW_GPIOx : GPIO端口号A-E
 * @param  pin : 引脚号码：0-31
 * @param  mode : 引脚操作模式
 *         @arg kGPIO_IFT : 悬浮输入
 *         @arg kGPIO_IPD : 下拉输入
 *         @arg kGPIO_IPU : 上拉输入
 *         @arg kGPIO_OOD : 开漏输出
 *         @arg kGPIO_OPP : 推挽输出
 * @retval None
 */
uint32_t GPIO_Init(uint32_t instance, uint32_t pin, GPIO_t mode)
{

    SYSCON->AHBCLKCTRL[0] |= SYSCON_AHBCLKCTRL_GPIO0_MASK
                            | SYSCON_AHBCLKCTRL_GPIO1_MASK
#if defined(SYSCON_AHBCLKCTRL_GPIO2_MASK)
                            | SYSCON_AHBCLKCTRL_GPIO2_MASK
                            | SYSCON_AHBCLKCTRL_GPIO3_MASK
#endif
                            | SYSCON_AHBCLKCTRL_PINT_MASK
                            | SYSCON_AHBCLKCTRL_GINT_MASK 
                            | SYSCON_AHBCLKCTRL_IOCON_MASK
                            | SYSCON_AHBCLKCTRL_INPUTMUX_MASK;

    IOCON->PIO[instance][pin] &= ~IOCON_PIO_MODE_MASK;
    SetPinMux(instance,  pin, 0);
    switch(mode)
    {
        case kGPIO_IFT:
            SetPinPull(instance, pin, 0xFF);
            GPIO_SetPinDir(instance, pin, 0);
            break;
        case kGPIO_IPU:
            SetPinPull(instance, pin, 1);
            GPIO_SetPinDir(instance, pin, 0);
            break;
        case kGPIO_IPD:
            SetPinPull(instance, pin, 0);
            GPIO_SetPinDir(instance, pin, 0);
            break;
        case kGPIO_OPPH:
        case kGPIO_OPPL:
            (mode == kGPIO_OPPH)? GPIO_PinWrite(instance, pin, 1):GPIO_PinWrite(instance, pin, 0);
            GPIO_SetPinDir(instance, pin, 1);
            break;
        default:
            break;
    }
    
    return instance;
}



 /**
 * @brief  指定引脚输出状态
 * @note   设置指定引脚输出为0或1
 * @param  instance:
 *         @arg HW_GPIOx : GPIO端口号A-E
 * @param  pin : 引脚号码：0-31
 * @param  data : 引脚输出
 *         @arg 0 : 输出0
 *         @arg 1 : 输出1
 * @retval None
 */
void GPIO_PinWrite(uint32_t instance, uint32_t pin, uint8_t data)
{
    (data) ? (GPIO->SET[instance] = (1 << pin)):
    (GPIO->CLR[instance] = (1 << pin));
}

 /**
 * @brief  读取指定引脚的状态
 * @note   无
 * @param  instance:
 *         @arg HW_GPIOx : GPIO端口号A-E
 * @param  pin : 引脚号码：0-31
 * @retval 返回引脚状态0或1
 */
uint32_t GPIO_PinRead(uint32_t instance, uint32_t pin)
{
    return ((GPIO->PIN[instance] >> pin) & 0x01);
}

 /**
 * @brief  翻转引脚的状态
 * @note   None
 * @param  instance:
 *         @arg HW_GPIOx : GPIO端口号x
 * @param  pin : 引脚号码：0-31
 * @retval None
 */
void GPIO_PinToggle(uint32_t instance, uint8_t pin)
{
    GPIO->NOT[instance] = (1 << pin);
}

 /**
 * @brief  读取引脚端口的状态
 * @note   None
 * @param  instance:
 *         @arg HW_GPIOx : GPIO端口号A-E
 * @retval 返回一个32位端口的数据
 */
uint32_t GPIO_ReadPort(uint32_t instance)
{
    return GPIO->PIN[instance];
}

 /**
 * @brief  修改端口的输出状态
 * @note   None
 * @param  instance:
 *         @arg HW_GPIOx : GPIO端口号A-E
 * @param  data : 32位的数据
 * @retval None
 */
void GPIO_WritePort(uint32_t instance, uint32_t data)
{
    GPIO->PIN[instance] = data;
}

int GPIO_AddIntToSocket(uint32_t instance, uint32_t pin, uint8_t socket)
{
    /* add pin to socket */
    INPUTMUX->PINTSEL[socket] = pin + (instance * 32);
    NVIC_EnableIRQ(GPIO_IrqTbl[socket]);
    return CH_OK;
}

int GPIO_SetIntMode(uint32_t socket, GPIO_Int_t mode, bool val)
{
    /* all socket use edge sensitive */
    PINT->ISEL &= ~(1<<socket);
    
    switch(mode)
    {
        case kGPIO_Int_RE:
            (val)?(PINT->IENR |= (1<<socket)):(PINT->IENR &= ~(1<<socket));
            break;
        case kGPIO_Int_FE:
            (val)?(PINT->IENF |= (1<<socket)):(PINT->IENF &= ~(1<<socket));
            break;
        case kGPIO_Int_EE:
            (val)?(PINT->IENR |= (1<<socket)):(PINT->IENR &= ~(1<<socket));
            (val)?(PINT->IENF |= (1<<socket)):(PINT->IENF &= ~(1<<socket));
            break;
    }
    
    PINT->IST = 0xFF;
    return CH_OK;
}

/* partten match mode*/
void GPIO_SetPMEMode(bool val)
{
    if(val)
    {
        PINT->PMCTRL = PINT_PMCTRL_SEL_PMATCH_MASK | PINT_PMCTRL_ENA_RXEV_MASK;
    }
    else
    {
        PINT->PMCTRL &= ~(PINT_PMCTRL_SEL_PMATCH_MASK | PINT_PMCTRL_ENA_RXEV_MASK);
    }
}

/* isep: is a endpoint, true if OR, false if AND */
void GPIO_AddPMEBitToSocket(uint32_t pme_bit, GPIO_PME_Mode_t mode, bool isep, uint32_t socket)
{
    switch(pme_bit)
    {
        case 0:
            PINT->PMSRC &= ~PINT_PMSRC_SRC0_MASK;
            PINT->PMSRC |= PINT_PMSRC_SRC0(socket);
            (isep)?(PINT->PMCFG |= PINT_PMCFG_PROD_ENDPTS0_MASK):(PINT->PMCFG &= ~PINT_PMCFG_PROD_ENDPTS0_MASK);
            PINT->PMCFG &= ~PINT_PMCFG_CFG0_MASK;
            PINT->PMCFG |= PINT_PMCFG_CFG0(mode);
            break;
        case 1:
            PINT->PMSRC &= ~PINT_PMSRC_SRC1_MASK;
            PINT->PMSRC |= PINT_PMSRC_SRC1(socket);
            (isep)?(PINT->PMCFG |= PINT_PMCFG_PROD_ENDPTS1_MASK):(PINT->PMCFG &= ~PINT_PMCFG_PROD_ENDPTS1_MASK);
            PINT->PMCFG &= ~PINT_PMCFG_CFG1_MASK;
            PINT->PMCFG |= PINT_PMCFG_CFG1(mode);
            break;
        case 2:
            PINT->PMSRC &= ~PINT_PMSRC_SRC2_MASK;
            PINT->PMSRC |= PINT_PMSRC_SRC2(socket);
            (isep)?(PINT->PMCFG |= PINT_PMCFG_PROD_ENDPTS2_MASK):(PINT->PMCFG &= ~PINT_PMCFG_PROD_ENDPTS2_MASK);
            PINT->PMCFG &= ~PINT_PMCFG_CFG2_MASK;
            PINT->PMCFG |= PINT_PMCFG_CFG2(mode);
            break;
        case 3:
            PINT->PMSRC &= ~PINT_PMSRC_SRC3_MASK;
            PINT->PMSRC |= PINT_PMSRC_SRC3(socket);
            (isep)?(PINT->PMCFG |= PINT_PMCFG_PROD_ENDPTS3_MASK):(PINT->PMCFG &= ~PINT_PMCFG_PROD_ENDPTS3_MASK);
            PINT->PMCFG &= ~PINT_PMCFG_CFG3_MASK;
            PINT->PMCFG |= PINT_PMCFG_CFG3(mode);
            break;
        case 4:
            PINT->PMSRC &= ~PINT_PMSRC_SRC4_MASK;
            PINT->PMSRC |= PINT_PMSRC_SRC4(socket);
            (isep)?(PINT->PMCFG |= PINT_PMCFG_PROD_ENDPTS4_MASK):(PINT->PMCFG &= ~PINT_PMCFG_PROD_ENDPTS4_MASK);
            PINT->PMCFG &= ~PINT_PMCFG_CFG4_MASK;
            PINT->PMCFG |= PINT_PMCFG_CFG4(mode);
            break;
        case 5:
            PINT->PMSRC &= ~PINT_PMSRC_SRC5_MASK;
            PINT->PMSRC |= PINT_PMSRC_SRC5(socket);
            (isep)?(PINT->PMCFG |= PINT_PMCFG_PROD_ENDPTS5_MASK):(PINT->PMCFG &= ~PINT_PMCFG_PROD_ENDPTS5_MASK);
            PINT->PMCFG &= ~PINT_PMCFG_CFG5_MASK;
            PINT->PMCFG |= PINT_PMCFG_CFG5(mode);
            break;
        case 6:
            PINT->PMSRC &= ~PINT_PMSRC_SRC6_MASK;
            PINT->PMSRC |= PINT_PMSRC_SRC6(socket);
            (isep)?(PINT->PMCFG |= PINT_PMCFG_PROD_ENDPTS6_MASK):(PINT->PMCFG &= ~PINT_PMCFG_PROD_ENDPTS6_MASK);
            PINT->PMCFG &= ~PINT_PMCFG_CFG6_MASK;
            PINT->PMCFG |= PINT_PMCFG_CFG6(mode);
            break;
        case 7:
            PINT->PMSRC &= ~PINT_PMSRC_SRC7_MASK;
            PINT->PMSRC |= PINT_PMSRC_SRC7(socket);
            PINT->PMCFG &= ~PINT_PMCFG_CFG7_MASK;
            PINT->PMCFG |= PINT_PMCFG_CFG7(mode);
            break;
    }
    PINT->PMSRC = PINT->PMSRC;
}

void PORT_IRQHandler(uint32_t socket)
{
    PINT->IST = (1<<socket);
    
    if(PINT->PMCTRL & PINT_PMCTRL_SEL_PMATCH_MASK)
    {
        PINT->PMSRC = PINT->PMSRC;
    }
}

//! @}
