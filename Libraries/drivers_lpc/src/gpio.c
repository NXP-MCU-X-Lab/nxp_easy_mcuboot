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
 * @brief  �������ŵ������������
 * @note   ���������������������
 * @param  instance:
 *         @arg HW_GPIOx : GPIO�˿ں�A-E
 * @param  pin : ���ű�ţ�0-31
 * @param  dir : ����ѡ�����
 *         @arg 0 : ����
 *         @arg 1 : ���
 * @retval None
 */
void GPIO_SetPinDir(uint32_t instance, uint32_t pin, uint32_t dir)
{
    
    (dir == 1) ? (GPIO->DIR[instance] |= (1 << pin)):
    (GPIO->DIR[instance] &= ~(1 << pin));
}


 /**
 * @brief  ��ʼ������ָ�����ŵĹ���״̬
 * @note   ��ɳ�ʼ��һ����������
 * @param  instance:
 *         @arg HW_GPIOx : GPIO�˿ں�A-E
 * @param  pin : ���ź��룺0-31
 * @param  mode : ���Ų���ģʽ
 *         @arg kGPIO_IFT : ��������
 *         @arg kGPIO_IPD : ��������
 *         @arg kGPIO_IPU : ��������
 *         @arg kGPIO_OOD : ��©���
 *         @arg kGPIO_OPP : �������
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
 * @brief  ָ���������״̬
 * @note   ����ָ���������Ϊ0��1
 * @param  instance:
 *         @arg HW_GPIOx : GPIO�˿ں�A-E
 * @param  pin : ���ź��룺0-31
 * @param  data : �������
 *         @arg 0 : ���0
 *         @arg 1 : ���1
 * @retval None
 */
void GPIO_PinWrite(uint32_t instance, uint32_t pin, uint8_t data)
{
    (data) ? (GPIO->SET[instance] = (1 << pin)):
    (GPIO->CLR[instance] = (1 << pin));
}

 /**
 * @brief  ��ȡָ�����ŵ�״̬
 * @note   ��
 * @param  instance:
 *         @arg HW_GPIOx : GPIO�˿ں�A-E
 * @param  pin : ���ź��룺0-31
 * @retval ��������״̬0��1
 */
uint32_t GPIO_PinRead(uint32_t instance, uint32_t pin)
{
    return ((GPIO->PIN[instance] >> pin) & 0x01);
}

 /**
 * @brief  ��ת���ŵ�״̬
 * @note   None
 * @param  instance:
 *         @arg HW_GPIOx : GPIO�˿ں�x
 * @param  pin : ���ź��룺0-31
 * @retval None
 */
void GPIO_PinToggle(uint32_t instance, uint8_t pin)
{
    GPIO->NOT[instance] = (1 << pin);
}

 /**
 * @brief  ��ȡ���Ŷ˿ڵ�״̬
 * @note   None
 * @param  instance:
 *         @arg HW_GPIOx : GPIO�˿ں�A-E
 * @retval ����һ��32λ�˿ڵ�����
 */
uint32_t GPIO_ReadPort(uint32_t instance)
{
    return GPIO->PIN[instance];
}

 /**
 * @brief  �޸Ķ˿ڵ����״̬
 * @note   None
 * @param  instance:
 *         @arg HW_GPIOx : GPIO�˿ں�A-E
 * @param  data : 32λ������
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
