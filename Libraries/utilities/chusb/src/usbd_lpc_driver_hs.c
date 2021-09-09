#include "common.h"
#include "syscon.h"
#include "usbd.h"
#include "usbd_config.h"

void USBD_Reset (void);
void USBD_EnableEP (uint32_t EPNum);
void USBD_ResetEP (uint32_t EPNum);


#if defined(RTT)
#include <rtthread.h>
extern rt_mq_t msd_mq;
    
typedef struct
{
    uint8_t param;
    void (*exec)(uint32_t param);
}msd_msg_t;

#endif

#define BUF_ACTIVE         (1UL << 31)
#define EP_DISABLED        (1UL << 30)
#define EP_STALL           (1UL << 29)
#define TOOGLE_RESET       (1UL << 28)
#define EP_TYPE            (1UL << 26)
#define EP_RF_TV           (1UL << 27)

#define N_BYTES(n)         ((n & 0x7FFF) << 11)
#define BUF_ADDR(addr)     (((addr) >> 6) & 0x7FF)

#define EP_OUT_IDX(EPNum)  (EPNum * 2)
#define EP_IN_IDX(EPNum)   (EPNum * 2 + 1)

#define MAX_EP_COUNT        (6)
#define EP_BUFFER_SIZE     (MSD_EP_SIZE)

/* EP list must be 256 aligned */
uint32_t *EPList = (uint32_t*)0x40101F00;

/* buffer must be 64 byte aliged */
uint8_t *EP_BUF_BASE = (uint8_t*)0x40100000;

typedef struct BUF_INFO
{
  uint32_t  buf_len;
  uint32_t  buf_ptr;
}EP_BUF_INFO;

EP_BUF_INFO EPBufInfo[(MAX_EP_COUNT) * 2];

static uint32_t addr;
void USBD_ConfigEP (uint8_t ep);

/*get EP Command/Status register */
uint32_t * GetEpCmdStatPtr (uint32_t EPNum)
{
    uint32_t ptr = 0;

    if (EPNum & 0x80)
    {
        EPNum &= ~0x80;
        ptr = 8;
    }
    
    ptr += (uint32_t)EPList + EPNum * sizeof(uint32_t)*4;
    return ((uint32_t *)ptr);
}

/*
// Enable source power for USB PHYs
void POWER_SetUsbPhy(void)
{
  SYSCON->PDRUNCFGCLR[0] = PDRUNCFG_PD_VD5;
  // wait until VD5 fully power
  while((POWER->MREG_VDOK & POWER_VD5_OK_LOAD) != POWER_VD5_OK_LOAD );

} 
*/

void USBD_Init (void)
{
    int i;
    
    SetupUSBPLL();
        
    /* select USB clock to be usb pll */
    SYSCON->USB1CLKSEL = SYSCON_USB1CLKSEL_SEL(2);
    for(i=1; i<0xFF; i++)
    {
        if((GetClock(kUSBPLLClock) / i) == 48*1000*1000)
        {
            USBD_TRACE("USB1D divider found:%d\r\n", i);
            SYSCON->USB1CLKDIV = SYSCON_USB1CLKDIV_DIV(i-1);
            break;
        }
    }
    
    if(i == 0xFF)
    {
        USBD_TRACE("cannot found divider\r\n");
    }
    
    SYSCON->PDRUNCFG[0] &= ~SYSCON_PDRUNCFG_PDEN_VD5_MASK;
    /* wait until VD5 fully power */
    DelayMs(2);
    
    /* Enable USB1D and USB1RAM */
    SYSCON->AHBCLKCTRL[2] |= SYSCON_AHBCLKCTRL_USB1H_MASK | SYSCON_AHBCLKCTRL_USB1D_MASK | SYSCON_AHBCLKCTRL_USB1RAM_MASK;
    SYSCON->PDRUNCFG[0] &= ~(SYSCON_PDRUNCFG_PDEN_VD2_ANA_MASK | SYSCON_PDRUNCFG_PDEN_USB_RAM_MASK | SYSCON_PDRUNCFG_PDEN_VD3_MASK | SYSCON_PDRUNCFG_PDEN_VD5_MASK);
    SYSCON->PDRUNCFG[1] &= ~(SYSCON_PDRUNCFGSET_PDEN_USB1_PHY_MASK);
    
    *((uint32_t *)(USBHSH_BASE + 0x50)) |= USBHSH_PORTMODE_DEV_ENABLE_MASK;
    
    USBD_TRACE("EPNUm:%d\r\n", MAX_EP_COUNT);
    USBD_TRACE("EP_BUF_BASE: 0x%08X\r\n", (uint32_t)EP_BUF_BASE);
    USBD_TRACE("EPList: 0x%08X\r\n", (uint32_t)EPList);

    USBHSD->DEVCMDSTAT &= ~USBHSD_DEVCMDSTAT_DCON_MASK;
    USBHSD->DEVCMDSTAT |= USBHSD_DEVCMDSTAT_FORCE_NEEDCLK_MASK;

    USBHSD->EPLISTSTART = (uint32_t)EPList;
    
    USBD_TRACE("EPLISTSTART:0x%08X\r\n", USBHSD->EPLISTSTART);
    USBD_TRACE("DATABUFSTART:0x%08X\r\n", USBHSD->DATABUFSTART);

    USBD_Reset();
    NVIC_EnableIRQ(USB1_IRQn);
}


void USBD_Connect (bool con)
{
    if(con)
    {
        USBHSD->DEVCMDSTAT |= USBHSD_DEVCMDSTAT_DCON_MASK;
    }
    else
    {
        USBHSD->DEVCMDSTAT &= ~USBHSD_DEVCMDSTAT_DCON_MASK;
    }
}


void USBD_Reset (void)
{
    uint32_t * ptr;

    addr = 3 * EP_BUFFER_SIZE + (uint32_t)EP_BUF_BASE;

    EPBufInfo[0].buf_len = EP0_MAX_SIZE;
    EPBufInfo[0].buf_ptr = (uint32_t)EP_BUF_BASE;
    EPBufInfo[1].buf_len = EP0_MAX_SIZE;
    EPBufInfo[1].buf_ptr = (uint32_t)EP_BUF_BASE + 2 * EP_BUFFER_SIZE;

    ptr  = GetEpCmdStatPtr(0);

    /* EP0 OUT */
    *ptr = N_BYTES(EPBufInfo[0].buf_len) | BUF_ADDR(EPBufInfo[0].buf_ptr) | BUF_ACTIVE;
    ptr++;
    *ptr = BUF_ADDR(EPBufInfo[0].buf_ptr + EP_BUFFER_SIZE);     /* SETUP */

    USBHSD->DEVCMDSTAT &= ~USBHSD_DEVCMDSTAT_DEV_ADDR_MASK;
    USBHSD->DEVCMDSTAT |= USBHSD_DEVCMDSTAT_DEV_EN_MASK; /* USB device enable */

    USBHSD->INTSTAT = 0xFFFFFFFF;
    USBHSD->INTEN = USBHSD_INTEN_EP_INT_EN_MASK | USBHSD_INTEN_FRAME_INT_EN_MASK | USBHSD_INTEN_DEV_INT_EN_MASK;
                 
}

uint32_t usbd_config_ep(struct uendpoint_descriptor* d)
{
    uint32_t * ptr;

    uint8_t num =  d->bEndpointAddress;
    uint32_t size =  d->wMaxPacketSize;
    volatile uint8_t type = d->bmAttributes;
    USBD_TRACE("ConfigEP:0x%X %d %d\r\n", num, size, type);
    
    if (num & 0x80) /* IN EPs */
    {
        num &= ~0x80;
        EPBufInfo[EP_IN_IDX(num)].buf_len  = size;
        EPBufInfo[EP_IN_IDX(num)].buf_ptr  = addr;
        addr += (((size + (EP_BUFFER_SIZE-1)) >> 6) * EP_BUFFER_SIZE);     /* calc new free buffer address       */

        ptr  = GetEpCmdStatPtr(num | 0x80);
        *ptr = EP_DISABLED;
        *ptr &= ~EP_DISABLED;
        *ptr |= TOOGLE_RESET;
    }
    else /* OUT */
    {
        EPBufInfo[EP_OUT_IDX(num)].buf_len  = size;
        EPBufInfo[EP_OUT_IDX(num)].buf_ptr  = addr;
        ptr  = GetEpCmdStatPtr(num);
        *ptr = N_BYTES(EPBufInfo[EP_OUT_IDX(num)].buf_len) | BUF_ADDR(EPBufInfo[EP_OUT_IDX(num)].buf_ptr);
        *ptr |=  BUF_ACTIVE;
        *ptr |=  TOOGLE_RESET;
        addr += ((size + (EP_BUFFER_SIZE-1)) >> 6) * EP_BUFFER_SIZE;     /* calc new free buffer address       */
    }
    return 0;
}

uint32_t USBD_ReadEP (uint32_t EPNum, uint8_t *pData)
{
    uint32_t cnt, i;
    uint32_t *dataptr, *ptr;
    ptr = GetEpCmdStatPtr(EPNum);

    /* SETUP packet */
    if ((EPNum == 0) && (USBHSD->DEVCMDSTAT & USBHSD_DEVCMDSTAT_SETUP_MASK))
    {
        cnt =  8;   /* LPC hardware cannot tell you how much byte setup packet has been received. standard request len = 8 */
        dataptr = (uint32_t *)(EPBufInfo[EP_OUT_IDX(EPNum)].buf_ptr + EP_BUFFER_SIZE);

        for (i = 0; i < (cnt + 3) / 4; i++)
        {
            *((__packed uint32_t *)pData) = dataptr[i];
            pData += 4;
        }

        USBHSD->EPSKIP |= (1 << EP_IN_IDX(EPNum));
        while (USBHSD->EPSKIP & (1 << EP_IN_IDX(EPNum)));

        if (*(ptr + 2) & EP_STALL)
        {
            *(ptr + 2) &= ~(EP_STALL);
        }
        if (*ptr & EP_STALL)
        {
            *ptr &= ~(EP_STALL);
        }
    
        USBHSD->DEVCMDSTAT |= USBHSD_DEVCMDSTAT_SETUP_MASK;
    }
    else /*OUT packet */
    {
        ptr = GetEpCmdStatPtr(EPNum);
        cnt = EPBufInfo[EP_OUT_IDX(EPNum)].buf_len - ((*ptr >> 11) & 0x7FFF);

        dataptr = (uint32_t *)EPBufInfo[EP_OUT_IDX(EPNum)].buf_ptr;

        for (i = 0; i < (cnt + 3) / 4; i++)
        {
            *((__packed uint32_t *)pData) = dataptr[i];
            pData += 4;
        }
        *ptr = N_BYTES(EPBufInfo[EP_OUT_IDX(EPNum)].buf_len) | BUF_ADDR(EPBufInfo[EP_OUT_IDX(EPNum)].buf_ptr) | BUF_ACTIVE |  EP_TYPE | EP_RF_TV;;
    }
    return (cnt);
}


uint32_t USBD_WriteEP (uint32_t EPNum, uint8_t *pData, uint32_t cnt)
{
    uint32_t i;
    uint32_t * dataptr, *ptr;

    ptr = GetEpCmdStatPtr(EPNum);

    EPNum &= ~0x80;

    if(*ptr & BUF_ACTIVE)
    {
        //USBD_TRACE("USBD_WriteEP error\r\n");
        return 0;
    }

    *ptr &= ~(0x03FFFFFF);
    *ptr |=  BUF_ADDR(EPBufInfo[EP_IN_IDX(EPNum)].buf_ptr) | N_BYTES(cnt);
    //USBD_TRACE("WriteEP%d len:%d 0x%08X\r\n", EPNum, cnt, EPBufInfo[EP_IN_IDX(EPNum)].buf_ptr);

    dataptr = (uint32_t *)EPBufInfo[EP_IN_IDX(EPNum)].buf_ptr;
    for (i = 0; i < (cnt + 3) / 4; i++)
    {
        dataptr[i] = * ((__packed uint32_t *)pData);
        pData += 4;
    }

    if (EPNum && (*ptr & EP_STALL))
    {
        return (0);
    }
    
    if((EPNum & 0x0F) == 0)
    {
		/* When EP0 IN is received, set ACTIVE bit on both EP0 IN and OUT. */
		*(ptr - 2) |= BUF_ACTIVE;	/* Set ACTIVE bit on EP0 OUT */
    }
    
    *ptr |= BUF_ACTIVE;
    return (cnt);
}

/*
 *  USB Device Interrupt Service Routine
 */

void USBD_IRQHandler(void)
{
    uint32_t sts, val, num;
    
    sts = USBHSD->INTSTAT;
    USBHSD->INTSTAT = sts;
    
    sts &= USBHSD->INTEN;
    

	/* Save USB info register. Used later for NAK and Error handling */
	uint32_t err = (USBHSD->INFO >> 11) & 0x0F;
    
    /* Device Status Interrupt (Reset, Connect change, Suspend/Resume) */
    if (sts & USBHSD_INTSTAT_DEV_INT_MASK)
    {
        val = USBHSD->DEVCMDSTAT;

        if(val & USBHSD_DEVCMDSTAT_DCON_C_MASK)
        {
            val |= USBHSD_DEVCMDSTAT_DCON_C_MASK;
            USBD_TRACE("detach\r\n");
        }
        
        /* reset interrupt */
        if (val & USBHSD_DEVCMDSTAT_DRES_C_MASK)
        {
            USBD_TRACE("usbd reset %d\r\n", ((val & USBHSD_DEVCMDSTAT_Speed_MASK) >> USBHSD_DEVCMDSTAT_Speed_SHIFT));
            
            USBD_Reset();
            usbd_stack_reset();
            USBHSD->DEVCMDSTAT |= USBHSD_DEVCMDSTAT_DRES_C_MASK;
        }

        /* suspend/resume interrupt */
        if (val & (1UL << 25))
        {
            //USBD_TRACE("suspend/resume\r\n");
            /* suspend interrupt */
            if (val & (1UL << 17))
            {
               // USBD_Suspend();
            }
            /* resume interrupt */
            else
            {
            
            }
            USBHSD->DEVCMDSTAT |= (1UL << 25);
        }

        /* connect interrupt                                                      */
        if (val & (1UL << 24))
        {
            USBHSD->DEVCMDSTAT |= (1UL << 24);
        }
        USBHSD->INTSTAT = USBHSD_INTSTAT_DEV_INT_MASK;
    }

    /* Start of Frame                                                           */
    if (sts & USBHSD_INTSTAT_FRAME_INT_MASK)
    {
        USBHSD->INTSTAT = USBHSD_INTSTAT_FRAME_INT_MASK;
    }

    /* EndPoint Interrupt */
    if (sts & 0x3FF)
    {
        val = USBHSD->DEVCMDSTAT;
        if (sts & USBHSD_INTSTAT_EP0OUT_MASK) 
        {
            if(val & USBHSD_DEVCMDSTAT_SETUP_MASK)  /* SETUP */
            {
                usbd_ep0_setup_handler();
            }
            else
            {
                usbd_ep0_out_handler();
            }
        }
        
        if (sts & USBHSD_INTSTAT_EP0IN_MASK) /* EP0 in token */
        {
            usbd_ep0_in_handler();
        }
        else /* other ep */
        {
            for(num = 2; num < (MAX_EP_COUNT*2); num++)
            {
                 if (sts & (1UL << num))
                 {
                    #if defined(RTT)
                        msd_msg_t msg;
                        msg.param = num;
                        msg.exec = NULL;
                        rt_mq_send(msd_mq, &msg, sizeof(msg));
                    #else
                        usbd_data_ep_handler(num/2, num % 2);
                    #endif
                 }
            }
        }
    }
}

uint32_t ep_read(uint8_t ep, uint8_t *buf, uint8_t data01)
{
    return USBD_ReadEP(ep, buf);
}

uint32_t ep_write(uint8_t ep, uint8_t *buf, uint32_t len, uint8_t data01)
{
    ep |= 0x80; /* write ep must use IN EP */
    return USBD_WriteEP (ep, buf, len);
}

uint32_t set_addr(uint8_t addr)
{
    USBHSD->DEVCMDSTAT |= USBHSD_DEVCMDSTAT_DEV_ADDR(addr);
    return 0;
}

uint32_t ep_clear_feature(uint8_t ep)
{
    USBD_TRACE("ep clear features\r\n");
    return 0;
}

uint32_t detach(void)
{
    USBD_TRACE("detach\r\n");
    return 0;
}

const struct usbd_ops_t ops;

void ch_usb_init(struct usbd_t *h)
{
    static struct usbd_ops_t ops;
    ops.ep_read = ep_read;
    ops.ep_write = ep_write;
    ops.set_addr = set_addr;
    ops.ep_config = usbd_config_ep;
    ops.ep_clear_feature = ep_clear_feature;
    usbd_init(h, &ops);
}

void USB1_IRQHandler (void)
{
    USBD_IRQHandler();
}
