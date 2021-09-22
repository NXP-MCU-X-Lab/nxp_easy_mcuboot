#include "common.h"
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

#define N_BYTES(n)         ((n & 0x3FF) << 16)
#define BUF_ADDR(addr)     (((addr) >> 6) & 0xFFFF)

#define EP_OUT_IDX(EPNum)  (EPNum * 2)
#define EP_IN_IDX(EPNum)   (EPNum * 2 + 1)

#define MAX_EP_COUNT        (5)
/* EP list must be 256 aligned  */
ALIGN(256)  volatile uint32_t EPList[(MAX_EP_COUNT) * 4];

/* buffer must be 64 byte aliged */
ALIGN(64)   uint8_t EP_BUF_BASE[64*(MAX_EP_COUNT*2)*2]; /* 10 physical EP, each EP has 2 buffer */

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
 *  USB Device Initialize Function
 *   Called by the User to initialize USB Device
 *    Return Value:    None
 */

void USBD_Init (void)
{
    /* select USB clock to be 48M FRO */
    #if defined(SYSCON_USB0CLKSEL_SEL_MASK)
    SYSCON->USB0CLKSEL = SYSCON_USB0CLKSEL_SEL(0);
    SYSCON->USB0CLKDIV = SYSCON_USB0CLKDIV_DIV((GetClock(kFROHfClock) == 48*1000*1000)?(0):(1));
    #else
    SYSCON->USBCLKSEL = SYSCON_USBCLKSEL_SEL(0);
    SYSCON->USBCLKDIV = SYSCON_USBCLKDIV_DIV((GetClock(kFROHfClock) == 48*1000*1000)?(0):(1));
    #endif
    
    /* enable USB clock */
    #if defined(SYSCON_AHBCLKCTRL_USB0D_MASK)
    SYSCON->AHBCLKCTRL[1] |= SYSCON_AHBCLKCTRL_USB0D_MASK;
    SYSCON->AHBCLKCTRL[2] |= SYSCON_AHBCLKCTRL_USB0HSL_MASK;
    
    SYSCON->PDRUNCFG[0] &= ~SYSCON_PDRUNCFG_PDEN_USB0_PHY_MASK;
    
    *((uint32_t *)(USBFSH_BASE + 0x5C)) |= USBFSH_PORTMODE_DEV_ENABLE_MASK;
    
    #else
    SYSCON->AHBCLKCTRL[1] |= SYSCON_AHBCLKCTRL_USB0_MASK;
    SYSCON->PDRUNCFG[0] &= ~SYSCON_PDRUNCFG_PDEN_USB_PHY_MASK;
    #endif


    USBD_TRACE("EPNUm:%d\r\n", MAX_EP_COUNT);
    USBD_TRACE("EP_BUF_BASE: 0x%08X\r\n", (uint32_t)EP_BUF_BASE);
    USBD_TRACE("EPList: 0x%08X\r\n", (uint32_t)EPList);
    
    USB0->DEVCMDSTAT |= (1UL << 9); /* PLL ON */

    USB0->DATABUFSTART = (uint32_t)EP_BUF_BASE & 0xFFC00000;
    USB0->EPLISTSTART  = (uint32_t)EPList;

    NVIC_EnableIRQ(USB0_IRQn);

    USBD_Reset();
}


/*
 *  USB Device Connect Function
 *   Called by the User to Connect/Disconnect USB Device
 *    Parameters:      con:   Connect/Disconnect
 *    Return Value:    None
 */

void USBD_Connect (bool con)
{
    if ( con )
    USB0->DEVCMDSTAT |=  (1UL << 16);/* Set device connect status          */
    else
    USB0->DEVCMDSTAT &= ~(1UL << 16);/* Clear device connect status        */
    return;
}


/*
 *  USB Device Reset Function
 *   Called automatically on USB Device Reset
 *    Return Value:    None
 */

void USBD_Reset (void) {

  uint32_t i;
  uint32_t * ptr;

  addr = 3 * 64 + (uint32_t)EP_BUF_BASE;

  for (i = 2; i < (5 * 4); i++) {
    EPList[i] = (1UL << 30);            /* EPs disabled                       */
  }

  EPBufInfo[0].buf_len = EP0_MAX_SIZE;
  EPBufInfo[0].buf_ptr = (uint32_t)EP_BUF_BASE;
  EPBufInfo[1].buf_len = EP0_MAX_SIZE;
  EPBufInfo[1].buf_ptr = (uint32_t)EP_BUF_BASE + 2 * 64;

  ptr  = GetEpCmdStatPtr(0);

  *ptr = N_BYTES(EPBufInfo[0].buf_len) |     /*EP0 OUT                        */
         BUF_ADDR(EPBufInfo[0].buf_ptr)|
         BUF_ACTIVE;
  ptr++;
  *ptr = BUF_ADDR(EPBufInfo[0].buf_ptr + 64);/* SETUP                         */


  USB0->DEVCMDSTAT |= (1UL << 7);         /* USB device enable              */

  USB0->INTSTAT     = 0x2FC;              /* clear EP interrupt flags      */

  USB0->INTEN = (                   (1UL << 31)|/* SOF intr enable    */
                                       (1UL << 0 )     |/* EP0 OUT intr enable*/
                                       (1UL << 1 )     |/* EP0 IN intr enable */
                                       (1UL << 31));     /* stat change int en */
                                       
    USBD_ConfigEP(0x01);
    USBD_EnableEP(0x01);
    USBD_ResetEP(0x01);

    USBD_ConfigEP(0x81);
    USBD_EnableEP(0x81);
    USBD_ResetEP(0x81);
    
    USBD_ConfigEP(0x02);
    USBD_EnableEP(0x02);
    USBD_ResetEP(0x02);

    USBD_ConfigEP(0x82);
    USBD_EnableEP(0x82);
    USBD_ResetEP(0x82);
    
    USBD_ConfigEP(0x03);
    USBD_EnableEP(0x03);
    USBD_ResetEP(0x03);

    USBD_ConfigEP(0x83);
    USBD_EnableEP(0x83);
    USBD_ResetEP(0x83);
    
    USBD_ConfigEP(0x04);
    USBD_EnableEP(0x04);
    USBD_ResetEP(0x04);

    USBD_ConfigEP(0x84);
    USBD_EnableEP(0x84);
    USBD_ResetEP(0x84);
}

void USBD_SetAddress (uint32_t adr, uint32_t setup) {
  if (!setup) {
    USB0->DEVCMDSTAT &= ~0x7F;
    USB0->DEVCMDSTAT |= adr |  (1UL<<7);
  }
}


/*
 *  Configure USB Device Endpoint according to Descriptor
 *    Parameters:      pEPD:  Pointer to Device Endpoint Descriptor
 *    Return Value:    None
 */

void USBD_ConfigEP (uint8_t ep)
{
    uint32_t num, val;
    uint32_t * ptr;

    num  = ep;
    val  = 64;

    /* IN EPs                                                                   */
    if (num & 0x80)
    {
        num &= ~0x80;
        EPBufInfo[EP_IN_IDX(num)].buf_len  = val;
        EPBufInfo[EP_IN_IDX(num)].buf_ptr  = addr;
        addr += ((val + 63) >> 6) * 64;     /* calc new free buffer address       */

        ptr  = GetEpCmdStatPtr(num | 0x80);
        *ptr = EP_DISABLED;
    }
    else /* OUT EPs */
    {
        EPBufInfo[EP_OUT_IDX(num)].buf_len  = val;
        EPBufInfo[EP_OUT_IDX(num)].buf_ptr  = addr;
        ptr  = GetEpCmdStatPtr(num);
        *ptr = N_BYTES(EPBufInfo[EP_OUT_IDX(num)].buf_len) | BUF_ADDR(EPBufInfo[EP_OUT_IDX(num)].buf_ptr) | EP_DISABLED;
        addr += ((val + 63) >> 6) * 64;     /* calc new free buffer address       */
    }
}



/*
 *  Enable USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_EnableEP (uint32_t EPNum)
{
    uint32_t * ptr;;

    ptr = GetEpCmdStatPtr(EPNum);

    if (EPNum & 0x80) /* IN EP */
    {
        EPNum &= ~0x80;
        *ptr &= ~EP_DISABLED;
        USB0->INTSTAT = (1 << EP_IN_IDX(EPNum));
        USB0->INTEN  |= (1 << EP_IN_IDX(EPNum));
    }
    else /* OUT EP  */
    {
        *ptr &= ~EP_DISABLED;
        *ptr |=  BUF_ACTIVE;
        USB0->INTSTAT = (1 << EP_OUT_IDX(EPNum));
        USB0->INTEN  |= (1 << EP_OUT_IDX(EPNum));
    }
}


void USBD_ResetEP (uint32_t EPNum)
{
    uint32_t *ptr;

    ptr = GetEpCmdStatPtr(EPNum);
    *ptr |= TOOGLE_RESET;
}

uint32_t USBD_ReadEP (uint32_t EPNum, uint8_t *pData)
{
    uint32_t cnt, i;
    uint32_t *dataptr, *ptr;

    ptr = GetEpCmdStatPtr(EPNum);

    /* SETUP packet */
    if ((EPNum == 0) && (USB0->DEVCMDSTAT & (1UL << 8)))
    {
        cnt =  8;   /* LPC hardware cannot tell you how much byte setup packet has been received. standard request len = 8 */
        dataptr = (uint32_t *)(EPBufInfo[EP_OUT_IDX(EPNum)].buf_ptr + 64);

        for (i = 0; i < (cnt + 3) / 4; i++)
        {
            *((__packed uint32_t *)pData) = dataptr[i];
            pData += 4;
        }

        USB0->EPSKIP |= (1 << EP_IN_IDX(EPNum));
        while (USB0->EPSKIP & (1 << EP_IN_IDX(EPNum)));

        if (*(ptr + 2) & EP_STALL)
        {
            *(ptr + 2) &= ~(EP_STALL);
        }
        if (*ptr & EP_STALL)
        {
            *ptr &= ~(EP_STALL);
        }
    
        USB0->DEVCMDSTAT |= (1UL << 8);
    }
    else /*OUT packet */
    {
        ptr = GetEpCmdStatPtr(EPNum);
        cnt = EPBufInfo[EP_OUT_IDX(EPNum)].buf_len - ((*ptr >> 16) & 0x3FF);
    
        dataptr = (uint32_t *)EPBufInfo[EP_OUT_IDX(EPNum)].buf_ptr;

        for (i = 0; i < (cnt + 3) / 4; i++)
        {
            *((__packed uint32_t *)pData) = dataptr[i];
            pData += 4;
        }
        *ptr = N_BYTES(EPBufInfo[EP_OUT_IDX(EPNum)].buf_len) | BUF_ADDR(EPBufInfo[EP_OUT_IDX(EPNum)].buf_ptr) | BUF_ACTIVE;
    }
    return (cnt);
}


uint32_t USBD_WriteEP (uint32_t EPNum, uint8_t *pData, uint32_t cnt)
{
    uint32_t i;
    uint32_t * dataptr, *ptr;

    ptr = GetEpCmdStatPtr(EPNum);

    EPNum &= ~0x80;

    while (*ptr & BUF_ACTIVE);

    *ptr &= ~(0x3FFFFFF);
    *ptr |=  BUF_ADDR(EPBufInfo[EP_IN_IDX(EPNum)].buf_ptr) | N_BYTES(cnt);
           

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

    *ptr |= BUF_ACTIVE;
    return (cnt);
}

/*
 *  USB Device Interrupt Service Routine
 */

void USBD_IRQHandler(void)
{
    uint32_t sts, val, num;
    
    sts = USB0->INTSTAT & USB0->INTEN;
    USB0->INTSTAT = sts;
    
    /* Device Status Interrupt (Reset, Connect change, Suspend/Resume) */
    if (sts & USB_INTSTAT_DEV_INT_MASK)
    {
        val = USB0->DEVCMDSTAT;

        /* reset interrupt */
        if (val & USB_DEVCMDSTAT_DRES_C_MASK)
        {
            USBD_TRACE("usbd reset\r\n");
            USBD_Reset();
            usbd_stack_reset();
            USB0->DEVCMDSTAT |= USB_DEVCMDSTAT_DRES_C_MASK;
        }

        /* suspend/resume interrupt */
        if (val & (1UL << 25))
        {
            USBD_TRACE("suspend/resume\r\n");
            /* suspend interrupt */
            if (val & (1UL << 17))
            {
               // USBD_Suspend();
            }
            /* resume interrupt */
            else
            {
            
            }
            USB0->DEVCMDSTAT |= (1UL << 25);
        }

        /* connect interrupt                                                      */
        if (val & (1UL << 24))
        {
            USB0->DEVCMDSTAT |= (1UL << 24);
        }
        USB0->INTSTAT = USB_INTSTAT_DEV_INT_MASK;
    }

    /* Start of Frame                                                           */
    if (sts & USB_INTSTAT_FRAME_INT_MASK)
    {
        USB0->INTSTAT = USB_INTSTAT_FRAME_INT_MASK;
    }

    /* EndPoint Interrupt */
    if (sts & 0x3FF)
    {
        
        val = USB0->DEVCMDSTAT;
        if (sts & USB_INTSTAT_EP0OUT_MASK) 
        {
            if(val & USB_DEVCMDSTAT_SETUP_MASK)  /* SETUP */
            {
                usbd_ep0_setup_handler();
            }
            else
            {
                usbd_ep0_out_handler();
            }
        }
        else if (sts & USB_INTSTAT_EP0IN_MASK) /* EP0 in token */
        {
            usbd_ep0_in_handler();
        }
        else /* other ep */
        {
            for(num = 2; num < (MAX_EP_COUNT*2); num++)
            {
                 if (sts & (1UL << num))
                 {
                   // printf("EP%d %s\r\n", num / 2, (num % 2)?("IN"):("OUT"));
                    #if defined(RTT)
                        msd_msg_t msg;
                        msg.param = num;
                        msg.exec = NULL;
                        rt_mq_send(msd_mq, &msg, sizeof(msg));
                    #else
                        usbd_data_ep_handler(num / 2, num % 2);
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
    USBD_SetAddress(addr, 0);
    return 0;
}

uint32_t usbd_config_ep(struct uendpoint_descriptor* d)
{
    uint8_t num =  d->bEndpointAddress;
    uint32_t size =  d->wMaxPacketSize;
    uint8_t type = d->bmAttributes;
    //USBD_TRACE("ConfigEP:0x%X %d %d\r\n", num, size, type);
    USBD_ConfigEP(num);
    USBD_EnableEP(num);
    USBD_ResetEP(num);
    return 0;
}

uint32_t ep_clear_feature(uint8_t ep)
{
    uint32_t *ptr;

    ptr = GetEpCmdStatPtr(ep);
    printf("ep_clear_feature:0x%X 0x%08X 0x%08X\r\n", ep, USB0->INFO >> 11, *ptr);
    if (ep & 0x80)
    {
        *ptr &=  ~EP_STALL;
    }
    else
    {
        *ptr &=  ~EP_STALL;
        *ptr |=   BUF_ACTIVE;
    }
    USBD_ResetEP(ep);
    return CH_OK;
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

void USB_IRQHandler (void)
{
    USBD_IRQHandler();
}

void USB0_IRQHandler (void)
{
    USBD_IRQHandler();
}
