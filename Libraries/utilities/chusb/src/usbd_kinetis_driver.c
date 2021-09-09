#include "common.h"
#include "usbd.h"

#define SETUP_TOKEN    0x0D
#define IN_TOKEN       0x09
#define OUT_TOKEN      0x01
#define TOK_PID(idx)   ((BD[idx].stat >> 2) & 0x0F)

typedef struct
{
    uint8_t     stat;          /* see USB_BD_Stat_t */
    uint8_t     reserved;      
    uint16_t    bc;            /* byte count */
    uint32_t    buf_addr;      /* buffer address */
} USB_BD_t;


/* Change logical EP idx to phyiscal EP inx */
#define IN    1  /* USB device(MCU) is TX */
#define OUT   0  /* USB device(MCU) is RX */
#define ODD   0
#define EVEN  1
#define IDX(Ep, dir, Ev_Odd) ((((Ep & 0x0F) * 4) + (2 * dir) + (1 *  Ev_Odd)))

#define BD_OWN_MASK        0x80
#define BD_DATA01_MASK     0x40
#define BD_KEEP_MASK       0x20
#define BD_DTS_MASK        0x08
#define BD_STALL_MASK      0x04

static uint32_t Data1 = 0x55555555;

#define MAX_EP_COUNT        (8)

ALIGN(4) uint8_t EPBuf[MAX_EP_COUNT*4][64]; /* each EP has Tx and Rx 2EP, and each EP has ODD and EVEN 2 buf */

/* alloc BDT */
ALIGN(512) USB_BD_t BD[MAX_EP_COUNT*2*2]; /* each EP has Tx and Rx 2 EP */

uint32_t USBD_EPRead(uint8_t ep, uint8_t *buf, uint8_t data01)
{
  uint32_t n, sz, setup = 0;

  ep = IDX(ep, OUT, 0);
  sz  = BD[ep].bc;
   // printf("R:DATA%d\r\n", data01);
    
    if ((ep == 0) && (TOK_PID(ep) == SETUP_TOKEN))
    {
        setup = 1;
    }

    for (n = 0; n < sz; n++)
    {
        buf[n] = EPBuf[ep][n];
    }

    BD[ep].bc = 64;

    /* data out stage */
    if (setup && (buf[6] != 0) && ((buf[0] & 0x80) == 0x00))
    {
        data01 = 1;
    }
    
    if(data01)
    {
        //printf("\r\nR DATA1 %d\r\n", ep);
        BD[ep].stat  = BD_DTS_MASK | BD_DATA01_MASK;
    }
    else
    {
        //printf("\r\nR DATA0 %d\r\n", ep);
        BD[ep].stat  = BD_DTS_MASK;
    }
  
  BD[ep].stat |= BD_OWN_MASK;
  
  return sz;
}

void USBD_EPWrite(uint8_t ep, uint8_t *buf, uint8_t len, uint8_t data01)
{
    uint32_t n;
    uint8_t *p;
    ep &=0x0F;

   
    
    ep = IDX(ep, IN, ODD);
    p = (uint8_t *)BD[ep].buf_addr;
    
    n = len;
    BD[ep].bc = len;
    while(n--)
    {
        *p++ = *buf++;
    }
    
//    if ((Data1 >> (ep / 2)) & 1)
//    {
//        printf("W DATA0 %d\r\n", ep);
//        BD[ep].stat = BD_OWN_MASK | BD_DTS_MASK;
//    }
//    else
//    {
//        printf("W DATA1 %d\r\n", ep);
//        BD[ep].stat = BD_OWN_MASK | BD_DTS_MASK | BD_DATA01_MASK;
//    }
    
    if (data01)
    {
        //printf("W DATA1 %d\r\n", ep);
        BD[ep].stat = BD_OWN_MASK | BD_DTS_MASK | BD_DATA01_MASK;
    }
    else
    {
        //printf("W DATA0 %d\r\n", ep);
        BD[ep].stat = BD_OWN_MASK | BD_DTS_MASK;
    }
    Data1 ^= (1 << (ep / 2));
}


void USBD_EnableAllEP(void)
{
    int i;
    
    for(i=1; i<ARRAY_SIZE(USB0->ENDPOINT); i++)
    {
        USB0->ENDPOINT[i].ENDPT |= (USB_ENDPT_EPHSHK_MASK | USB_ENDPT_EPTXEN_MASK | USB_ENDPT_EPRXEN_MASK);
        BD[IDX(i, IN, ODD)].stat= BD_OWN_MASK | BD_DTS_MASK;
        BD[IDX(i, IN, ODD)].bc = 0x00;
        BD[IDX(i, IN, ODD)].buf_addr =(uint32_t  )EPBuf[IDX(i, IN, ODD)];            

        BD[IDX(i, OUT, ODD)].stat= BD_OWN_MASK | BD_DTS_MASK;;
        BD[IDX(i, OUT, ODD)].bc = 64;
        BD[IDX(i, OUT, ODD)].buf_addr =(uint32_t)EPBuf[IDX(i, OUT, ODD)];   
    }
}

void USB_Reset(void)
{
    int i;
    for(i=1; i<ARRAY_SIZE(USB0->ENDPOINT); i++)
    {
        USB0->ENDPOINT[i].ENDPT = 0x00;
    }

    BD[IDX(0, OUT, ODD)].bc = 64;
    BD[IDX(0, OUT, ODD)].buf_addr =(uint32_t)EPBuf[IDX(0, OUT, ODD)];
    BD[IDX(0, OUT, ODD)].stat = BD_OWN_MASK | BD_DTS_MASK | BD_DATA01_MASK;   

    BD[IDX(0, OUT, EVEN)].bc = 64;
    BD[IDX(0, OUT, EVEN)].buf_addr =(uint32_t)EPBuf[IDX(0, OUT, EVEN)];
    BD[IDX(0, OUT, EVEN)].stat = BD_OWN_MASK | BD_DTS_MASK | BD_DATA01_MASK;     
   
    BD[IDX(0, IN, ODD)].bc = 0;
    BD[IDX(0, IN, ODD)].buf_addr =(uint32_t)EPBuf[IDX(0, IN, ODD)];      
    BD[IDX(0, IN, ODD)].stat = BD_OWN_MASK | BD_DTS_MASK;
   
    BD[IDX(0, IN, EVEN)].bc = 0; 
    BD[IDX(0, IN, EVEN)].buf_addr =(uint32_t)EPBuf[IDX(0, IN, EVEN)];      
    BD[IDX(0, IN, EVEN)].stat = BD_OWN_MASK | BD_DTS_MASK;

    /* reset all BDT to even */
    USB0->CTL |= USB_CTL_ODDRST_MASK;

    USB0->ENDPOINT[0].ENDPT = 0x0D;
    
    /* enable and clear all error states */
    USB0->ERRSTAT = 0xFF;
    USB0->ERREN = 0xFF;
    USB0->ADDR = 0x00;
    USB0->ISTAT = 0xFF;
    USB0->INTEN = USB_INTEN_TOKDNEEN_MASK | USB_INTEN_USBRSTEN_MASK;


    Data1 = 0x55555555;
    USBD_EnableAllEP();
}

void USB_SetAddr(uint8_t addr)
{
    USB0->ADDR = addr;
}


static int USB_SetClockDiv(uint32_t srcClock)
{
    uint8_t frac,div;
#ifdef SIM_CLKDIV2_USBDIV
    /* clear all divivder */
    SIM->CLKDIV2 &= ~SIM_CLKDIV2_USBDIV_MASK;
    SIM->CLKDIV2 &= ~SIM_CLKDIV2_USBFRAC_MASK;
    
    for(frac = 0; frac < 2; frac++)
    {
        for(div = 0; div < 8; div++)
        {
            if(((srcClock*(frac+1))/(div+1)) > 47000000 && ((srcClock*(frac+1))/(div+1)) < 49000000)
            {
                SIM->CLKDIV2 |= SIM_CLKDIV2_USBDIV(div);
                (frac)?(SIM->CLKDIV2 |= SIM_CLKDIV2_USBFRAC_MASK):(SIM->CLKDIV2 &= ~SIM_CLKDIV2_USBFRAC_MASK);
                LIB_TRACE("USB clock OK src:%d frac:%d div:%d 0x%08X\r\n", srcClock, frac, div, SIM->CLKDIV2);
                return 0;
            }
        }
    }
#else
    return 0;
#endif
    return 1;
}

uint8_t USB_ClockInit(void)
{
    /* open clock gate */
#if defined(SIM_SOPT2_USBSRC_MASK)
    SIM->SOPT2 |= SIM_SOPT2_USBSRC_MASK;
#elif defined(PCC0_PCC_USB0FS_CGC_MASK)
    SCG->FIRCDIV &= ~SCG_FIRCDIV_FIRCDIV1_MASK;
    SCG->FIRCDIV |= SCG_FIRCDIV_FIRCDIV1(1);
    
    PCC0->PCC_USB0FS &= ~PCC0_PCC_USB0FS_CGC_MASK;
    PCC0->PCC_USB0FS |= PCC0_PCC_USB0FS_PCS(3);
    PCC0->PCC_USB0FS |= PCC0_PCC_USB0FS_CGC_MASK;
    
#else
    #error "No USB clock gate"
#endif
    
    /* clock config */
    uint32_t clock;
    clock = GetClock(kMCGOutClock);
    LIB_TRACE("USB clock input:%dHz\r\n", clock);
    if(USB_SetClockDiv(clock))
    {
        LIB_TRACE("USB clock setup fail\r\n");
        return 1;
    }

    /* which MCG generator is to be used */
#if defined(SIM_SOPT2_PLLFLLSEL_MASK)
        SIM->SOPT2 &= ~SIM_SOPT2_PLLFLLSEL_MASK;
    #ifdef SIM_SOPT2_PLLFLLSEL
        (MCG->C6 & MCG_C6_PLLS_MASK)?
        (SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL(1)):   /* PLL */
        (SIM->SOPT2 &= ~SIM_SOPT2_PLLFLLSEL(0));  /* FLL */
    #else
        (MCG->C6 & MCG_C6_PLLS_MASK)?
        (SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK):   /* PLL */
        (SIM->SOPT2 &= ~SIM_SOPT2_PLLFLLSEL_MASK);  /* FLL */
    #endif
#endif
    
#if defined(SIM_SOPT2_USBSRC_MASK)
    SIM->SOPT2 |= SIM_SOPT2_USBSRC_MASK;
#endif
    
	return 0;
}

uint8_t USBD_Init(void)
{
    USB_ClockInit();
    
    /* Enable USBOTG clock        */
    #if defined(SIM_SCGC4_USBOTG_MASK)
    SIM->SCGC4   |=   SIM_SCGC4_USBOTG_MASK;  
    #elif defined(SIM_SCGC4_USBFS_MASK)
    SIM->SCGC4   |=   SIM_SCGC4_USBFS_MASK;  
    #endif
    
    /* open clock gate */
#if defined(SIM_SOPT2_USBSRC_MASK)
    SIM->SOPT2 |= SIM_SOPT2_USBSRC_MASK;
#elif defined(PCC0_PCC_USB0FS_CGC_MASK)
    SCG->FIRCDIV &= ~SCG_FIRCDIV_FIRCDIV1_MASK;
    SCG->FIRCDIV |= SCG_FIRCDIV_FIRCDIV1(1);
    
    PCC0->PCC_USB0FS &= ~PCC0_PCC_USB0FS_CGC_MASK;
    PCC0->PCC_USB0FS |= PCC0_PCC_USB0FS_PCS(3);
    PCC0->PCC_USB0FS |= PCC0_PCC_USB0FS_CGC_MASK;
    
#else
    #error "No USB clock gate"
#endif
    /* enable USB reguator */
	SIM->SOPT1 |= SIM_SOPT1_USBREGEN_MASK;
    
    /* disable memory protection */
    #ifdef MPU
    MPU->CESR=0;
    #endif
    
    /* reset USB moudle */
	USB0->USBTRC0 |= USB_USBTRC0_USBRESET_MASK;
	while(USB0->USBTRC0 & USB_USBTRC0_USBRESET_MASK){};

    /* BDT adddress */
	USB0->BDTPAGE1=(uint8_t)((uint32_t)BD>>8);
	USB0->BDTPAGE2=(uint8_t)((uint32_t)BD>>16);
	USB0->BDTPAGE3=(uint8_t)((uint32_t)BD>>24);

    /* clear all IT bit */
    USB0->ISTAT = 0xFF;
    
    /* enable USB reset IT */
    USB0->INTEN |= USB_INTEN_USBRSTEN_MASK;

	USB0->USBCTRL = USB_USBCTRL_PDE_MASK;
	USB0->USBTRC0 |= 0x40;

    /* enable USB moudle */
	USB0->CTL |= USB_CTL_USBENSOFEN_MASK;
            
    /* enable pull down reisger */
	USB0->CONTROL |= USB_CONTROL_DPPULLUPNONOTG_MASK;
    
    NVIC_EnableIRQ(USB0_IRQn);
	return CH_OK;
}

void USB_Token_Handle(uint8_t stat)
{
    uint8_t num;
    uint8_t dir;
    uint8_t ev_odd;
    
    /* get endpoint direction and ep number */
    (stat & USB_STAT_TX_MASK)?(dir = IN):(dir = OUT);
    num = ((stat & USB_STAT_ENDP_MASK )>> USB_STAT_ENDP_SHIFT);
    ev_odd = ((stat & USB_STAT_ODD_MASK) >> USB_STAT_ODD_SHIFT);
    
    uint8_t pid;
    
    pid = TOK_PID(IDX(num, dir, ev_odd));
    switch(pid)
    {
        case SETUP_TOKEN:
            Data1 &= ~0x02;
            //USBD_TRACE("SETUP: EP%d DIR:%d EV_ODD:%d - ", num, dir, ev_odd);
            break;
        case OUT_TOKEN:
           // USBD_TRACE("OUT: EP%d DIR:%d EV_ODD:%d\r\n", num, dir, ev_odd);
            break;
        case IN_TOKEN:
            //USBD_TRACE("IN: EP%d DIR:%d EV_ODD:%d\r\n", num, dir, ev_odd);
            break;
        default:
            printf("Unknown PID:%d\r\n", pid);
            break;
    }
    
    /* EP0 Handling */
	if(num == 0)
	{
        if(pid == SETUP_TOKEN)
        {
            usbd_ep0_setup_handler();
            USB0->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;
        }
        
        /* 如果是 数据为0的OUT包 比如 SETUP过程中的状态阶段 则不会产生OUT中断，硬件直接回ACK */
        if(pid == OUT_TOKEN)
        {
            usbd_ep0_out_handler();
        }
        
        if(pid == IN_TOKEN)
        {
            usbd_ep0_in_handler();
        }
	}
    else/* Other EP handling */
    {
        usbd_data_ep_handler(num, dir);
    }
}


void USBD_StallEP(uint8_t ep)
{
    USB0->ENDPOINT[ep].ENDPT |= USB_ENDPT_EPSTALL_MASK;
}

void USB0_IRQHandler(void)
{    
    uint32_t istr, stat;

    istr  = USB0->ISTAT;
    stat  = USB0->STAT;
    USB0->ISTAT = istr;

    istr &= USB0->INTEN;
    
    if(istr & USB_ISTAT_USBRST_MASK)
    {
        USBD_TRACE("usbd bus reset\r\n");
        USB_Reset();
        usbd_stack_reset();
    }
    
    if(istr & USB_ISTAT_SOFTOK_MASK) 
	{
        USBD_TRACE("usbd sof\r\n"); 
	}
    
    if(istr & USB_ISTAT_STALL_MASK)
	{
		USBD_TRACE("usbd stall\r\n");
        
        if(USB0->ENDPOINT[0].ENDPT & USB_ENDPT_EPSTALL_MASK)
        {
            USB0->ENDPOINT[0].ENDPT &= ~USB_ENDPT_EPSTALL_MASK;
        }
	}
    
    if(istr & USB_ISTAT_TOKDNE_MASK) 
	{
		USB_Token_Handle(stat);
	}

    if(istr & USB_ISTAT_SLEEP_MASK) 
	{
		USBD_TRACE("usbd sleep\r\n");  
	}
    
    if(istr & USB_ISTAT_ERROR_MASK)
	{
        uint8_t err = USB0->ERRSTAT;
        printf("ERROR:0x%X\r\n", err);
        USB0->ERRSTAT = err;
	}
}

uint32_t ep_read(uint8_t ep, uint8_t *buf, uint8_t data01)
{
    return USBD_EPRead(ep, buf, data01);
}

uint32_t ep_write(uint8_t ep, uint8_t *buf, uint32_t len, uint8_t data01)
{
    USBD_EPWrite(ep, buf, len, data01);
    return 0;
}

uint32_t set_addr(uint8_t addr)
{
    USB_SetAddr(addr);
    return 0;
}

uint32_t get_ep_max_size(uint8_t ep)
{
    return EP0_MAX_SIZE;
}

uint32_t usbd_config_ep(struct uendpoint_descriptor* d)
{
    USBD_TRACE("usbd_config_ep:0x%X\r\n", d->bEndpointAddress);
    return 0;
}

uint32_t ep_clear_feature(uint8_t ep)
{
    USBD_TRACE("ep_clear_feature\r\n");
    return 0;
}

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

