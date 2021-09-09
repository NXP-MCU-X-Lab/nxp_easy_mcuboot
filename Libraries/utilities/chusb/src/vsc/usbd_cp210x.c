#include <string.h>

#include "usb_common.h"
#include "usbd.h"
#include "usbd_cp210x.h"
#include "uart_bridge.h"
#include "uart.h"
/* https://www.silabs.com/documents/public/application-notes/AN571.pdf */

typedef struct
{
    uint16_t vid;
    uint16_t pid;
    const char* name;
}cp210x_id_t;


cp210x_id_t cp210x_id_table[] = 
{
    {0x10C4, 0xEA60, "CP2102"},   /*   USB-1UART */
    {0x10C4, 0xEA70, "CP2105"},   /*   USB-2UART */
    {0x10C4, 0xEA71, "CP2108"},   /*   USB-4UART */
};

typedef struct
{
    struct usbd_t                   *h;
    cp210x_id_t                     id;
    struct usbd_cp210x_callback_t   *cb;
}cp210x_t;

static cp210x_t cp210x;



#define USBD_CP210X_CH0_BULKIN             (1)
#define USBD_CP210X_CH0_BULKOUT            (2)
#define USBD_CP210X_BUCK_SIZE              (512)


static cp210x_cpr_t cpr;
static cp210x_ssr_t ssr;
static uint8_t cdc_out_buf[USBD_CP210X_BUCK_SIZE];
static rt_sem_t cp_out_sem;
extern rt_mutex_t usb_lock;

const uint8_t cp210x_descriptor[] =
{
    /* CH0 */
    USB_DESC_LENGTH_INTERFACE,
    USB_DESC_TYPE_INTERFACE,
    0,                                  /* interfac index */
    0x00,
    0x02,                               /* 2 eps */
	USB_CLASS_VEND_SPECIFIC,
	0x00,
	0x00,
    USBD_IF_STR_IDX(USBD_MSC_IF_IDX),
  
	/* eps descriptor */
	0x07,
	USB_DESC_TYPE_ENDPOINT,
	(0x00 | USBD_CP210X_CH0_BULKOUT),
	0x02,
	WBVAL(USBD_CP210X_BUCK_SIZE),
	0,
    
	0x07,
	USB_DESC_TYPE_ENDPOINT,
	(0x80 | USBD_CP210X_CH0_BULKIN),
	0x02,                               /* buck endpoint */
	WBVAL(USBD_CP210X_BUCK_SIZE),       /* size */
	0,                                  /* internal, 0 in buck */
};

/* data handler */
static uint32_t cp210x_data_ep_handler(uint8_t ep, uint8_t dir)
{
    if(dir == 1) /* in */
    {
        if(ep == USBD_CP210X_CH0_BULKIN)
        {
            bridge_uart_usb_data_in_ready();
        }
    }
    else /* out */
    {
        if(ep == USBD_CP210X_CH0_BULKOUT)
        {
            rt_sem_release(cp_out_sem);
        }
    }
    
    return 0;
}

/* handle vender specfic class request */
static uint32_t cp210x_vender_request_handler(struct usbd_t *h)
{
    uint8_t in_resp[8];
    
    switch(h->setup.request)
    {
        case CP210X_VENDOR_SPECIFIC:
            USBD_TRACE("CP210X_VENDOR_SPECIFIC\r\n");
            switch(h->setup.value)
            {
                case 0x370B:
                    in_resp[0] = 0;
                    begin_data_in_stage(in_resp, 1);
                    break;
                default:
                    USBD_TRACE("unknown vender specific request value:%X\r\n", h->setup.value);
                    break;
            }
            break;
        case CP210X_SET_LINE_CTL:
            USBD_TRACE("CP210X_SET_LINE_CTL\r\n");
            break;
        case CP210X_SET_CHAR:
            USBD_TRACE("CP210X_SET_CHAR\r\n");
            break;
        case CP210X_IFC_ENABLE:
            USBD_TRACE("CP210X_IFC_ENABLE\r\n");
            break;
        case CP210X_SET_BAUDDIV:
            USBD_TRACE("CP210X_SET_BAUDDIV\r\n");
            break;
        case CP210X_GET_MDMSTS:
            USBD_TRACE("CP210X_GET_MDMSTS\r\n");
            in_resp[0] = 0;
            begin_data_in_stage(in_resp, 1);
            break;
        case CP210X_SET_FLOW:
            USBD_TRACE("CP210X_SET_FLOW\r\n");
            break;
        case CP210X_SET_CHARS:
            USBD_TRACE("CP210X_SET_CHARS\r\n");
            break;
        case CP210X_SET_BAUDRATE:
            USBD_TRACE("CP210X_SET_BAUDRATE\r\n");
            break;
        case CP210X_GET_COMM_STATUS:
            USBD_TRACE("CP210X_GET_COMM_STATUS\r\n");
            ssr.ulAmountInOutQueue = 0;
            begin_data_in_stage((uint8_t*)&ssr, sizeof(ssr));
            break;
        case CP210X_GET_PROPS:
            USBD_TRACE("CP210X_GET_PROPS\r\n");
            cpr.wLength = sizeof(cp210x_cpr_t);
            cpr.bcdVersion = 0x0100;
            cpr.ulServiceMask = 0x00000001;
            cpr.ulMaxTxQueue = USBD_CP210X_BUCK_SIZE;
            cpr.ulCurrentRxQueue = USBD_CP210X_BUCK_SIZE;
            cpr.ulMaxBaud = 1*1000*1000;
            cpr.ulProvSubType = 0;
            cpr.ulProvCapabilities = 0x00;
            cpr.ulSettableParams = 0x3F;
            cpr.ulSettableBaud = 0xFFFFFFFF;
            cpr.wSettableData = 0xFF;
            cpr.ulCurrentTxQueue = USBD_CP210X_BUCK_SIZE;
            cpr.ulCurrentRxQueue = USBD_CP210X_BUCK_SIZE;
            memcpy(cpr.uniProvName, L"SILABS USB V1.0", sizeof(cpr.uniProvName));
            
            begin_data_in_stage((uint8_t*)&cpr, cpr.wLength);
            break;
        default:
            USBD_TRACE("Unknown cp210x vender specfic reqeust\r\n");
            break;
    }
    
    if(h->setup.length == 0 && (h->setup.request_type & 0x80) == 0x00)
    {
        usbd_status_in_stage();
    }
    return 0;
}


void cp_out_thread_entry(void* parameter)
{
    int i, free, size;
    while(1)
    {
        rt_sem_take(cp_out_sem, RT_WAITING_FOREVER); 
        /* read data from USB */
        rt_mutex_take(usb_lock, RT_WAITING_FOREVER);
        size = usbd_ep_read(USBD_CP210X_CH0_BULKOUT, cdc_out_buf);
        rt_mutex_release(usb_lock);
        
        for(i=4; i<5; i++)
        {
            /* get free buffer */
            free = bridge_uart_tx_get_free(i);
            while(free < size)
            {
                free = bridge_uart_tx_get_free(i);
                rt_thread_delay(1);
            }
            
            bridge_uart_send(i, cdc_out_buf, size);
        }
    }
}

void usbd_vsc_cp210x_init(struct usbd_t *h)
{
    cp210x.id = cp210x_id_table[0];
    
    uint8_t *p;
    struct uconfig_descriptor *uconfiguration_descriptor;
    desc_t d;
    
    /* make descriptor */
    get_descriptor_data("device_descriptor", &d);
    struct udevice_descriptor* device_desc = (struct udevice_descriptor*)d.buf;
    device_desc->bDeviceClass = 0x00;
    device_desc->bDeviceSubClass = 0x00;
    device_desc->bDeviceProtocol = 0x00;
    device_desc->idVendor =  cp210x.id.vid;
    device_desc->idProduct = cp210x.id.pid;
    
    /* make configuration descriptor */
    get_descriptor_data("configuration_descriptor", &d);
    uconfiguration_descriptor = (struct uconfig_descriptor *)d.buf;
    
    uconfiguration_descriptor->bLength = USB_DESC_LENGTH_CONFIG;
    uconfiguration_descriptor->type = USB_DESC_TYPE_CONFIGURATION;
    uconfiguration_descriptor->wTotalLength = USB_DESC_LENGTH_CONFIG;
    uconfiguration_descriptor->bNumInterfaces = 1;                      /*  interfae count */
    uconfiguration_descriptor->bConfigurationValue = 1;
    uconfiguration_descriptor->iConfiguration = 0;
    uconfiguration_descriptor->bmAttributes = 0x80;
    uconfiguration_descriptor->MaxPower = 0x32;
    
    /* make intf and add to configuation data */
    p = uconfiguration_descriptor->data;
    
    d.buf = cp210x_descriptor;
    d.len =  sizeof(cp210x_descriptor);
    memcpy(p, d.buf, d.len);
    p += d.len;
    uconfiguration_descriptor->wTotalLength += d.len;
    
    h->vender_request_handler = cp210x_vender_request_handler;
    h->data_ep_handler = cp210x_data_ep_handler;
    
    rt_thread_t tid;
    
    cp_out_sem = rt_sem_create("cpout", 0, RT_IPC_FLAG_FIFO);
    
    tid = rt_thread_create("cpout", cp_out_thread_entry, RT_NULL, 512, 19, 20);
    rt_thread_startup(tid);
}

