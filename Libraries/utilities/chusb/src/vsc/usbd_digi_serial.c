#include <string.h>

#include "usb_common.h"
#include "usbd.h"
#include "usbd_digi_serial.h"
#include "uart_bridge.h"
#include "uart.h"
#include "ulog.h"

#define DIGI_USBD_VID     (0x1FC9)
#define DIGI_USBD_PID     (0xEA60)

#define USBD_DIGI_SERIAL_CTL_BULKIN             (1)
#define USBD_DIGI_SERIAL_CTL_BULKOUT            (1)
#define USBD_DIGI_SERIAL_DATA_BULKIN            (2)
#define USBD_DIGI_SERIAL_DATA_BULKOUT           (2)

#define DIGI_CMD_SET_BAUD_RATE			0	/* INB, OOB */
#define DIGI_CMD_SET_WORD_SIZE			1	/* INB, OOB */
#define DIGI_CMD_SET_PARITY             2	/* INB, OOB */
#define DIGI_CMD_SET_STOP_BITS			3	/* INB, OOB */
#define DIGI_CMD_SET_INPUT_FLOW_CONTROL 4	/* INB, OOB */
#define DIGI_CMD_SET_OUTPUT_FLOW_CONTROL    5	/* INB, OOB */
#define DIGI_CMD_SET_DTR_SIGNAL			6	/* INB, OOB */
#define DIGI_CMD_SET_RTS_SIGNAL			7	/* INB, OOB */
#define DIGI_CMD_READ_INPUT_SIGNALS		8	/*      OOB */
#define DIGI_CMD_IFLUSH_FIFO			9	/*      OOB */
#define DIGI_CMD_RECEIVE_ENABLE			10	/* INB, OOB */
#define DIGI_CMD_BREAK_CONTROL			11	/* INB, OOB */
#define DIGI_CMD_LOCAL_LOOPBACK			12	/* INB, OOB */
#define DIGI_CMD_TRANSMIT_IDLE			13	/* INB, OOB */
#define DIGI_CMD_READ_UART_REGISTER		14	/*      OOB */
#define DIGI_CMD_WRITE_UART_REGISTER    15	/* INB, OOB */
#define DIGI_CMD_AND_UART_REGISTER		16	/* INB, OOB */
#define DIGI_CMD_OR_UART_REGISTER		17	/* INB, OOB */
#define DIGI_CMD_SEND_DATA              18	/* INB      */
#define DIGI_CMD_RECEIVE_DATA			19	/* INB      */
#define DIGI_CMD_RECEIVE_DISABLE		20	/* INB      */
#define DIGI_CMD_GET_PORT_TYPE			21	/*      OOB */
#define DIGI_CMD_GET_BUFFER_STAT       22


/* baud rates */
#define DIGI_BAUD_50				0
#define DIGI_BAUD_75				1
#define DIGI_BAUD_110				2
#define DIGI_BAUD_150				3
#define DIGI_BAUD_200				4
#define DIGI_BAUD_300				5
#define DIGI_BAUD_600				6
#define DIGI_BAUD_1200				7
#define DIGI_BAUD_1800				8
#define DIGI_BAUD_2400				9
#define DIGI_BAUD_4800				10
#define DIGI_BAUD_7200				11
#define DIGI_BAUD_9600				12
#define DIGI_BAUD_14400				13
#define DIGI_BAUD_19200				14
#define DIGI_BAUD_28800				15
#define DIGI_BAUD_38400				16
#define DIGI_BAUD_57600				17
#define DIGI_BAUD_76800				18
#define DIGI_BAUD_115200			19
#define DIGI_BAUD_153600			20
#define DIGI_BAUD_230400			21
#define DIGI_BAUD_460800			22


static uint8_t digi_cdc_out_buf[512];
extern rt_mutex_t usb_lock;
static rt_sem_t dg_out_sem;
extern int usb_report_stat_chl;
extern int usb_ctl_need_report_stat;
extern stat_t stat;

/* msd */
const uint8_t digi_serial_descriptor[] =
{
    USB_DESC_LENGTH_INTERFACE,
    USB_DESC_TYPE_INTERFACE,
    0,                                  /* interfac index */
    0x00,
    0x04,                               /* 4 end point */
	USB_CLASS_VEND_SPECIFIC,
	0x00,
	0x00,
    USBD_IF_STR_IDX(USBD_MSC_IF_IDX),
  
	/* endpoint descriptor CTL */
	0x07,
	USB_DESC_TYPE_ENDPOINT,
	(0x80 | USBD_DIGI_SERIAL_CTL_BULKIN),
	0x02,   /* buck */
	WBVAL(512),
	0,
   
	0x07,
	USB_DESC_TYPE_ENDPOINT,
	(0x00 | USBD_DIGI_SERIAL_CTL_BULKOUT),
	0x02,
	WBVAL(512),
	0,
    
	/* endpoint descriptor DATA */
	0x07,
	USB_DESC_TYPE_ENDPOINT,
	(0x80 | USBD_DIGI_SERIAL_DATA_BULKIN),
	0x02,
	WBVAL(512),
	0,
   
	0x07,
	USB_DESC_TYPE_ENDPOINT,
	(0x00 | USBD_DIGI_SERIAL_DATA_BULKOUT),
	0x02,
	WBVAL(512),
	0,
};



/* data handler */
static uint32_t digi_data_ep_handler(uint8_t ep, uint8_t dir)
{
    uint8_t digi_cdc_ob_out_buf[128];
    
    uint8_t size, cmd, chl, val;
    if(dir == 1) /* in transfer */
    {
        if(ep == USBD_DIGI_SERIAL_CTL_BULKIN)
        {
            bridge_uart_usb_data_in_ready();
        }
        
        if(ep == USBD_DIGI_SERIAL_DATA_BULKIN)
        {

        }
    }
    else /* out transfer */
    {
        if(ep == USBD_DIGI_SERIAL_DATA_BULKOUT)
        {
            size = usbd_ep_read(ep, digi_cdc_ob_out_buf);
            
            cmd = digi_cdc_ob_out_buf[0];
            chl = digi_cdc_ob_out_buf[1];
            val = digi_cdc_ob_out_buf[2];
            
            switch(cmd)
            {
                case DIGI_CMD_READ_INPUT_SIGNALS:
                 //   LOG_I("CTRLOUT[%d] - DIGI_CMD_READ_INPUT_SIGNALS\r\n", chl);
                    break;
                case DIGI_CMD_SET_INPUT_FLOW_CONTROL:
                 //   LOG_I("CTRLOUT[%d] - DIGI_CMD_SET_INPUT_FLOW_CONTROL\r\n", chl);
                    break;
                case DIGI_CMD_SET_BAUD_RATE:
             //       LOG_I("CTRLOUT[%d] - DIGI_CMD_SET_BAUD_RATE baud:%d\r\n", chl, val);
                    switch(val)
                    {
                        case DIGI_BAUD_4800:
                            UART_SetBaudRate(chl, 4800);
                            break;
                        case DIGI_BAUD_9600:
                            UART_SetBaudRate(chl, 9600);
                            break;
                        case DIGI_BAUD_38400:
                            UART_SetBaudRate(chl, 38400);
                            break;
                        case DIGI_BAUD_115200:
                            UART_SetBaudRate(chl, 115200);
                            break;
                        default:
                            LOG_E("un-supported baud rate");
                            break;
                    }
                    break;
                case DIGI_CMD_LOCAL_LOOPBACK:
           //         LOG_I("CTRLOUT[%d] - DIGI_CMD_LOCAL_LOOPBACK val:%d\r\n", chl, val);
                    UART_SetLoopbackMode(chl, val);
                    break;
                case DIGI_CMD_SET_DTR_SIGNAL:
               //     LOG_I("CTRLOUT[%d] - DIGI_CMD_SET_DTR_SIGNAL\r\n", chl);
                    break;
                case DIGI_CMD_GET_BUFFER_STAT:
                    //LOG_I("CTRLOUT[%d] - DIGI_CMD_GET_BUFFER_STAT\r\n", chl);
                    usb_report_stat_chl = chl;
                    usb_ctl_need_report_stat = 1;
                    break;
                default:
                    LOG_E("unknown ctrl out cmd:%d chl:%d\r\n", cmd, chl);
                    ulog_hexdump("example", 16, digi_cdc_ob_out_buf, size);
                    break;
            }
        }
        
        if(ep == USBD_DIGI_SERIAL_CTL_BULKOUT)
        {
            rt_sem_release(dg_out_sem);
        }
    }
    return 0;
}


void dg_out_thread_entry(void* parameter)
{
    uint32_t size, len, free;
    uint8_t op_code, chl;
    
    while(1)
    {
        rt_sem_take(dg_out_sem, RT_WAITING_FOREVER); 
        
        /* read data from USB */
        size = usbd_ep_read(USBD_DIGI_SERIAL_CTL_BULKOUT, digi_cdc_out_buf);
            
        op_code = digi_cdc_out_buf[0];
        chl = digi_cdc_out_buf[1];
        len = digi_cdc_out_buf[2];
        
        if(op_code == DIGI_CMD_SEND_DATA)
        {
            /* get free buffer */
            free = bridge_uart_tx_get_free(chl);
            while(free < size)
            {
                free = bridge_uart_tx_get_free(chl);
                rt_thread_delay(1);
            }
            stat.total_out += len;
            bridge_uart_send(chl, &digi_cdc_out_buf[3], len);
        }
    }
}
    

/* handle vender specfic class request */
static uint32_t digi_vender_request_handler(struct usbd_t *h)
{
    LOG_I("digi_vender_request_handler\r\n");
    return 0;
}

void usbd_vsc_digi_serial_init(struct usbd_t *h)
{
    uint8_t *p;
    struct uconfig_descriptor *uconfiguration_descriptor;
    desc_t d;
    
    /* make descriptor */
    get_descriptor_data("device_descriptor", &d);
    struct udevice_descriptor* device_desc = (struct udevice_descriptor*)d.buf;
    device_desc->bDeviceClass = USB_CLASS_VEND_SPECIFIC;
    device_desc->bDeviceSubClass = 0x00;
    device_desc->bDeviceProtocol = 0x00;

    /* make configuration descriptor */
    get_descriptor_data("configuration_descriptor", &d);
    uconfiguration_descriptor = (struct uconfig_descriptor *)d.buf;
    
    uconfiguration_descriptor->bLength = USB_DESC_LENGTH_CONFIG;
    uconfiguration_descriptor->type = USB_DESC_TYPE_CONFIGURATION;
    uconfiguration_descriptor->wTotalLength = USB_DESC_LENGTH_CONFIG;
    uconfiguration_descriptor->bNumInterfaces = 1;
    uconfiguration_descriptor->bConfigurationValue = 1;
    uconfiguration_descriptor->iConfiguration = 0;
    uconfiguration_descriptor->bmAttributes = 0x80;
    uconfiguration_descriptor->MaxPower = 0x32;
    device_desc->idVendor =  DIGI_USBD_VID;
    device_desc->idProduct = DIGI_USBD_PID;
    
    /* make intf and add to configuation data */
    p = uconfiguration_descriptor->data;
    
    d.buf = digi_serial_descriptor;
    d.len =  sizeof(digi_serial_descriptor);
    memcpy(p, d.buf, d.len);
    p += d.len;
    uconfiguration_descriptor->wTotalLength += d.len;
    
    h->vender_request_handler = digi_vender_request_handler;
    h->data_ep_handler = digi_data_ep_handler;
    
    dg_out_sem = rt_sem_create("dg_out", 0, RT_IPC_FLAG_FIFO);
    
    rt_thread_t tid;
    tid = rt_thread_create("dg_out", dg_out_thread_entry, RT_NULL, 256, 6, 20);
    rt_thread_startup(tid);
}


