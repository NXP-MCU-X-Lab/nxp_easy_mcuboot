#include <string.h>

#include "usb_common.h"
#include "usbd.h"
#include "usbd_cdc.h"

struct ucdc_t
{
    struct usbd_t                   *h;
    struct usbd_cdc_callback_t     *cb;
    struct ucdc_line_coding         line_coding;
};

static struct ucdc_t cdc;

uint32_t cdc_standard_request_to_intf_handler(struct usbd_t *h)
{
    if(h->setup.request == USB_REQ_SET_INTERFACE)
    {
        USBD_TRACE("CDC USB_REQ_SET_INTERFACE\r\n");
        usbd_status_in_stage();
    }
    return CH_OK;
}

uint32_t cdc_class_request_handler(struct usbd_t *h)
{
    switch(h->setup.request)
    {
        case 0x21:  /* get line coding */
            if(cdc.cb && cdc.cb->get_line_coding)
            {
                cdc.cb->get_line_coding(&cdc.line_coding);
                begin_data_in_stage((uint8_t*)&cdc.line_coding, sizeof(struct ucdc_line_coding));
            }
            break;
        case 0x22:  /* set control line state */
            if(cdc.cb && cdc.cb->set_control_line_serial_state)
            {
                cdc.cb->set_control_line_serial_state(h->setup.value & 0x01);
            }
            usbd_status_in_stage();
            USBD_TRACE("set control line state 0x%X\r\n", h->setup.value);
            break;
        case 0x20:
            USBD_TRACE("set line coding\r\n");
            break;
        default:
            USBD_TRACE("unknown class request from cdc\r\n");
            break;
    }

    return CH_OK;
}

uint32_t cdc_setup_out_data_received(uint8_t *buf, uint32_t len)
{
    return cdc.cb->set_line_coding((struct ucdc_line_coding *)buf);
}

static uint8_t cdc_out_buf[CDC_EP_SIZE];

/* the maxium len cannot reach MAX_EPSIZE */
uint32_t usbd_cdc_send(uint8_t *buf, uint32_t len)
{
    usbd_ep_write(USBD_CDC_ACM_EP_BULKIN, buf, len);
    return CH_OK;
}

uint32_t cdc_data_ep_handler(uint8_t ep, uint8_t dir)
{
    uint32_t size;
    if(dir == 1) /* in transfer */
    {
        if(ep == USBD_CDC_ACM_EP_BULKIN)
        {
            USBD_TRACE("CDC EP:%d IN TOKEN\r\n", ep);
            if(cdc.cb && cdc.cb->send_notify)
            {
                cdc.cb->send_notify();
            }
        }
    }
    else /* out transfer */
    {
        if(ep == USBD_CDC_ACM_EP_BULKOUT)
        {
            size = usbd_ep_read(ep, cdc_out_buf);
            if(cdc.cb && cdc.cb->recv_handler)
            {
                cdc.cb->recv_handler(cdc_out_buf, size);
            }
        }
    }

    return CH_OK;
}

void usbd_cdc_set_cb(struct usbd_cdc_callback_t *cb)
{
    cdc.cb = cb;
}

void usbd_cdc_init(struct usbd_t *h)
{
    cdc.h = h;
    uint8_t *p;
    struct uconfig_descriptor *uconfiguration_descriptor;
    desc_t d;

    get_descriptor_data("configuration_descriptor", &d);
    uconfiguration_descriptor = (struct uconfig_descriptor *)d.buf;

    uconfiguration_descriptor->bLength = USB_DESC_LENGTH_CONFIG;
    uconfiguration_descriptor->type = USB_DESC_TYPE_CONFIGURATION;
    uconfiguration_descriptor->wTotalLength = USB_DESC_LENGTH_CONFIG;
    uconfiguration_descriptor->bNumInterfaces = 2;
    uconfiguration_descriptor->bConfigurationValue = 1;
    uconfiguration_descriptor->iConfiguration = 0;
    uconfiguration_descriptor->bmAttributes = 0x80;
    uconfiguration_descriptor->MaxPower = 0x32;

    /* add configuation data */
    p = uconfiguration_descriptor->data;

    get_descriptor_data("cdc_acm_if0", &d);
    memcpy(p, d.buf, d.len);
    p += d.len;
    uconfiguration_descriptor->wTotalLength += d.len;

    get_descriptor_data("cdc_acm_if1", &d);
    memcpy(p, d.buf, d.len);
    uconfiguration_descriptor->wTotalLength += d.len;

    /* install descriptor */
    get_descriptor_data("device_descriptor", &d);
    struct udevice_descriptor* device_desc = (struct udevice_descriptor*)d.buf;
    device_desc->bDeviceClass = USB_CLASS_CDC;

    /* install class callback */
    cdc.h->class_request_handler = cdc_class_request_handler;
    cdc.h->standard_request_to_intf_handler = cdc_standard_request_to_intf_handler;
    cdc.h->setup_out_data_received_handler = cdc_setup_out_data_received;
    cdc.h->data_ep_handler = cdc_data_ep_handler;
    cdc.h->vender_request_handler = NULL;
}
