#include <string.h>

#include "usb_common.h"
#include "usbd.h"
#include "usbd_hid.h"

struct uhid_t
{
    struct usbd_t *h;              /* usbd handle */
    struct usbd_hid_callback_t *cb;
    uint8_t out_buf[64];
    uint8_t keyboard_in_buf[8];
    uint8_t mouse_in_buf[4];
};

static struct uhid_t hid;


uint32_t hid_class_request_handler(struct usbd_t *h)
{
    USBD_TRACE("class request:0x%X\r\n", h->setup.request);

    if(h->setup.request == 0x0A)
    {
        /* HID:set idle, do nothing */
        USBD_TRACE("reqeust to class: HID set idle\r\n");
        usbd_status_in_stage();
    } 
    return CH_OK;
}

uint32_t hid_standard_request_to_intf_handler(struct usbd_t *h)
{
    desc_t d;
    if(h->setup.request == USB_REQ_GET_DESCRIPTOR)
    { 
        if((h->setup.value >> 8) == USB_DESC_TYPE_REPORT)
        {
            /* get hid report descriptor class index 0 */
            if(h->setup.index == USBD_HID2_IF_IDX)
            {
                get_descriptor_data("report_descriptor_keyboard", &d);
                USBD_TRACE("send key boarad report descrtipor size:%d\r\n", d.len);
                begin_data_in_stage((uint8_t*)d.buf, d.len);
            }
            
            if(h->setup.index == USBD_HID1_IF_IDX)
            {
                get_descriptor_data("report_descriptor_mouse", &d);
                USBD_TRACE("send mouse report descrtipor size:%d\r\n", d.len);
                begin_data_in_stage((uint8_t*)d.buf, d.len);
            }
            
            if(h->setup.index == USBD_HID0_IF_IDX)
            {
                get_descriptor_data("report_descriptor_custom", &d);
                USBD_TRACE("send custom report descrtipor size:%d\r\n", d.len);
                begin_data_in_stage((uint8_t*)d.buf, d.len);
            }
        }
    }

    return CH_OK;
}


uint32_t hid_data_ep_handler(uint8_t ep, uint8_t dir)
{
    uint32_t size;
    
    switch(ep)
    {
        case USBD_HID1_EP_INTIN:
        case USBD_HID0_EP_INTIN:
        case USBD_HID2_EP_INTIN:
            if(hid.cb)
            {
                if(dir == 1) /* in transfer */
                {
                    hid.cb->data_send_nty(ep);
                }
                else
                {
                    size = usbd_ep_read(ep, hid.out_buf);
                    hid.cb->data_received(ep, hid.out_buf, size);
                }
            }
            break;
    }
    return CH_OK;
}


/*
 *  Get Mouse Input Report -> MouseInReport
 *    Parameters:      report:
 *                               Byte0.0: 1st Button (Left)
 *                               Byte0.1: 2nd Button (Right)
 *                               Byte0.2: 3rd Button
 *                               Byte1:   Relative X Pos
 *                               Byte2:   Relative Y Pos
 *                               Byte3:   Relative Wheel Pos
 *                     size:     report size
 *    Return Value:    None
 */
uint32_t usbd_set_mouse(uint8_t btn, int8_t x, int8_t y, int8_t wheel)
{
    uint8_t *p = hid.mouse_in_buf;
    
    p[0] = btn;
    p[1] = x;
    p[2] = y;
    p[3] = wheel;
    
    usbd_ep_write(USBD_HID1_EP_INTIN, hid.mouse_in_buf, 4);
    return CH_OK;
}
    
uint32_t usbd_set_keyboard(uint8_t fun_key, uint8_t key)
{
    uint8_t *p = hid.keyboard_in_buf;
    
    p[0] = fun_key;
    p[1] = 0x00; /* reserved */
    p[2] = key;
    p[3] = 0;
    p[4] = 0;
    p[5] = 0;
    p[6] = 0;
    p[7] = 0;
    
    usbd_ep_write(USBD_HID0_EP_INTIN, hid.keyboard_in_buf, 8);
    return CH_OK;
}

void usbd_hid_set_cb(struct usbd_hid_callback_t *cb)
{
    hid.cb = cb;
}

void usbd_hid_init(struct usbd_t *h, uint32_t intf_cnt)
{
    hid.h = h;
    uint8_t *p;
    struct uconfig_descriptor *uconfiguration_descriptor;
    desc_t d;
    
    get_descriptor_data("configuration_descriptor", &d);
    uconfiguration_descriptor = (struct uconfig_descriptor *)d.buf;
    
    uconfiguration_descriptor->bLength = USB_DESC_LENGTH_CONFIG;
    uconfiguration_descriptor->type = USB_DESC_TYPE_CONFIGURATION;
    uconfiguration_descriptor->wTotalLength = USB_DESC_LENGTH_CONFIG;
    
    uconfiguration_descriptor->bNumInterfaces = intf_cnt; /* hid interface num */
    uconfiguration_descriptor->bConfigurationValue = 1;
    uconfiguration_descriptor->iConfiguration = 0;
    uconfiguration_descriptor->bmAttributes = 0x80;
    uconfiguration_descriptor->MaxPower = 0x32;
    
    p = uconfiguration_descriptor->data;
    
    get_descriptor_data("hid2_descriptor", &d);
    memcpy(p, d.buf, d.len);
    p += d.len;
    uconfiguration_descriptor->wTotalLength += d.len;
    
    if(intf_cnt == 2)
    {
        get_descriptor_data("hid1_descriptor", &d);
        memcpy(p, d.buf, d.len);
        p += d.len;
        uconfiguration_descriptor->wTotalLength += d.len;
    }

    if(intf_cnt == 3)
    {
        get_descriptor_data("hid2_descriptor", &d);
        memcpy(p, d.buf, d.len);
        p += d.len;
        uconfiguration_descriptor->wTotalLength += d.len;
    }
    
    /* install class callback */
    hid.h->class_request_handler = hid_class_request_handler;
    hid.h->standard_request_to_intf_handler = hid_standard_request_to_intf_handler;
    hid.h->data_ep_handler = hid_data_ep_handler;
    hid.h->setup_out_data_received_handler = NULL;
    hid.h->vender_request_handler = NULL;
}


