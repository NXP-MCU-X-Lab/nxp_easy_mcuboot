#include <string.h>

#include "usb_common.h"
#include "usbd.h"
#include "usbd_cdc.h"
#include "usbd_hid.h"
#include "usbd_msc.h"
#include "usbd_composite.h"


static struct usbd_t *handle = NULL;

static uint32_t composite_standard_request_to_intf_handler(struct usbd_t *h)
{
    switch(handle->setup.index)
    {
        case USBD_MSC_IF_IDX:
            msc_standard_request_to_intf_handler(h);
            break;
        case USBD_CDC_CIF_IDX:
            cdc_standard_request_to_intf_handler(h);
            break;
        case USBD_HID0_IF_IDX:
        case USBD_HID1_IF_IDX:
        case USBD_HID2_IF_IDX:
            hid_standard_request_to_intf_handler(h);
            break;
    }
    return CH_OK;
}

static uint32_t composite_class_request_handler(struct usbd_t *h)
{
    switch(handle->setup.index)
    {
        case USBD_MSC_IF_IDX:
            msc_class_request_handler(h);
            break;
        case USBD_CDC_CIF_IDX:
            cdc_class_request_handler(h);
            break;
        case USBD_HID0_IF_IDX:
        case USBD_HID1_IF_IDX:
        case USBD_HID2_IF_IDX:
            hid_class_request_handler(h);
            break;
    }
    return CH_OK;
}

static uint32_t composite_data_ep_handler(uint8_t ep, uint8_t dir)
{
    hid_data_ep_handler(ep, dir);
    cdc_data_ep_handler(ep, dir);
    msc_data_ep_handler(ep, dir);
    return CH_OK;
}


uint32_t composite_setup_out_data_received_handler(uint8_t *buf, uint32_t len)
{
    switch(handle->setup.index)
    {
        case USBD_CDC_CIF_IDX:
        case USBD_CDC_DIF_IDX:
            cdc_setup_out_data_received(buf, len);
            break;
    }
    return CH_OK;
}

void usbd_composite_init(struct usbd_t *h, uint32_t option)
{
    handle = h;
    uint8_t *p;
    struct uconfig_descriptor *uconfiguration_descriptor;
    desc_t d;
    
    get_descriptor_data("configuration_descriptor", &d);
    uconfiguration_descriptor = (struct uconfig_descriptor *)d.buf;
    
    uconfiguration_descriptor->bLength = USB_DESC_LENGTH_CONFIG;
    uconfiguration_descriptor->type = USB_DESC_TYPE_CONFIGURATION;
    uconfiguration_descriptor->wTotalLength = USB_DESC_LENGTH_CONFIG;
    uconfiguration_descriptor->bNumInterfaces = 0;
    uconfiguration_descriptor->bConfigurationValue = 1;
    uconfiguration_descriptor->iConfiguration = 0;
    uconfiguration_descriptor->bmAttributes = 0x80;
    uconfiguration_descriptor->MaxPower = 0x32;
    
    /* add configuation data */
    p = uconfiguration_descriptor->data;
    
    if(option & USBD_USE_MSC)
    {
        uconfiguration_descriptor->bNumInterfaces++;
        
        get_descriptor_data("msc_descriptor", &d);
        memcpy(p, d.buf, d.len);
        p += d.len;
        uconfiguration_descriptor->wTotalLength += d.len;
    }
    
    if(option & USBD_USE_CDC)
    {
        uconfiguration_descriptor->bNumInterfaces += 2;
        
        get_descriptor_data("cdc_acm_iad_descriptor", &d);
        memcpy(p, d.buf, d.len);
        p += d.len;
        uconfiguration_descriptor->wTotalLength += d.len;
        
        get_descriptor_data("cdc_acm_if0", &d);
        memcpy(p, d.buf, d.len);
        p += d.len;
        uconfiguration_descriptor->wTotalLength += d.len;
        
        get_descriptor_data("cdc_acm_if1", &d);
        memcpy(p, d.buf, d.len);
        p += d.len;
        uconfiguration_descriptor->wTotalLength += d.len;
    }
    
    if(option & USBD_USE_HID0)
    {
        uconfiguration_descriptor->bNumInterfaces++;
        
        get_descriptor_data("hid0_descriptor", &d);
        memcpy(p, d.buf, d.len);
        p += d.len;
        uconfiguration_descriptor->wTotalLength += d.len;
    }

    if(option & USBD_USE_HID1)
    {
        uconfiguration_descriptor->bNumInterfaces++;
        
        get_descriptor_data("hid1_descriptor", &d);
        memcpy(p, d.buf, d.len);
        p += d.len;
        uconfiguration_descriptor->wTotalLength += d.len;
    }
    
    if(option & USBD_USE_HID2)
    {
        uconfiguration_descriptor->bNumInterfaces++;
        
        get_descriptor_data("hid2_descriptor", &d);
        memcpy(p, d.buf, d.len);
        p += d.len;
        uconfiguration_descriptor->wTotalLength += d.len;
    }
    
    /* install descriptor */
    get_descriptor_data("device_descriptor", &d);
    struct udevice_descriptor* device_desc = (struct udevice_descriptor*)d.buf;
    device_desc->bDeviceClass = USB_CLASS_MISC;
    device_desc->bDeviceSubClass = 0x02;
    device_desc->bDeviceProtocol = 0x01;

    /* install class callback */
    handle->class_request_handler = composite_class_request_handler;
    handle->standard_request_to_intf_handler = composite_standard_request_to_intf_handler;
    handle->setup_out_data_received_handler = composite_setup_out_data_received_handler;
    handle->data_ep_handler = composite_data_ep_handler;
}
