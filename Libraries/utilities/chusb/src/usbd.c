#include "usbd.h"
#include "usb_common.h"

/* usbd device handle */
static struct usbd_t *handle;

/* read data from PC */
uint32_t usbd_ep_read(uint8_t ep, uint8_t *buf)
{
    uint32_t size;
    
    size = handle->ops->ep_read(ep, buf, (handle->data01_out >> ep) & 0x01);
    handle->data01_out ^= (1 << ep);
    
    return size;
}

/* write data to EP to prepare PC to read EP */
uint32_t usbd_ep_write(uint8_t ep, uint8_t *buf, uint32_t len)
{
    uint32_t size;
    
    size = handle->ops->ep_write(ep, buf, len, (handle->data01_in >> ep) & 0x01);
    handle->data01_in ^= (1 << ep);
    return size;
}

void usbd_status_in_stage(void)
{
    /* status in stage, use DATA 1 */
    handle->data01_in |= (1 << 0);
    usbd_ep_write(0, NULL, 0);
}

void usbd_status_out_stage(void)
{
    handle->data01_out |= (1 << 0);
    usbd_ep_read(0, handle->ep0_out_buf);
}

void usbd_data_in_stage(void)
{
    uint32_t len;
    
    if(handle->ep0_in_remain_size > EP0_MAX_SIZE)
    {
        len = EP0_MAX_SIZE;
    }
    else
    {
        len = handle->ep0_in_remain_size;
    }
    
    if(len || handle->need_zlp)
        usbd_ep_write(0, handle->ep0_in_buf, len);

    handle->ep0_in_buf += len;
    handle->ep0_in_remain_size -= len;
}

/*
request_type:
    D7: dir
        0, host to device
        1, device to host
    D6~5: request type:
        0: standard
        1: class
        2: vender
    D4~0: receiver of request
        0: device
        1: interface
        2: endpoint
*/
void _dump_setup_packet(struct urequest *setup)
{
    USBD_TRACE("\r\n[setup:  ");
    USBD_TRACE("request_type:0x%X ", setup->request_type);
    USBD_TRACE("request:0x%X ", setup->request);
    USBD_TRACE("value:0x%X ", setup->value);
    USBD_TRACE("index:0x%X ", setup->index);
    USBD_TRACE("length:%d ", setup->length);
    USBD_TRACE("]\r\n");
}

void usbd_stack_reset(void)
{
    handle->data01_in = 0x00000000;         /* ep_write function, all begin with DATA0 */
    handle->data01_out = 0xFFFFFFFF;        /* ep_read function, means begin with second read, so, must be DATA1 */
    handle->ep0_out_ptr = handle->ep0_out_buf;
    handle->ep0_out_remain_size = 0;
    handle->need_zlp = false;
    handle->is_configured = false;
}

void usbd_init(struct usbd_t *dev, struct usbd_ops_t *ops)
{
    handle = dev;
    handle->ops  = ops;
    usbd_stack_reset();
}

bool usbd_is_configured(void)
{
    return handle->is_configured;
}


void begin_data_in_stage(uint8_t *buf, uint32_t len)
{
    handle->ep0_in_buf = buf;
    handle->ep0_in_remain_size = len;
    
    /* data in stage begin with DATA1 */
    handle->data01_in |= (1 << 0); 
    
    if((len % EP0_MAX_SIZE) == 0 && (len >= EP0_MAX_SIZE))
    {
        handle->need_zlp = true;
    }
    else
    {
        handle->need_zlp = false;
    }
    usbd_data_in_stage();
}

void begin_data_out_stage(void)
{
    handle->ep0_out_remain_size = handle->setup.length;
    handle->ep0_out_ptr = handle->ep0_out_buf;
}

void data_out_stage(void)
{
    uint32_t size;
    size = usbd_ep_read(0, handle->ep0_out_ptr);
    
    handle->ep0_out_ptr += size;
    handle->ep0_out_remain_size -= size;
    
    if(handle->ep0_out_remain_size == 0)
    {
        if(handle->setup_out_data_received_handler)
        {
            handle->setup_out_data_received_handler(handle->ep0_out_buf, handle->setup.length);
        }

        /* status in stage */
        usbd_status_in_stage();
    }
}

    
static uint32_t _get_descriptor(struct urequest* setup)
{
    struct ustring_descriptor ustr;
    struct uconfig_descriptor *uconfig;
    uint8_t string_index;
    uint8_t language_str_buf[] = {0x04, 0x03, 0x09, 0x04};
    desc_t d;
    if(setup->request_type == USB_REQ_TYPE_DIR_IN)
    {
        switch(setup->value >> 8)
        {
            case USB_DESC_TYPE_DEVICE:
                USBD_TRACE("device_descriptor\r\n");
                get_descriptor_data("device_descriptor", &d);
                break;
            
            case USB_DESC_TYPE_CONFIGURATION:
                USBD_TRACE("configuration_descriptor\r\n");
                get_descriptor_data("configuration_descriptor", &d);
            
                uconfig = (struct uconfig_descriptor*)d.buf;
                d.len = uconfig->wTotalLength;

                break;
            case USB_DESC_TYPE_DEVICEQUALIFIER:
                USBD_TRACE("usb_qualifier_descriptor\r\n");
                get_descriptor_data("qualifier_descriptor", &d);
                break;
            case USB_DESC_TYPE_STRING:
                string_index = (setup->value & 0xFF);
                USBD_TRACE("string_descriptor:%d\r\n", string_index);
                
                switch(string_index)
                {
                    case 0:
                        create_string_descriptor(&ustr, (uint8_t*)language_str_buf, sizeof(language_str_buf));
                        d.buf = (uint8_t*)&ustr;
                        d.len = ustr.bLength;
                        break;
                    case 1:
                        create_string_descriptor(&ustr, (uint8_t*)MANUFACTURE_STR, sizeof(MANUFACTURE_STR));
                        d.buf = (uint8_t*)&ustr;
                        d.len = ustr.bLength;
                        break;
                    case 2:
                        create_string_descriptor(&ustr, (uint8_t*)PRODUCT_STR, sizeof(PRODUCT_STR));
                        d.buf = (uint8_t*)&ustr;
                        d.len = ustr.bLength;
                        break;
                    case 3:
                        create_string_descriptor(&ustr, (uint8_t*)SERIAL_STR, sizeof(SERIAL_STR));
                        d.buf = (uint8_t*)&ustr;
                        d.len = ustr.bLength;
                        break;
                    case USBD_IF_STR_IDX(USBD_MSC_IF_IDX):
                        create_string_descriptor(&ustr, (uint8_t*)USBD_MSC_IF_STR, sizeof(USBD_MSC_IF_STR));
                        d.buf = (uint8_t*)&ustr;
                        d.len = ustr.bLength;
                        break;
                    case USBD_IF_STR_IDX(USBD_CDC_CIF_IDX):
                        create_string_descriptor(&ustr, (uint8_t*)USBD_CDC_CIF_STR, sizeof(USBD_CDC_CIF_STR));
                        d.buf = (uint8_t*)&ustr;
                        d.len = ustr.bLength;
                        break;
                    case USBD_IF_STR_IDX(USBD_CDC_DIF_IDX):
                        create_string_descriptor(&ustr, (uint8_t*)USBD_CDC_DIF_STR, sizeof(USBD_CDC_CIF_STR));
                        d.buf = (uint8_t*)&ustr;
                        d.len = ustr.bLength;
                        break;
                    case USBD_IF_STR_IDX(USBD_HID0_IF_IDX):
                        create_string_descriptor(&ustr, (uint8_t*)USBD_HID0_IF_STR, sizeof(USBD_HID0_IF_STR));
                        d.buf = (uint8_t*)&ustr;
                        d.len = ustr.bLength;
                        break;
                    case USBD_IF_STR_IDX(USBD_HID1_IF_IDX):
                        create_string_descriptor(&ustr, (uint8_t*)USBD_HID1_IF_STR, sizeof(USBD_HID1_IF_STR));
                        d.buf = (uint8_t*)&ustr;
                        d.len = ustr.bLength;
                        break;
                    case USBD_IF_STR_IDX(USBD_HID2_IF_IDX):
                        create_string_descriptor(&ustr, (uint8_t*)USBD_HID2_IF_STR, sizeof(USBD_HID2_IF_STR));
                        d.buf = (uint8_t*)&ustr;
                        d.len = ustr.bLength;
                        break;
                    default:
                        USBD_TRACE("unknown string index:%d, use default\r\n", string_index);
                        break;
                }
                break;
            case USB_DESC_TYPE_OTHERSPEED:
                USBD_TRACE("other speed configuration\r\n");
                get_descriptor_data("configuration_descriptor", &d);
                {
                    struct uconfig_descriptor *p = (struct uconfig_descriptor*)d.buf;
                    p->type = USB_DESC_TYPE_OTHERSPEED;
                    d.len = p->wTotalLength;
                }
                break;
            default:
                USBD_TRACE("unsupported descriptor request:%d\n", setup->value >> 8);
                d.buf = NULL;
                d.len = 0;
                break;
        }
       
        if(setup->length < d.len)
        {
            begin_data_in_stage((uint8_t*)d.buf, setup->length);
        }
        else
        {
            begin_data_in_stage((uint8_t*)d.buf, d.len);
        }
        
    }
    else
    {
        USBD_TRACE("request direction error\n");
    }
    return CH_OK; 
}

static uint32_t _standard_request(struct urequest* setup)
{
    desc_t d;
    uint8_t *p;
    uint8_t ofs, len;
    struct uconfig_descriptor *uconfiguration_descriptor;
    
    switch(setup->request_type & USB_REQ_TYPE_RECIPIENT_MASK)
    {
        case USB_REQ_TYPE_DEVICE:
            switch(setup->request)
            {
                case USB_REQ_GET_DESCRIPTOR:
                    _get_descriptor(setup);
                    break;
                case USB_REQ_SET_ADDRESS:
                    USBD_TRACE("standard request: set addr:%d\r\n", setup->value);
                    handle->addr = (setup->value & 0x7F);
                    usbd_status_in_stage();
                    break;
                case USB_REQ_SET_CONFIGURATION:
                    USBD_TRACE("standard request: set configuration\r\n");
                
                    /* find endpoint descriptor and call ep_config */
                    get_descriptor_data("configuration_descriptor", &d);
                    uconfiguration_descriptor = (struct uconfig_descriptor *)d.buf;
                
                    p = (uint8_t*)d.buf;
                    ofs = 0;
                    len = 0;
                    
                    while((ofs < uconfiguration_descriptor->wTotalLength) && (*p) != 0)
                    {
                        len = (*p);
                        switch(*(p+1))
                        {
                            case USB_DESC_TYPE_ENDPOINT:
                                if(handle->ops->ep_config)
                                {
                                    handle->ops->ep_config((struct uendpoint_descriptor*)(p));
                                }
                                break;
                        }
                        p += len;
                        ofs += len;
                    }
                    usbd_status_in_stage();
                    handle->is_configured = true;
                    break;
                case USB_REQ_GET_STATUS:
                    USBD_TRACE("standard request, get status\r\n");
                    static uint8_t get_status_responds[2] = {0x00, 0x00};
                    begin_data_in_stage(get_status_responds, 2);
                    break;
                default:
                    USBD_TRACE("unkown request %d to device \r\n", setup->request);
                    break;
            }
            break;
        case USB_REQ_TYPE_INTERFACE:
            USBD_TRACE("request to interface\r\n");
            handle->standard_request_to_intf_handler(handle);
            break;
        case USB_REQ_TYPE_ENDPOINT:
            if(setup->request == USB_REQ_CLEAR_FEATURE)
            {
                handle->ops->ep_clear_feature(handle->setup.index);
            }
            else
            {
                USBD_TRACE("other request to endpoint\r\n");
            }
            usbd_status_in_stage();
            break;
        case USB_REQ_TYPE_OTHER:
            USBD_TRACE("unknown other type request\n");
            break;
        default:
            USBD_TRACE("unknown device request\n");
            break;
    }
    return CH_OK;
}

uint32_t usbd_ep0_setup_handler(void)
{
    int size;
    
    size = usbd_ep_read(0, (uint8_t*)&handle->setup);
    if(size != sizeof(struct urequest))
    {
        USBD_TRACE("read setup packet error size:%d\r\n", size);
        return CH_ERR;
    }
    _dump_setup_packet(&handle->setup);
    
    /* if has data stage && dir = OUT. means there is data out stage */
    if(handle->setup.length && ((handle->setup.request_type & 0x80) == 0x00))
    {
        handle->data01_out |= (1 << 0); /* force set to DATA1 */
        begin_data_out_stage();
    }
    else /* next is a setup package */
    {
        handle->data01_out &= ~(1 << 0); /* force set to DATA0 */
    }
    
    switch(handle->setup.request_type & USB_REQ_TYPE_MASK)
    {
        case USB_REQ_TYPE_STANDARD:
            _standard_request(&handle->setup);
            break;
        case USB_REQ_TYPE_CLASS:
            handle->class_request_handler(handle);
            break;
        case USB_REQ_TYPE_VENDOR:
            USBD_TRACE("vendor type request\n");
            if(handle->vender_request_handler != NULL)
            {
                handle->vender_request_handler(handle);
            }
            break;
        default:
            USBD_TRACE("unknown setup request type\n");
            //rt_usbd_ep0_set_stall(device);
            return CH_ERR;
    }
    return CH_OK;
}

uint32_t usbd_ep0_out_handler(void)
{
    if(((handle->setup.request_type & 0x80) == 0x00) && handle->setup.length) /* data out stage */
    {
        USBD_TRACE("Data out stage\r\n");
        data_out_stage();
    }
    else
    {
        usbd_status_out_stage();
    }
    
    return CH_OK;
}

/* 0:OUT packet  1:IN packet */
void usbd_data_ep_handler(uint8_t ep, uint8_t dir)
{
    handle->data_ep_handler(ep, dir);
}

uint32_t usbd_ep0_in_handler(void)
{
    /* host to device direction means it's a set address request */
    if((handle->setup.request_type & 0x80) == USB_REQ_TYPE_DIR_OUT)
    {
        handle->ops->set_addr(handle->addr);
    }
    else
    {
        usbd_data_in_stage();
    }
    return CH_OK;
}

