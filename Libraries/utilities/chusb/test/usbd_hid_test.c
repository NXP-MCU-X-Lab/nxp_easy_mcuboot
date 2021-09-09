#include "usbd_hid.h"

extern struct usbd_t usbd;

uint32_t data_send_notify(uint8_t ep)
{
    printf("ep%d: data_send_notify\r\n", ep);
    return CH_OK;
}

uint32_t hid_data_received(uint8_t ep, uint8_t *buf, uint32_t len)
{
    int i;
    printf("ep%d: received, size:%d\r\n", ep, len);
    for(i=0; i<len; i++)
    {
        printf("%02X ", buf[i]);
    }
    printf("\r\n");
    return CH_OK;
}

struct usbd_hid_callback_t hid_cb = 
{
    data_send_notify,
    hid_data_received,
};

void usbd_hid_test(struct usbd_t *h)
{
    usbd_hid_set_cb(&hid_cb);
    usbd_hid_init(h, 2);
}
