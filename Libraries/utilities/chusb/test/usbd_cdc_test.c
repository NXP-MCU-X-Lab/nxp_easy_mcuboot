#include "usbd_cdc.h"

/* host need device's line_coding config */
uint32_t get_line_coding(struct ucdc_line_coding *line_coding)
{
    line_coding->dwDTERate = 115200;
    line_coding->bCharFormat = 2;
    USBD_TRACE("get_line_coding, baud:%d\r\n", line_coding->dwDTERate);
    return CH_OK;
}

/* device need to implment host's config */
uint32_t set_line_coding(struct ucdc_line_coding *line_coding)
{
    USBD_TRACE("set line coding:%d\r\n", line_coding->dwDTERate);
    return CH_OK;
}

uint32_t cdc_data_received(uint8_t *buf, uint32_t len)
{
    printf("cdc_data_received %d\r\n", len);
    return CH_OK;
}

struct usbd_cdc_callback_t cdc_cb = 
{
    get_line_coding,
    set_line_coding,
    cdc_data_received,
};

void usbd_cdc_test(struct usbd_t *h)
{
    usbd_cdc_set_cb(&cdc_cb);
    usbd_cdc_init(h);
}
