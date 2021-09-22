#ifndef __USBD_MSC_H_
#define	__USBD_MSC_H_

#include <stdint.h>
#include <usbd.h>
#include <usb_common.h>


struct usbd_msc_callback_t
{
    uint32_t (*msc_read_sector) (uint32_t block, uint8_t *buf, uint32_t block_cnt);
    uint32_t (*msc_write_sector)(uint32_t block, uint8_t *buf, uint32_t block_cnt);
    uint32_t (*msc_get_disk_info)(uint32_t *total_block_cnt, uint32_t *block_size);
};

void usbd_msc_set_cb(struct usbd_msc_callback_t *cb);
void usbd_msc_init(struct usbd_t *h);
uint32_t msc_standard_request_to_intf_handler(struct usbd_t *h);
uint32_t msc_class_request_handler(struct usbd_t *h);
uint32_t msc_data_ep_handler(uint8_t ep, uint8_t dir);

#endif






