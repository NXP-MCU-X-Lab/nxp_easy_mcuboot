#ifndef __USBD_cdc_H_
#define	__USBD_cdc_H_

#include <stdint.h>
#include <usbd.h>
#include <usb_common.h>


struct usbd_cdc_callback_t
{
    uint32_t (*get_line_coding)(struct ucdc_line_coding *line_coding);
    uint32_t (*set_line_coding)(struct ucdc_line_coding *line_coding);
    uint32_t (*set_control_line_serial_state)(uint8_t val);
    uint32_t (*recv_handler)(uint8_t *buf, uint32_t len);
    uint32_t (*send_notify)(void);
};

void usbd_cdc_init(struct usbd_t *h);
uint32_t usbd_cdc_send(uint8_t *buf, uint32_t len);
void usbd_cdc_set_cb(struct usbd_cdc_callback_t *cb);
uint32_t cdc_class_request_handler(struct usbd_t *h);
uint32_t cdc_standard_request_to_intf_handler(struct usbd_t *h);
uint32_t cdc_data_ep_handler(uint8_t ep, uint8_t dir);
uint32_t cdc_setup_out_data_received(uint8_t *buf, uint32_t len);

#endif

