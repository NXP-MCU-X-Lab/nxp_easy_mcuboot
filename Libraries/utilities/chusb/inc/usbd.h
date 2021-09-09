#ifndef __CHLIB_USBD_H_
#define	__CHLIB_USBD_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "usb_common.h"
#include "usbd_config.h"

#if ( USBD_DEBUG == 1 )
#include <stdio.h>
#endif

#if ( USBD_DEBUG == 1 )
#define USBD_TRACE	printf
#else
#define USBD_TRACE(...)
#endif

#define USBD_USE_HID0       (1 << USBD_HID0_IF_IDX)
#define USBD_USE_HID1       (1 << USBD_HID1_IF_IDX)
#define USBD_USE_HID2       (1 << USBD_HID2_IF_IDX)
#define USBD_USE_CDC        (1 << USBD_CDC_CIF_IDX)
#define USBD_USE_MSC        (1 << USBD_MSC_IF_IDX)


struct usbd_ops_t
{
    uint32_t (*ep_read)(uint8_t ep, uint8_t *buf, uint8_t data01);
    uint32_t (*ep_write)(uint8_t ep, uint8_t *buf, uint32_t len, uint8_t data01);
    uint32_t (*ep_config)(struct uendpoint_descriptor* d);
    uint32_t (*ep_clear_feature)(uint8_t ep);
    uint32_t (*set_addr)(uint8_t addr);
};

struct usbd_t
{
    uint8_t* ep0_in_buf;
    uint32_t ep0_in_remain_size;
    uint8_t ep0_out_buf[64];
    uint32_t ep0_out_remain_size;
    uint8_t *ep0_out_ptr;
    
    struct usbd_ops_t *ops;
    struct urequest setup;
    uint8_t addr;
    uint32_t data01_in;                             /* data toggle status for ep_write */
    uint32_t data01_out;                            /* data toggle status for ep_read */
    bool need_zlp;                                  /* need to send zero lengh IN packet */
    bool is_configured;
    uint32_t (*class_request_handler)(struct usbd_t *h);
    uint32_t (*standard_request_to_intf_handler)(struct usbd_t *h);
    uint32_t (*setup_out_data_received_handler)(uint8_t *buf, uint32_t len);
    uint32_t (*data_ep_handler)(uint8_t ep, uint8_t dir);
    uint32_t (*vender_request_handler)(struct usbd_t *h);
};


/* API */
void usbd_init(struct usbd_t *dev, struct usbd_ops_t *ops);
uint32_t usbd_ep_read(uint8_t ep, uint8_t *buf);
uint32_t usbd_ep_write(uint8_t ep, uint8_t *buf, uint32_t len);
uint32_t usbd_ep0_setup_handler(void);
uint32_t usbd_ep0_in_handler(void);
uint32_t usbd_ep0_out_handler(void);
void begin_data_in_stage(uint8_t *buf, uint32_t len);
void usbd_data_ep_handler(uint8_t ep, uint8_t dir);
void usbd_status_in_stage(void);
void usbd_status_out_stage(void);
void usbd_stack_reset(void);
bool usbd_is_configured(void);
uint32_t get_descriptor_data(const char *name, desc_t *d);
void create_string_descriptor(struct ustring_descriptor *d, uint8_t *buf, uint32_t len);

#endif
