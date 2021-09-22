#ifndef __USBD_COMPOSITE_H_
#define	__USBD_COMPOSITE_H_

#include <stdint.h>
#include <usbd.h>
#include <usbd_cdc.h>

void usbd_composite_init(struct usbd_t *h, uint32_t option);

#endif


