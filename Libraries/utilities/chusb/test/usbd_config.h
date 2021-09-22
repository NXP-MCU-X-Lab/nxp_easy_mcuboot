#ifndef __CHLIB_USBD_CONFIG_H_
#define	__CHLIB_USBD_CONFIG_H_

/* general USBD configration */
#define EP0_MAX_SIZE                    (64)
#define USBD_VID                        (0x0465)
#define USBD_PID                        (0x622F)
#define MANUFACTURE_STR                 L"HIPNUC"
#define PRODUCT_STR                     L"USBTEST"
#define SERIAL_STR                      L"0123"
#define USBD_DEBUG                      (0)


/* intf defination */
#define USBD_MSC_IF_IDX     (0)
#define USBD_MSC_IF_STR     L"MSC"
#define MSD_BUF_SIZE        (1024*4)
#define MSD_EP_SIZE         (64)

#define USBD_CDC_CIF_IDX    (1)
#define USBD_CDC_CIF_STR    L"CDC CIF"
#define USBD_CDC_DIF_IDX    (2)
#define USBD_CDC_DIF_STR    L"CDC_DIF"
#define CDC_EP_SIZE         (64)

#define USBD_HID0_IF_IDX    (3)
#define USBD_HID0_IF_STR    L"HID0"

#define USBD_HID1_IF_IDX    (4)
#define USBD_HID1_IF_STR    L"HID1"

#define USBD_HID2_IF_IDX    (5)
#define USBD_HID2_IF_STR    L"CMSIS-DAP"
#define USBD_HID_CUSTOM_REPORT_SIZE     (64)
#define USBD_HID_EP_INTERVAL            (1)

#define USBD_IF_STR_IDX(IF_NUM)     (4 + IF_NUM)

/* ep defination */
#define USBD_HID0_EP_INTIN              (3)
#define USBD_HID0_EP_INTOUT             (3)
#define USBD_HID1_EP_INTIN              (8)
#define USBD_HID1_EP_INTOUT             (8)
#define USBD_HID2_EP_INTIN              (1)
#define USBD_HID2_EP_INTOUT             (1)
#define USBD_CDC_ACM_EP_INTIN           (3)
#define USBD_CDC_ACM_EP_BULKIN          (4)
#define USBD_CDC_ACM_EP_BULKOUT         (4)
#define USBD_MSC_EP_BULKIN              (2)
#define USBD_MSC_EP_BULKOUT             (2)

#endif
