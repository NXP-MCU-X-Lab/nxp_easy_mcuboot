#ifndef __CHLIB_USB_COMMON_H_
#define	__CHLIB_USB_COMMON_H_

#include <stdint.h>
#include <stdbool.h>

#if !defined(CH_OK)
#define CH_OK       (0)
#endif

#if !defined(CH_ERR)
#define CH_ERR      (1)
#endif

#define USB_CLASS_DEVICE                0x00
#define USB_CLASS_AUDIO                 0x01
#define USB_CLASS_CDC                   0x02
#define USB_CLASS_HID                   0x03
#define USB_CLASS_PHYSICAL              0x05
#define USB_CLASS_IMAGE                 0x06
#define USB_CLASS_PRINTER               0x07
#define USB_CLASS_MASS_STORAGE          0x08
#define USB_CLASS_HUB                   0x09
#define USB_CLASS_CDC_DATA              0x0a
#define USB_CLASS_SMART_CARD            0x0b
#define USB_CLASS_SECURITY              0x0d
#define USB_CLASS_VIDEO                 0x0e
#define USB_CLASS_HEALTHCARE            0x0f
#define USB_CLASS_DIAG_DEVICE           0xdc
#define USB_CLASS_WIRELESS              0xe0
#define USB_CLASS_MISC                  0xef
#define USB_CLASS_APP_SPECIFIC          0xfe
#define USB_CLASS_VEND_SPECIFIC         0xff

#define USB_REQ_TYPE_STANDARD           0x00
#define USB_REQ_TYPE_CLASS              0x20
#define USB_REQ_TYPE_VENDOR             0x40
#define USB_REQ_TYPE_MASK               0x60

#define USB_REQ_TYPE_DIR_OUT            0x00
#define USB_REQ_TYPE_DIR_IN             0x80

#define USB_REQ_TYPE_DEVICE             0x00
#define USB_REQ_TYPE_INTERFACE          0x01
#define USB_REQ_TYPE_ENDPOINT           0x02
#define USB_REQ_TYPE_OTHER              0x03
#define USB_REQ_TYPE_RECIPIENT_MASK     0x1f

#define USB_REQ_GET_STATUS              0x00
#define USB_REQ_CLEAR_FEATURE           0x01
#define USB_REQ_SET_FEATURE             0x03
#define USB_REQ_SET_ADDRESS             0x05
#define USB_REQ_GET_DESCRIPTOR          0x06
#define USB_REQ_SET_DESCRIPTOR          0x07
#define USB_REQ_GET_CONFIGURATION       0x08
#define USB_REQ_SET_CONFIGURATION       0x09
#define USB_REQ_GET_INTERFACE           0x0A
#define USB_REQ_SET_INTERFACE           0x0B
#define USB_REQ_SYNCH_FRAME             0x0C
#define USB_REQ_SET_ENCRYPTION          0x0D
#define USB_REQ_GET_ENCRYPTION          0x0E
#define USB_REQ_RPIPE_ABORT             0x0E
#define USB_REQ_SET_HANDSHAKE           0x0F
#define USB_REQ_RPIPE_RESET             0x0F
#define USB_REQ_GET_HANDSHAKE           0x10
#define USB_REQ_SET_CONNECTION          0x11
#define USB_REQ_SET_SECURITY_DATA       0x12
#define USB_REQ_GET_SECURITY_DATA       0x13
#define USB_REQ_SET_WUSB_DATA           0x14
#define USB_REQ_LOOPBACK_DATA_WRITE     0x15
#define USB_REQ_LOOPBACK_DATA_READ      0x16
#define USB_REQ_SET_INTERFACE_DS        0x17


#define USB_DESC_LENGTH_DEVICE          0x12
#define USB_DESC_LENGTH_CONFIG          0x9
#define USB_DESC_LENGTH_IAD             0x8
#define USB_DESC_LENGTH_STRING          0x4
#define USB_DESC_LENGTH_INTERFACE       0x9
#define USB_DESC_LENGTH_ENDPOINT        0x7

#define USB_DESC_TYPE_DEVICE            0x01
#define USB_DESC_TYPE_CONFIGURATION     0x02
#define USB_DESC_TYPE_STRING            0x03
#define USB_DESC_TYPE_INTERFACE         0x04
#define USB_DESC_TYPE_ENDPOINT          0x05
#define USB_DESC_TYPE_DEVICEQUALIFIER   0x06
#define USB_DESC_TYPE_OTHERSPEED        0x07
#define USB_DESC_TYPE_IAD               0x0b
#define USB_DESC_TYPE_HID               0x21
#define USB_DESC_TYPE_REPORT            0x22
#define USB_DESC_TYPE_PHYSICAL          0x23
#define USB_DESC_TYPE_HUB               0x29



/* bRequest: USB Standard Request Codes */
#define USB_REQUEST_GET_STATUS                 (0)
#define USB_REQUEST_CLEAR_FEATURE              (1)
#define USB_REQUEST_SET_FEATURE                (3)
#define USB_REQUEST_SET_ADDRESS                (5)
#define USB_REQUEST_GET_DESCRIPTOR             (6)
#define USB_REQUEST_SET_DESCRIPTOR             (7)
#define USB_REQUEST_GET_CONFIGURATION          (8)
#define USB_REQUEST_SET_CONFIGURATION          (9)
#define USB_REQUEST_GET_INTERFACE              (10)
#define USB_REQUEST_SET_INTERFACE              (11)
#define USB_REQUEST_SYNC_FRAME                 (12)


#define WBVAL(x)                          (x & 0xFF),((x >> 8) & 0xFF)

/* USB Specification Release Number */
#define USB_VERSION_2_0 (0x0200)
#define USB_VERSION_1_1 (0x0110)

typedef struct 
{
    uint32_t tag;
    const char* name;
    const uint8_t *buf;
    uint16_t len;
}desc_t;

struct udevice_descriptor
{
    uint8_t bLength;
    uint8_t type;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
};

struct uconfig_descriptor
{
    uint8_t bLength;
    uint8_t type;
    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t MaxPower;
    uint8_t data[256];
};
typedef struct uconfig_descriptor* ucfg_desc_t;

struct usb_qualifier_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint16_t bcdUSB; // TODO: big-endian.
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint8_t  bNumConfigurations;
    uint8_t  bRESERVED;
} __attribute__ ((packed));

struct ustring_descriptor
{
    uint8_t bLength;
    uint8_t type;
    uint8_t String[64];
};
typedef struct ustring_descriptor* ustr_desc_t;

struct uinterface_descriptor
{
    uint8_t bLength;
    uint8_t type;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
};
typedef struct uinterface_descriptor* uintf_desc_t;

/* Interface Association Descriptor (IAD) */
struct uiad_descriptor
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bFirstInterface;
    uint8_t bInterfaceCount;
    uint8_t bFunctionClass;
    uint8_t bFunctionSubClass;
    uint8_t bFunctionProtocol;
    uint8_t iFunction;
};
typedef struct uiad_descriptor* uiad_desc_t;

struct uendpoint_descriptor
{
    uint8_t bLength;
    uint8_t type;
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterval;
};
typedef struct uendpoint_descriptor* uep_desc_t;

/* usb stanrd request */
struct urequest
{
    uint8_t request_type;
    uint8_t request;
    uint16_t value;
    uint16_t index;
    uint16_t length;
};

struct ucdc_line_coding
{
    uint32_t dwDTERate;
    uint8_t bCharFormat;
    uint8_t bParityType;
    uint8_t bDataBits;
};

//! @brief Computes the number of elements in an array.
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))
#endif

/*
 * the define related to mass storage
 */
#define USBREQ_GET_MAX_LUN              0xfe
#define USBREQ_MASS_STORAGE_RESET       0xff

#define SIZEOF_CSW                      0x0d
#define SIZEOF_CBW                      0x1f
#define SIZEOF_INQUIRY_CMD              0x24
#define SIZEOF_MODE_SENSE_6             0x4
#define SIZEOF_READ_CAPACITIES          0xc
#define SIZEOF_READ_CAPACITY            0x8
#define SIZEOF_REQUEST_SENSE            0x12

#define CBWFLAGS_DIR_M                  0x80
#define CBWFLAGS_DIR_IN                 0x80
#define CBWFLAGS_DIR_OUT                0x00

#define SCSI_TEST_UNIT_READY            0x00
#define SCSI_REQUEST_SENSE              0x03
#define SCSI_INQUIRY_CMD                0x12
#define SCSI_ALLOW_REMOVAL              0x1e
#define SCSI_MODE_SENSE_6               0x1a
#define SCSI_START_STOP                 0x1b
#define SCSI_READ_CAPACITIES            0x23
#define SCSI_READ_CAPACITY              0x25
#define SCSI_READ_10                    0x28
#define SCSI_WRITE_10                   0x2a
#define SCSI_VERIFY_10                  0x2f

#define CBW_SIGNATURE                   0x43425355
#define CSW_SIGNATURE                   0x53425355
#define CBW_TAG_VALUE                   0x12345678

struct ustorage_cbw
{
    uint32_t signature;
    uint32_t tag;
    uint32_t xfer_len;
    uint8_t dflags;
    uint8_t lun;
    uint8_t cb_len;
    uint8_t cb[16];
};
typedef struct ustorage_cbw* ustorage_cbw_t;

struct ustorage_csw
{
    uint32_t signature;
    uint32_t tag;
    int32_t data_reside;
    uint8_t  status;
};
typedef struct ustorage_csw* ustorage_csw_t;

#endif
