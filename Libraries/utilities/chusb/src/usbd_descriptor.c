#include <string.h>
#include "usbd.h"
#include "usb_common.h"
#include "usbd_hid.h"

static uint8_t device_descriptor[] =
{
    USB_DESC_LENGTH_DEVICE,             /* bLength */
    USB_DESC_TYPE_DEVICE,               /* bDescriptorType */
    WBVAL(USB_VERSION_2_0),               /* USB2.00 */

    0x00,                                 /* bDeviceClass */
    0x00,                                 /* bDeviceSubClass */
    0x00,                                 /* bDeviceProtocol */

    EP0_MAX_SIZE,                         /* bMaxPacketSize0 */
    WBVAL(USBD_VID),                      /* idVendor 供应商ID（VID） */
    WBVAL(USBD_PID),                      /* idProduct  make sure different configuration use different PID */
    WBVAL(0x0100),                        /* bcdDevice 设备版本号（采用BCD码） */
    0x01,                                 /* iManufacturer  供应商的字符串描述符索引 */
    0x02,                                 /* iProduct  产品的字符串描述符索引 */
    0x03,                                 /* iSerialNumber 设备序号的字符串描述符索引 */
    0x01                                  /* bNumConfigurations: one possible configuration 该USB设备支持的配置数目*/
};

const uint8_t usb_qualifier_descriptor[] = 
{
    10,                                     /* bLength */
    USB_DESC_TYPE_DEVICEQUALIFIER,          /* bDescriptorType */
    WBVAL(0x0100),                          /* bcdUSB */
    0x00,                                   /* bDeviceClass */
    0x00,                                   /* bDeviceSubClass */
    0x00,                                   /* bDeviceProtocol */
    EP0_MAX_SIZE,                           /* bMaxPacketSize0 */
    0x01,                                   /* bNumConfigurations */
    0x00                                    /* bReserved */
};


static struct uconfig_descriptor configuration_descriptor;

const uint8_t cdc_acm_iad_descriptor[] = 
{
    /* interface association descriptor for CDC */
    USB_DESC_LENGTH_IAD,
    USB_DESC_TYPE_IAD,
    USBD_CDC_CIF_IDX,       /* bFirstInterface */
    0x02,                   /* bInterfaceCount */
    USB_CLASS_CDC,
    0x02,
    0x01,
    0x00,
};
    
const uint8_t cdc_acm_if0_descriptor[] = 
{
    /* CDC类接口描述符 */
    USB_DESC_LENGTH_INTERFACE,                  /* bLength字段。接口描述符的长度为9字节 */
    USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType字段。接口描述符的编号为0x04。 */
	USBD_CDC_CIF_IDX,                           /* bInterfaceNumber域，接口号 */
	0x00,                                       /* bAlternateSetting域，接口的可替换设置值 不需要改动 */
	0x01,                                       /* bNumEndpoints域，非0端点的数目。CDC接口只使用一个中断 */
    USB_CLASS_CDC,                              /* bInterfaceClass字段。该接口所使用的类。CDC类的类代码为0x02。 */
    0x02,                                       /* bInterfaceSubClass字段。该接口所使用的子类。要实现USB转串口， 就必须使用Abstract Control Model（抽象控制模型）子类。它的编号为0x02。 */
    0x01,                                       /* bInterfaceProtocol字段。使用Common AT Commands（通用AT命令） 协议。该协议的编号为0x01。 */
    USBD_IF_STR_IDX(USBD_CDC_CIF_IDX),
    
    /* Header Functional Descriptor */
    0x05,                                       /* bFunctionLength字段。该描述符长度为5字节 */
    0x24,                                       /* bDescriptorType字段。描述符类型为类特殊接口（CS_INTERFACE） 编号为0x24。*/
    0x00,                                       /* bDescriptorSubtype字段。描述符子类为Header Functional Descriptor 编号为0x00。 */
    WBVAL(USB_VERSION_2_0),                     /* USB2.00 */
    
    /* Call Management Functional Descriptor */
    0x05,                                       /* bFunctionLength字段。该描述符长度为5字节 */
    0x24,                                       /* bDescriptorType字段。描述符类型为类特殊接口（CS_INTERFACE） 编号为0x24。 */
    0x01,                                       /* bDescriptorSubtype字段。描述符子类为Call Management functional descriptor，编号为0x01。*/
    0x01,                                       /* bmCapabilities字段。设备自己不管理call management */
    0x01,                                       /* bDataInterface字段。没有数据类接口用作call management */
    
    /* Abstract Control Management Functional Descriptor */
    0x04,                                       /* bFunctionLength字段。该描述符长度为4字节 */
    0x24,                                       /* bDescriptorType字段。描述符类型为类特殊接口（CS_INTERFACE） 编号为0x24。*/
    0x02,                                       /* bDescriptorSubtype字段。描述符子类为Abstract Control Management functional descriptor，编号为0x02。*/
    0x06,                                       /* bmCapabilities字段。支持Set_Line_Coding、Set_Control_Line_State、 Get_Line_Coding请求和Serial_State通知 */

    /* Union Functional Descriptor */
    0x05,                                       /* bFunctionLength字段。该描述符长度为5字节。  */
    0x24,                                       /* bDescriptorType字段。描述符类型为类特殊接口（CS_INTERFACE） 编号为0x24 */
    0x06,                                       /* bDescriptorSubtype字段。描述符子类为 Union functional descriptor，编号为0x06。 */
    0x00,                                       /* MasterInterface字段。这里为前面编号为0的CDC接口。 */
    0x01,                                       /* SlaveInterface字段，这里为接下来编号为1的数据类接口。*/
    
	/* 端点描述符 */
	0x07,                                       /* bLength域，端点描述符长度：7字节 */
	USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType域：0x05表示本描述符为端点描述符 */
	(0x80 | USBD_CDC_ACM_EP_INTIN),             /* bEndpointAddress域，端点号和传输方向：端点 [bit7]=0 是output */
	0x03,                                       /* bmAttributes域，端点特性：中断传输 */
    WBVAL(64),                                  /* wMaxPacketSize域，端点支持最大数据包长度 */
	0x0A,                                       /* bInterval域，轮询间隔，以ms为单位。*/
};


const uint8_t cdc_acm_if1_descriptor[] = 
{
    /*  以下为接口1（数据接口）的接口描述符  */
    USB_DESC_LENGTH_INTERFACE,
    USB_DESC_TYPE_INTERFACE,
    USBD_CDC_DIF_IDX,                           /* bInterfaceNumber字段。该接口的编号，第二个接口，编号为1。*/
    0x00,                                       /* bAlternateSetting字段。该接口的备用编号，为0。 */
    0x02,                                       /* bNumEndpoints字段。非0端点的数目。该设备需要使用一对批量端点，设置为2。*/
    0x0A,                                       /* bInterfaceClass字段。该接口所使用的类。数据类接口的代码为0x0A。*/
    0x00,                                       /* bInterfaceSubClass字段。该接口所使用的子类为0。*/
    0x00,                                       /* bInterfaceProtocol字段。该接口所使用的协议为0。*/
    USBD_IF_STR_IDX(USBD_CDC_DIF_IDX),
    
	/* 端点描述符 */
	0x07,                                       /* bLength域，端点描述符长度：7字节 */
	USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType域：0x05表示本描述符为端点描述符 */
	(0x80 | USBD_CDC_ACM_EP_BULKIN),            /* bEndpointAddress域，端点号和传输方向：端点 [bit7]=0 是output */
	0x02,                                       /* bmAttributes域，端点特性：批量端点的编号为0x02 */
	WBVAL(CDC_EP_SIZE),                         /* wMaxPacketSize域，端点支持最大数据包长度 */
	0x00,                                       /* bInterval字段。端点查询的时间，这里对批量端点无效。*/
    
	/* 端点描述符 */
	0x07,                                       /* bLength域，端点描述符长度：7字节 */
	USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType域：0x05表示本描述符为端点描述符 */
	(0x00 | USBD_CDC_ACM_EP_BULKOUT),           /* bEndpointAddress域，端点号和传输方向：端点 [bit7]=0 是output */
	0x02,                                       /* bmAttributes域，端点特性：批量端点的编号为0x02 */
	WBVAL(CDC_EP_SIZE),                         /* wMaxPacketSize域，端点支持最大数据包长度 */
	0x00,                                       /* bInterval字段。端点查询的时间，这里对批量端点无效。*/
};

static const uint8_t report_descriptor_keyboard[] = {
    HID_UsagePage(HID_USAGE_PAGE_GENERIC), /* Usage Page (Generic Desktop)*/
    HID_Usage(HID_USAGE_GENERIC_KEYBOARD), /* Usage (Keyboard) */
    HID_Collection(HID_Application),
        HID_ReportSize(1),                         /* Report Size (1U) */
        HID_ReportCount(8),  
        HID_UsagePage(HID_USAGE_PAGE_KEYBOARD), /* Usage Page (Key Codes) */
        HID_UsageMin(0xE0), /* Usage Minimum (224U) */
        HID_UsageMax(0xE7), /* Usage Maximum (231U) */
        HID_LogicalMin(0),
        HID_LogicalMax(1),
        HID_Input(HID_Data | HID_Variable | HID_Relative), /* Input(Data, Variable, Absolute) Modifier byte */

        HID_ReportSize(8),
        HID_ReportCount(1),  
        HID_Input(HID_Constant),
    
        HID_ReportSize(1),
        HID_ReportCount(5),  
        HID_UsagePage(HID_USAGE_PAGE_GENERIC), /* Usage Page (Page# for LEDs) */
        HID_UsageMin(1), /* Usage Minimum (224U) */
        HID_UsageMax(5), /* Usage Maximum (231U) */
        HID_Output(HID_Data | HID_Variable | HID_Relative),
        HID_ReportSize(1),
        HID_ReportCount(3), 
        HID_Output(HID_Constant), /* Output (Constant), LED report padding */

        HID_ReportSize(8),
        HID_ReportCount(6), 
        HID_LogicalMin(0x00),
        HID_LogicalMax(0xFF),
        HID_UsagePage(HID_USAGE_PAGE_KEYBOARD), /* Usage Page (Key Codes) */
        HID_UsageMin(0x00),
        HID_UsageMax(0xFF),
        HID_Input(HID_Data),
    HID_EndCollection
};

const uint8_t hid0_descriptor[] =
{
	/* interface descritor INDEX:0 */
	USB_DESC_LENGTH_INTERFACE,                      /* bLength域，接口描述符长度：9字节 */
	USB_DESC_TYPE_INTERFACE,                        /* bDescriptorType域：0x04表示本描述符为接口描述符 */
	USBD_HID0_IF_IDX,                               /* bInterfaceNumber域，接口号 */
	0x00,                                           /* bAlternateSetting域，接口的可替换设置值 不需要改动 */
	0x02,                                           /* bNumEndpoints域，接口使用的端点数（除端点0 */
	USB_CLASS_HID,                                  /* bInterfaceClass域，接口所属的USB设备类：0xFF表示供应商自定义 */
	HID_SUBCLASS_NONE,                              /* bInterfaceSubClass域，接口所属的USB设备子类：0xFF表示供应商自定义 */
	HID_PROTOCOL_NONE,                              /* bInterfaceProtocol域，接口采用的USB设备类协议：0xFF表示供应商自定义 */
	USBD_IF_STR_IDX(USBD_HID0_IF_IDX),              /* iInterface域，接口字符串描述符的索引：0 */
           
    /* HID descritor */
    0x09,                                       /* bLength字段。本HID描述符下只有一个下级描述符。所以长度为9字节 */
    0x21,                                       /* bDescriptorType字段。HID描述符的编号为 0x21 */
    WBVAL(0x0100),                              /* bcdHID */ 
    0x21,                                       /* bCountyCode字段。设备适用的国家代码，这里选择为美国，代码0x21。 */
    0x01,                                       /* bNumDescriptors字段。下级描述符的数目。我们只有一个报告描述符 */
    0x22,                                       /* bDescriptorType字段。下级描述符的类型，为报告描述符，编号为0x22 */
    WBVAL(sizeof(report_descriptor_keyboard)),  /* bDescriptorLength字段。下级描述符的长度。下级描述符为报告描述符。 */
  
	/* endpoint descritor */
	0x07,                                           /* bLength域，端点描述符长度：7字节 */
	USB_DESC_TYPE_ENDPOINT,                         /* bDescriptorType域：0x05表示本描述符为端点描述符 */
	(0x80 | USBD_HID0_EP_INTIN),            /* bEndpointAddress域，端点号和传输方向：端点 [bit7]=0 是output */
	0x03,                                           /* bmAttributes域，端点特性：中断传输 */
	WBVAL(64),                                      /* wMaxPacketSize域，端点支持最大数据包长度 */
	USBD_HID_EP_INTERVAL,                           /* bInterval域，轮询间隔，以ms为单位。*/
   
	0x07,                                           /* bLength域，端点描述符长度：7字节 */
	USB_DESC_TYPE_ENDPOINT,                         /* bDescriptorType域：0x05表示本描述符为端点描述符 */
	(0x00 | USBD_HID0_EP_INTOUT),           /* bEndpointAddress域，端点号和传输方向：端点 [bit7]=0 是output */
	0x03,                                           /* bmAttributes域，端点特性：中断传输 */
	WBVAL(64),                                      /* wMaxPacketSize域，端点支持最大数据包长度 */
	USBD_HID_EP_INTERVAL,                           /* bInterval域，轮询间隔，以ms为单位。*/
};

static const uint8_t report_descriptor_mouse[] = {
  HID_UsagePage      ( HID_USAGE_PAGE_GENERIC                 ),
  HID_Usage          ( HID_USAGE_GENERIC_MOUSE                ),
  HID_Collection     ( HID_Application                        ),
    HID_Usage        ( HID_USAGE_GENERIC_POINTER              ),
    HID_Collection   ( HID_Physical                           ),
      HID_UsagePage  ( HID_USAGE_PAGE_BUTTON                  ),
      HID_UsageMin   ( 1                                      ),
      HID_UsageMax   ( 3                                      ),
      HID_LogicalMin ( 0                                      ),
      HID_LogicalMax ( 1                                      ),
      HID_ReportCount( 3                                      ),
      HID_ReportSize ( 1                                      ),
      HID_Input      ( HID_Data | HID_Variable | HID_Absolute ),
      HID_ReportCount( 1                                      ),
      HID_ReportSize ( 5                                      ),
      HID_Input      ( HID_Constant                           ),
      HID_UsagePage  ( HID_USAGE_PAGE_GENERIC                 ),
      HID_Usage      ( HID_USAGE_GENERIC_X                    ),
      HID_Usage      ( HID_USAGE_GENERIC_Y                    ),
      HID_Usage      ( HID_USAGE_GENERIC_WHEEL                ),
      HID_LogicalMin ( (uint8_t)-127                          ),
      HID_LogicalMax ( 127                                    ),
      HID_ReportSize ( 8                                      ),
      HID_ReportCount( 3                                      ),
      HID_Input      ( HID_Data | HID_Variable | HID_Relative ),
    HID_EndCollection,
  HID_EndCollection,
};
    
//static const uint8_t report_descriptor_custom[] = {
//    HID_UsagePageVendor(0x00),                  /* Global Item: Usage Page, General Desktop */
//    HID_Usage(0x00),
//    HID_Collection(0x01),                           /* 集合(Application) */
//        HID_ReportSize(8),                          /* Report Size(8 bit)*/
//#if (USBD_HID_CUSTOM_REPORT_SIZE >= 256)
//    HID_ReportCountS(USBD_HID_CUSTOM_REPORT_SIZE),      /* Report Count: byte0 used for ReportID, so it's  USBD_HID_REPORT_SIZE - 1*/
//#else
//    HID_ReportCount(USBD_HID_CUSTOM_REPORT_SIZE),      /* Report Count: byte0 used for ReportID, so it's  USBD_HID_REPORT_SIZE - 1*/
//#endif

//        HID_Usage(0x00),
//        HID_Input(0x02),                            /* 输入(data, variable, absolute) */
//    
//        HID_Usage(0x00),
//        HID_Output(0x02),                           /* 输出(data, variable, absolute) */
//    HID_EndCollection,                              /* 集合结束(Application) */
//};

//static const uint8_t report_descriptor_custom[] = {
//  HID_UsagePageVendor( 0x00                      ),
//  HID_Usage          ( 0x01                      ),
//  HID_Collection     ( HID_Application           ),
//    HID_LogicalMin   ( 0                         ), /* value range: 0 - 0xFF */
//    HID_LogicalMaxS  ( 0xFF                      ),
//    HID_ReportSize   ( 8                         ), /* 8 bits */
//    HID_ReportCount  ( 64  ),
//    HID_Usage        ( 0x01                      ),
//    HID_Input        ( HID_Data | HID_Variable | HID_Absolute ),
//    HID_ReportCount  ( 64 ),
//    HID_Usage        ( 0x01                      ),
//    HID_Output       ( HID_Data | HID_Variable | HID_Absolute ),
//    HID_ReportCount  ( 64),
//    HID_Usage        ( 0x01                      ),
//    HID_Feature      ( HID_Data | HID_Variable | HID_Absolute ),
//  HID_EndCollection,
//};

/* MCUBOOT HID report descsritors */
enum _hid_report_ids
{
    kBootloaderReportID_CommandOut = 1,
    kBootloaderReportID_DataOut = 2,
    kBootloaderReportID_CommandIn = 3,
    kBootloaderReportID_DataIn = 4
};

#define BL_FS_MAX_PACKET_SIZE (60)
#define BL_PACKET_SIZE_HEADER_SIZE (3) // alignment byte + length lsb + length msb (does not include report id)
#define BL_FS_ACTUAL_REPORT_SIZE (BL_FS_MAX_PACKET_SIZE + BL_PACKET_SIZE_HEADER_SIZE)
#define BL_FS_REPORT_SIZE BL_FS_ACTUAL_REPORT_SIZE

#define BL_CONFIG_REPORT_SIZE_BASE (8)
#define BL_FS_CONFIG_REPORT_SIZE (BL_CONFIG_REPORT_SIZE_BASE)

/* hidtc data buffer out report descriptor */
#define HID_USAGE_HIDTC_DATA_OUT(__id, __count, __size)          \
    0x85, ((uint8_t)(__id)),        /*	 REPORT_ID (__id) */      \
        0x19, 0x01,                 /*   USAGE_MINIMUM (1)*/     \
        0x29, 0x01,                 /*   USAGE_MAXIMUM (1)*/     \
        0x15, 0x00,                 /*   LOGICAL_MINIMUM (0)*/   \
        0x26, 0xff, 0x00,           /*   LOGICAL_MAXIMUM (255)*/ \
        0x75, ((uint8_t)(__size)),  /*	 REPORT_SIZE (n)*/        \
        0x95, ((uint8_t)(__count)), /*	 REPORT_COUNT (n)*/       \
        0x91, 0x02                  /*   OUTPUT (Data,Var,Abs) */

/* hidtc data buffer in report descriptor */
#define HID_USAGE_HIDTC_DATA_IN(__id, __count, __size)           \
    0x85, ((uint8_t)(__id)),        /*	 REPORT_ID (__id) */      \
        0x19, 0x01,                 /*   USAGE_MINIMUM (1)*/     \
        0x29, 0x01,                 /*   USAGE_MAXIMUM (1)*/     \
        0x15, 0x00,                 /*   LOGICAL_MINIMUM (0)*/   \
        0x26, 0xff, 0x00,           /*   LOGICAL_MAXIMUM (255)*/ \
        0x75, ((uint8_t)(__size)),  /*	 REPORT_SIZE (n)*/        \
        0x95, ((uint8_t)(__count)), /*	 REPORT_COUNT (n)*/       \
        0x81, 0x02                  /*   INPUT (Data,Var,Abs) */


static const uint8_t report_descriptor_custom[] = {
    0x06,
    0x00,
    0xFF, /* Usage Page (Vendor Defined Page 1)*/
    0x09,
    0x01, /* USAGE (Vendor 1) */
    0xA1,
    0x01, /* Collection (Application) */
    HID_USAGE_HIDTC_DATA_OUT(kBootloaderReportID_CommandOut, BL_FS_REPORT_SIZE, BL_FS_CONFIG_REPORT_SIZE),
    HID_USAGE_HIDTC_DATA_OUT(kBootloaderReportID_DataOut, BL_FS_REPORT_SIZE, BL_FS_CONFIG_REPORT_SIZE),
    HID_USAGE_HIDTC_DATA_IN(kBootloaderReportID_CommandIn, BL_FS_REPORT_SIZE, BL_FS_CONFIG_REPORT_SIZE),
    HID_USAGE_HIDTC_DATA_IN(kBootloaderReportID_DataIn, BL_FS_REPORT_SIZE, BL_FS_CONFIG_REPORT_SIZE),
    0xC0 /* end collection */
};

/* mouse */
const uint8_t hid1_descriptor[] =
{
	/* interface descritor INDEX:1 */
	USB_DESC_LENGTH_INTERFACE,
	USB_DESC_TYPE_INTERFACE,
	USBD_HID1_IF_IDX,
	0x00,
	0x02,
	USB_CLASS_HID,
	HID_SUBCLASS_NONE,
	HID_PROTOCOL_MOUSE,
	USBD_IF_STR_IDX(USBD_HID1_IF_IDX),
           
    /* HID描述符 */
    0x09,
    0x21,
    WBVAL(0x0100),
    0x21,
    0x01,
    0x22,
    WBVAL(sizeof(report_descriptor_mouse)),
  
	/* 端点描述符 */
	0x07,
	USB_DESC_TYPE_ENDPOINT,
	(0x80 | USBD_HID1_EP_INTIN),
	0x03,
	WBVAL(64),
	USBD_HID_EP_INTERVAL,
   
	0x07,
	USB_DESC_TYPE_ENDPOINT,
	(0x00 | USBD_HID1_EP_INTOUT),
	0x03,
	WBVAL(64),
	USBD_HID_EP_INTERVAL,
};

/* custom hid device */
const uint8_t hid2_descriptor[] =
{
	/* interface descritor INDEX:2 */
    USB_DESC_LENGTH_INTERFACE,
    USB_DESC_TYPE_INTERFACE,
    USBD_HID0_IF_IDX,
    0x00,
    0x02,
	USB_CLASS_HID,
	HID_SUBCLASS_NONE,
	HID_PROTOCOL_NONE,
	USBD_IF_STR_IDX(USBD_HID0_IF_IDX),
           
    /* HID descriptor */
    0x09,
    0x21,
    WBVAL(0x0100),
    0x00,
    0x01,
    0x22,
    WBVAL(sizeof(report_descriptor_custom)),
  
	/* endpoint descriptor */
	0x07,
	USB_DESC_TYPE_ENDPOINT,
	(0x80 | USBD_HID2_EP_INTIN),
	0x03,
	WBVAL(USBD_HID_CUSTOM_REPORT_SIZE),
	1,
   
	0x07,
	USB_DESC_TYPE_ENDPOINT,
	(0x00 | USBD_HID2_EP_INTOUT),
	0x03,
	WBVAL(USBD_HID_CUSTOM_REPORT_SIZE),
	1,
};

/* msd */
const uint8_t msc_descriptor[] =
{
    USB_DESC_LENGTH_INTERFACE,
    USB_DESC_TYPE_INTERFACE,
    USBD_MSC_IF_IDX,
    0x00,
    0x02,
	USB_CLASS_MASS_STORAGE,
	0x06,
	0x50,
    USBD_IF_STR_IDX(USBD_MSC_IF_IDX),
  
	/* endpoint descriptor */
	0x07,
	USB_DESC_TYPE_ENDPOINT,
	(0x80 | USBD_MSC_EP_BULKIN),
	0x02,
	WBVAL(MSD_EP_SIZE),
	0,
   
	0x07,
	USB_DESC_TYPE_ENDPOINT,
	(0x00 | USBD_MSC_EP_BULKOUT),
	0x02,
	WBVAL(MSD_EP_SIZE),
	0,
};


void create_string_descriptor(struct ustring_descriptor *d, uint8_t *buf, uint32_t len)
{
    d->type = USB_DESC_TYPE_STRING;
    d->bLength = len + 2;
    memcpy(d->String, buf, len);
}

desc_t descriptors[] = 
{
    {0xFF, "device_descriptor",               device_descriptor,              sizeof(device_descriptor)},
    {0xFF, "configuration_descriptor",        (uint8_t*)&configuration_descriptor,       sizeof(struct uconfig_descriptor)},
    {0xFF, "cdc_acm_if0",                     cdc_acm_if0_descriptor,         sizeof(cdc_acm_if0_descriptor)},
    {0xFF, "cdc_acm_if1",                     cdc_acm_if1_descriptor,         sizeof(cdc_acm_if1_descriptor)},
    {0xFF, "hid0_descriptor",                 hid0_descriptor,                sizeof(hid0_descriptor)},
    {0xFF, "report_descriptor_keyboard",      report_descriptor_keyboard,     sizeof(report_descriptor_keyboard)},
    {0xFF, "hid1_descriptor",                 hid1_descriptor,                sizeof(hid1_descriptor)},
    {0xFF, "report_descriptor_mouse",         report_descriptor_mouse,        sizeof(report_descriptor_mouse)},
    {0xFF, "hid2_descriptor",                 hid2_descriptor,                sizeof(hid2_descriptor)},
    {0xFF, "report_descriptor_custom",        report_descriptor_custom,       sizeof(report_descriptor_custom)},
    {0xFF, "cdc_acm_iad_descriptor",          cdc_acm_iad_descriptor,         sizeof(cdc_acm_iad_descriptor)},
    {0xFF, "msc_descriptor",                  msc_descriptor,                 sizeof(msc_descriptor)},
    {0xFF, "qualifier_descriptor",            usb_qualifier_descriptor,       sizeof(usb_qualifier_descriptor)},
};



/* get descriptor from descriptor templete list */
uint32_t get_descriptor_data(const char *name, desc_t *d)
{
    int i;
    for(i=0; i<ARRAY_SIZE(descriptors); i++)
    {
        if(!strcmp(name, descriptors[i].name))
        {
            memcpy(d, &descriptors[i], sizeof(desc_t));
            return CH_OK;
        }
    }
    return CH_ERR;
}



