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
    WBVAL(USBD_VID),                      /* idVendor ��Ӧ��ID��VID�� */
    WBVAL(USBD_PID),                      /* idProduct  make sure different configuration use different PID */
    WBVAL(0x0100),                        /* bcdDevice �豸�汾�ţ�����BCD�룩 */
    0x01,                                 /* iManufacturer  ��Ӧ�̵��ַ������������� */
    0x02,                                 /* iProduct  ��Ʒ���ַ������������� */
    0x03,                                 /* iSerialNumber �豸��ŵ��ַ������������� */
    0x01                                  /* bNumConfigurations: one possible configuration ��USB�豸֧�ֵ�������Ŀ*/
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
    /* CDC��ӿ������� */
    USB_DESC_LENGTH_INTERFACE,                  /* bLength�ֶΡ��ӿ��������ĳ���Ϊ9�ֽ� */
    USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType�ֶΡ��ӿ��������ı��Ϊ0x04�� */
	USBD_CDC_CIF_IDX,                           /* bInterfaceNumber�򣬽ӿں� */
	0x00,                                       /* bAlternateSetting�򣬽ӿڵĿ��滻����ֵ ����Ҫ�Ķ� */
	0x01,                                       /* bNumEndpoints�򣬷�0�˵����Ŀ��CDC�ӿ�ֻʹ��һ���ж� */
    USB_CLASS_CDC,                              /* bInterfaceClass�ֶΡ��ýӿ���ʹ�õ��ࡣCDC��������Ϊ0x02�� */
    0x02,                                       /* bInterfaceSubClass�ֶΡ��ýӿ���ʹ�õ����ࡣҪʵ��USBת���ڣ� �ͱ���ʹ��Abstract Control Model���������ģ�ͣ����ࡣ���ı��Ϊ0x02�� */
    0x01,                                       /* bInterfaceProtocol�ֶΡ�ʹ��Common AT Commands��ͨ��AT��� Э�顣��Э��ı��Ϊ0x01�� */
    USBD_IF_STR_IDX(USBD_CDC_CIF_IDX),
    
    /* Header Functional Descriptor */
    0x05,                                       /* bFunctionLength�ֶΡ�������������Ϊ5�ֽ� */
    0x24,                                       /* bDescriptorType�ֶΡ�����������Ϊ������ӿڣ�CS_INTERFACE�� ���Ϊ0x24��*/
    0x00,                                       /* bDescriptorSubtype�ֶΡ�����������ΪHeader Functional Descriptor ���Ϊ0x00�� */
    WBVAL(USB_VERSION_2_0),                     /* USB2.00 */
    
    /* Call Management Functional Descriptor */
    0x05,                                       /* bFunctionLength�ֶΡ�������������Ϊ5�ֽ� */
    0x24,                                       /* bDescriptorType�ֶΡ�����������Ϊ������ӿڣ�CS_INTERFACE�� ���Ϊ0x24�� */
    0x01,                                       /* bDescriptorSubtype�ֶΡ�����������ΪCall Management functional descriptor�����Ϊ0x01��*/
    0x01,                                       /* bmCapabilities�ֶΡ��豸�Լ�������call management */
    0x01,                                       /* bDataInterface�ֶΡ�û��������ӿ�����call management */
    
    /* Abstract Control Management Functional Descriptor */
    0x04,                                       /* bFunctionLength�ֶΡ�������������Ϊ4�ֽ� */
    0x24,                                       /* bDescriptorType�ֶΡ�����������Ϊ������ӿڣ�CS_INTERFACE�� ���Ϊ0x24��*/
    0x02,                                       /* bDescriptorSubtype�ֶΡ�����������ΪAbstract Control Management functional descriptor�����Ϊ0x02��*/
    0x06,                                       /* bmCapabilities�ֶΡ�֧��Set_Line_Coding��Set_Control_Line_State�� Get_Line_Coding�����Serial_State֪ͨ */

    /* Union Functional Descriptor */
    0x05,                                       /* bFunctionLength�ֶΡ�������������Ϊ5�ֽڡ�  */
    0x24,                                       /* bDescriptorType�ֶΡ�����������Ϊ������ӿڣ�CS_INTERFACE�� ���Ϊ0x24 */
    0x06,                                       /* bDescriptorSubtype�ֶΡ�����������Ϊ Union functional descriptor�����Ϊ0x06�� */
    0x00,                                       /* MasterInterface�ֶΡ�����Ϊǰ����Ϊ0��CDC�ӿڡ� */
    0x01,                                       /* SlaveInterface�ֶΣ�����Ϊ���������Ϊ1��������ӿڡ�*/
    
	/* �˵������� */
	0x07,                                       /* bLength�򣬶˵����������ȣ�7�ֽ� */
	USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType��0x05��ʾ��������Ϊ�˵������� */
	(0x80 | USBD_CDC_ACM_EP_INTIN),             /* bEndpointAddress�򣬶˵�źʹ��䷽�򣺶˵� [bit7]=0 ��output */
	0x03,                                       /* bmAttributes�򣬶˵����ԣ��жϴ��� */
    WBVAL(64),                                  /* wMaxPacketSize�򣬶˵�֧��������ݰ����� */
	0x0A,                                       /* bInterval����ѯ�������msΪ��λ��*/
};


const uint8_t cdc_acm_if1_descriptor[] = 
{
    /*  ����Ϊ�ӿ�1�����ݽӿڣ��Ľӿ�������  */
    USB_DESC_LENGTH_INTERFACE,
    USB_DESC_TYPE_INTERFACE,
    USBD_CDC_DIF_IDX,                           /* bInterfaceNumber�ֶΡ��ýӿڵı�ţ��ڶ����ӿڣ����Ϊ1��*/
    0x00,                                       /* bAlternateSetting�ֶΡ��ýӿڵı��ñ�ţ�Ϊ0�� */
    0x02,                                       /* bNumEndpoints�ֶΡ���0�˵����Ŀ�����豸��Ҫʹ��һ�������˵㣬����Ϊ2��*/
    0x0A,                                       /* bInterfaceClass�ֶΡ��ýӿ���ʹ�õ��ࡣ������ӿڵĴ���Ϊ0x0A��*/
    0x00,                                       /* bInterfaceSubClass�ֶΡ��ýӿ���ʹ�õ�����Ϊ0��*/
    0x00,                                       /* bInterfaceProtocol�ֶΡ��ýӿ���ʹ�õ�Э��Ϊ0��*/
    USBD_IF_STR_IDX(USBD_CDC_DIF_IDX),
    
	/* �˵������� */
	0x07,                                       /* bLength�򣬶˵����������ȣ�7�ֽ� */
	USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType��0x05��ʾ��������Ϊ�˵������� */
	(0x80 | USBD_CDC_ACM_EP_BULKIN),            /* bEndpointAddress�򣬶˵�źʹ��䷽�򣺶˵� [bit7]=0 ��output */
	0x02,                                       /* bmAttributes�򣬶˵����ԣ������˵�ı��Ϊ0x02 */
	WBVAL(CDC_EP_SIZE),                         /* wMaxPacketSize�򣬶˵�֧��������ݰ����� */
	0x00,                                       /* bInterval�ֶΡ��˵��ѯ��ʱ�䣬����������˵���Ч��*/
    
	/* �˵������� */
	0x07,                                       /* bLength�򣬶˵����������ȣ�7�ֽ� */
	USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType��0x05��ʾ��������Ϊ�˵������� */
	(0x00 | USBD_CDC_ACM_EP_BULKOUT),           /* bEndpointAddress�򣬶˵�źʹ��䷽�򣺶˵� [bit7]=0 ��output */
	0x02,                                       /* bmAttributes�򣬶˵����ԣ������˵�ı��Ϊ0x02 */
	WBVAL(CDC_EP_SIZE),                         /* wMaxPacketSize�򣬶˵�֧��������ݰ����� */
	0x00,                                       /* bInterval�ֶΡ��˵��ѯ��ʱ�䣬����������˵���Ч��*/
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
	USB_DESC_LENGTH_INTERFACE,                      /* bLength�򣬽ӿ����������ȣ�9�ֽ� */
	USB_DESC_TYPE_INTERFACE,                        /* bDescriptorType��0x04��ʾ��������Ϊ�ӿ������� */
	USBD_HID0_IF_IDX,                               /* bInterfaceNumber�򣬽ӿں� */
	0x00,                                           /* bAlternateSetting�򣬽ӿڵĿ��滻����ֵ ����Ҫ�Ķ� */
	0x02,                                           /* bNumEndpoints�򣬽ӿ�ʹ�õĶ˵��������˵�0 */
	USB_CLASS_HID,                                  /* bInterfaceClass�򣬽ӿ�������USB�豸�ࣺ0xFF��ʾ��Ӧ���Զ��� */
	HID_SUBCLASS_NONE,                              /* bInterfaceSubClass�򣬽ӿ�������USB�豸���ࣺ0xFF��ʾ��Ӧ���Զ��� */
	HID_PROTOCOL_NONE,                              /* bInterfaceProtocol�򣬽ӿڲ��õ�USB�豸��Э�飺0xFF��ʾ��Ӧ���Զ��� */
	USBD_IF_STR_IDX(USBD_HID0_IF_IDX),              /* iInterface�򣬽ӿ��ַ�����������������0 */
           
    /* HID descritor */
    0x09,                                       /* bLength�ֶΡ���HID��������ֻ��һ���¼������������Գ���Ϊ9�ֽ� */
    0x21,                                       /* bDescriptorType�ֶΡ�HID�������ı��Ϊ 0x21 */
    WBVAL(0x0100),                              /* bcdHID */ 
    0x21,                                       /* bCountyCode�ֶΡ��豸���õĹ��Ҵ��룬����ѡ��Ϊ����������0x21�� */
    0x01,                                       /* bNumDescriptors�ֶΡ��¼�����������Ŀ������ֻ��һ������������ */
    0x22,                                       /* bDescriptorType�ֶΡ��¼������������ͣ�Ϊ���������������Ϊ0x22 */
    WBVAL(sizeof(report_descriptor_keyboard)),  /* bDescriptorLength�ֶΡ��¼��������ĳ��ȡ��¼�������Ϊ������������ */
  
	/* endpoint descritor */
	0x07,                                           /* bLength�򣬶˵����������ȣ�7�ֽ� */
	USB_DESC_TYPE_ENDPOINT,                         /* bDescriptorType��0x05��ʾ��������Ϊ�˵������� */
	(0x80 | USBD_HID0_EP_INTIN),            /* bEndpointAddress�򣬶˵�źʹ��䷽�򣺶˵� [bit7]=0 ��output */
	0x03,                                           /* bmAttributes�򣬶˵����ԣ��жϴ��� */
	WBVAL(64),                                      /* wMaxPacketSize�򣬶˵�֧��������ݰ����� */
	USBD_HID_EP_INTERVAL,                           /* bInterval����ѯ�������msΪ��λ��*/
   
	0x07,                                           /* bLength�򣬶˵����������ȣ�7�ֽ� */
	USB_DESC_TYPE_ENDPOINT,                         /* bDescriptorType��0x05��ʾ��������Ϊ�˵������� */
	(0x00 | USBD_HID0_EP_INTOUT),           /* bEndpointAddress�򣬶˵�źʹ��䷽�򣺶˵� [bit7]=0 ��output */
	0x03,                                           /* bmAttributes�򣬶˵����ԣ��жϴ��� */
	WBVAL(64),                                      /* wMaxPacketSize�򣬶˵�֧��������ݰ����� */
	USBD_HID_EP_INTERVAL,                           /* bInterval����ѯ�������msΪ��λ��*/
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
//    HID_Collection(0x01),                           /* ����(Application) */
//        HID_ReportSize(8),                          /* Report Size(8 bit)*/
//#if (USBD_HID_CUSTOM_REPORT_SIZE >= 256)
//    HID_ReportCountS(USBD_HID_CUSTOM_REPORT_SIZE),      /* Report Count: byte0 used for ReportID, so it's  USBD_HID_REPORT_SIZE - 1*/
//#else
//    HID_ReportCount(USBD_HID_CUSTOM_REPORT_SIZE),      /* Report Count: byte0 used for ReportID, so it's  USBD_HID_REPORT_SIZE - 1*/
//#endif

//        HID_Usage(0x00),
//        HID_Input(0x02),                            /* ����(data, variable, absolute) */
//    
//        HID_Usage(0x00),
//        HID_Output(0x02),                           /* ���(data, variable, absolute) */
//    HID_EndCollection,                              /* ���Ͻ���(Application) */
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
           
    /* HID������ */
    0x09,
    0x21,
    WBVAL(0x0100),
    0x21,
    0x01,
    0x22,
    WBVAL(sizeof(report_descriptor_mouse)),
  
	/* �˵������� */
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



