#ifndef __USBD_HID_H_
#define	__USBD_HID_H_

#include <stdint.h>
#include <usbd.h>

/* Usage Pages */
#define HID_USAGE_PAGE_UNDEFINED        0x00
#define HID_USAGE_PAGE_GENERIC          0x01
#define HID_USAGE_PAGE_SIMULATION       0x02
#define HID_USAGE_PAGE_VR               0x03
#define HID_USAGE_PAGE_SPORT            0x04
#define HID_USAGE_PAGE_GAME             0x05
#define HID_USAGE_PAGE_DEV_CONTROLS     0x06
#define HID_USAGE_PAGE_KEYBOARD         0x07
#define HID_USAGE_PAGE_LED              0x08
#define HID_USAGE_PAGE_BUTTON           0x09
#define HID_USAGE_PAGE_ORDINAL          0x0A
#define HID_USAGE_PAGE_TELEPHONY        0x0B
#define HID_USAGE_PAGE_CONSUMER         0x0C
#define HID_USAGE_PAGE_DIGITIZER        0x0D
#define HID_USAGE_PAGE_UNICODE          0x10
#define HID_USAGE_PAGE_ALPHANUMERIC     0x14
/* ... */
/* USB Report Descriptor */

/* 每行开始的第一字节为该条目的前缀，前缀的格式为：

7 6 5 4 | 3 2   | 1 0
bTag    | bType | bSize

bSize: 0: 后面跟0字节  1:后面跟1字节 2:后面跟2字节 3:后面跟4字节

bType 条目类型: 
    0: Main Item 
    1: Global Item  
    2: Local Item

bTag: 条目的功能
当bType 为 Mian Item:
    0 Input
    1 Output
    2 Feature
    3 Collection
    4 End Collection
    
当bType 为 Global Item:
    0 Usage Page
    1 Logical Miniumum
    2 Logical Maximum
    3 Phy Min
    4 Phy Max
    5 Report Size
    6 Report Count
    7 Report ID
    
当bType 为 Local Item:     
    0 Usage
    1 Usage Min
    2 Usage Max

http://www.amobbs.com/thread-4827807-1-1.html?_dsign=86361ec8
http://www.usb.org/developers/hidpage/#Class_Definition
*/


/* Generic Desktop Page (0x01) */
#define HID_USAGE_GENERIC_POINTER               0x01
#define HID_USAGE_GENERIC_MOUSE                 0x02
#define HID_USAGE_GENERIC_JOYSTICK              0x04
#define HID_USAGE_GENERIC_GAMEPAD               0x05
#define HID_USAGE_GENERIC_KEYBOARD              0x06
#define HID_USAGE_GENERIC_KEYPAD                0x07
#define HID_USAGE_GENERIC_X                     0x30
#define HID_USAGE_GENERIC_Y                     0x31
#define HID_USAGE_GENERIC_Z                     0x32
#define HID_USAGE_GENERIC_RX                    0x33
#define HID_USAGE_GENERIC_RY                    0x34
#define HID_USAGE_GENERIC_RZ                    0x35
#define HID_USAGE_GENERIC_SLIDER                0x36
#define HID_USAGE_GENERIC_DIAL                  0x37
#define HID_USAGE_GENERIC_WHEEL                 0x38
#define HID_USAGE_GENERIC_HATSWITCH             0x39
#define HID_USAGE_GENERIC_COUNTED_BUFFER        0x3A
#define HID_USAGE_GENERIC_BYTE_COUNT            0x3B
#define HID_USAGE_GENERIC_MOTION_WAKEUP         0x3C
#define HID_USAGE_GENERIC_VX                    0x40
#define HID_USAGE_GENERIC_VY                    0x41
#define HID_USAGE_GENERIC_VZ                    0x42
#define HID_USAGE_GENERIC_VBRX                  0x43
#define HID_USAGE_GENERIC_VBRY                  0x44
#define HID_USAGE_GENERIC_VBRZ                  0x45
#define HID_USAGE_GENERIC_VNO                   0x46
#define HID_USAGE_GENERIC_SYSTEM_CTL            0x80
#define HID_USAGE_GENERIC_SYSCTL_POWER          0x81
#define HID_USAGE_GENERIC_SYSCTL_SLEEP          0x82
#define HID_USAGE_GENERIC_SYSCTL_WAKE           0x83
#define HID_USAGE_GENERIC_SYSCTL_CONTEXT_MENU   0x84
#define HID_USAGE_GENERIC_SYSCTL_MAIN_MENU      0x85
#define HID_USAGE_GENERIC_SYSCTL_APP_MENU       0x86
#define HID_USAGE_GENERIC_SYSCTL_HELP_MENU      0x87
#define HID_USAGE_GENERIC_SYSCTL_MENU_EXIT      0x88
#define HID_USAGE_GENERIC_SYSCTL_MENU_SELECT    0x89
#define HID_USAGE_GENERIC_SYSCTL_MENU_RIGHT     0x8A
#define HID_USAGE_GENERIC_SYSCTL_MENU_LEFT      0x8B
#define HID_USAGE_GENERIC_SYSCTL_MENU_UP        0x8C
#define HID_USAGE_GENERIC_SYSCTL_MENU_DOWN      0x8D
/* ... */

/* Simulation Controls Page (0x02) */
/* ... */
#define HID_USAGE_SIMULATION_RUDDER             0xBA
#define HID_USAGE_SIMULATION_THROTTLE           0xBB
/* ... */

/* Main Items */
#define HID_Input(x)           0x81,x
#define HID_Output(x)          0x91,x
#define HID_Feature(x)         0xB1,x
#define HID_Collection(x)      0xA1,x
#define HID_EndCollection      0xC0

/* Data (Input, Output, Feature) */
#define HID_Data               0<<0
#define HID_Constant           1<<0
#define HID_Array              0<<1
#define HID_Variable           1<<1
#define HID_Absolute           0<<2
#define HID_Relative           1<<2
#define HID_NoWrap             0<<3
#define HID_Wrap               1<<3
#define HID_Linear             0<<4
#define HID_NonLinear          1<<4
#define HID_PreferredState     0<<5
#define HID_NoPreferred        1<<5
#define HID_NoNullPosition     0<<6
#define HID_NullState          1<<6
#define HID_NonVolatile        0<<7
#define HID_Volatile           1<<7

/* Collection Data */
#define HID_Physical           0x00
#define HID_Application        0x01
#define HID_Logical            0x02
#define HID_Report             0x03
#define HID_NamedArray         0x04
#define HID_UsageSwitch        0x05
#define HID_UsageModifier      0x06

/* Global Items */
#define HID_UsagePage(x)       0x05,x
#define HID_UsagePageVendor(x) 0x06,x,0xFF
#define HID_LogicalMin(x)      0x15,x
#define HID_LogicalMinS(x)     0x16,(x&0xFF),((x>>8)&0xFF)
#define HID_LogicalMinL(x)     0x17,(x&0xFF),((x>>8)&0xFF),((x>>16)&0xFF),((x>>24)&0xFF)
#define HID_LogicalMax(x)      0x25,x
#define HID_LogicalMaxS(x)     0x26,(x&0xFF),((x>>8)&0xFF)
#define HID_LogicalMaxL(x)     0x27,(x&0xFF),((x>>8)&0xFF),((x>>16)&0xFF),((x>>24)&0xFF)
#define HID_PhysicalMin(x)     0x35,x
#define HID_PhysicalMinS(x)    0x36,(x&0xFF),((x>>8)&0xFF)
#define HID_PhysicalMinL(x)    0x37,(x&0xFF),((x>>8)&0xFF),((x>>16)&0xFF),((x>>24)&0xFF)
#define HID_PhysicalMax(x)     0x45,x
#define HID_PhysicalMaxS(x)    0x46,(x&0xFF),((x>>8)&0xFF)
#define HID_PhysicalMaxL(x)    0x47,(x&0xFF),((x>>8)&0xFF),((x>>16)&0xFF),((x>>24)&0xFF)
#define HID_UnitExponent(x)    0x55,x
#define HID_Unit(x)            0x65,x
#define HID_UnitS(x)           0x66,(x&0xFF),((x>>8)&0xFF)
#define HID_UnitL(x)           0x67,(x&0xFF),((x>>8)&0xFF),((x>>16)&0xFF),((x>>24)&0xFF)
#define HID_ReportSize(x)      0x75,x
#define HID_ReportSizeS(x)     0x76,(x&0xFF),((x>>8)&0xFF)
#define HID_ReportSizeL(x)     0x77,(x&0xFF),((x>>8)&0xFF),((x>>16)&0xFF),((x>>24)&0xFF)
#define HID_ReportID(x)        0x85,x
#define HID_ReportCount(x)     0x95,x
#define HID_ReportCountS(x)    0x96,(x&0xFF),((x>>8)&0xFF)
#define HID_ReportCountL(x)    0x97,(x&0xFF),((x>>8)&0xFF),((x>>16)&0xFF),((x>>24)&0xFF)
#define HID_Push               0xA4
#define HID_Pop                0xB4

/* Local Items */
#define HID_Usage(x)           0x09,x
#define HID_UsageMin(x)        0x19,x
#define HID_UsageMax(x)        0x29,x




/* HID Subclass Codes */
#define HID_SUBCLASS_NONE               0x00
#define HID_SUBCLASS_BOOT               0x01

/* HID Protocol Codes */
#define HID_PROTOCOL_NONE               0x00
#define HID_PROTOCOL_BOOT               0x00
#define HID_PROTOCOL_KEYBOARD           0x01
#define HID_PROTOCOL_REPORT             0x01
#define HID_PROTOCOL_MOUSE              0x02

/* HID KEYBOARD KEY DEFINE */
/*Key Code*/
#define KEY_ERRORROLLOVER 0x01U
#define KEY_POSTFAIL 0x02U
#define KEY_ERRORUNDEFINED 0x03U
#define KEY_A 0x04U
#define KEY_B 0x05U
#define KEY_C 0x06U
#define KEY_D 0x07U
#define KEY_E 0x08U
#define KEY_F 0x09U
#define KEY_G 0x0AU
#define KEY_H 0x0BU
#define KEY_I 0x0CU
#define KEY_J 0x0DU
#define KEY_K 0x0EU
#define KEY_L 0x0FU
#define KEY_M 0x10U
#define KEY_N 0x11U
#define KEY_O 0x12U
#define KEY_P 0x13U
#define KEY_Q 0x14U
#define KEY_R 0x15U
#define KEY_S 0x16U
#define KEY_T 0x17U
#define KEY_U 0x18U
#define KEY_V 0x19U
#define KEY_W 0x1AU
#define KEY_X 0x1BU
#define KEY_Y 0x1CU
#define KEY_Z 0x1DU
#define KEY_1_EXCLAMATION_MARK 0x1EU
#define KEY_2_AT 0x1FU
#define KEY_3_NUMBER_SIGN 0x20U
#define KEY_4_DOLLAR 0x21U
#define KEY_5_PERCENT 0x22U
#define KEY_6_CARET 0x23U
#define KEY_7_AMPERSAND 0x24U
#define KEY_8_ASTERISK 0x25U
#define KEY_9_OPARENTHESIS 0x26U
#define KEY_0_CPARENTHESIS 0x27U
#define KEY_ENTER 0x28U
#define KEY_ESCAPE 0x29U
#define KEY_BACKSPACE 0x2AU
#define KEY_TAB 0x2BU
#define KEY_SPACEBAR 0x2CU
#define KEY_MINUS_UNDERSCORE 0x2DU
#define KEY_EQUAL_PLUS 0x2EU
#define KEY_OBRACKET_AND_OBRACE 0x2FU
#define KEY_CBRACKET_AND_CBRACE 0x30U
#define KEY_BACKSLASH_VERTICAL_BAR 0x31U
#define KEY_NONUS_NUMBER_SIGN_TILDE 0x32U
#define KEY_SEMICOLON_COLON 0x33U
#define KEY_SINGLE_AND_DOUBLE_QUOTE 0x34U
#define KEY_GRAVE_ACCENT_AND_TILDE 0x35U
#define KEY_COMMA_AND_LESS 0x36U
#define KEY_DOT_GREATER 0x37U
#define KEY_SLASH_QUESTION 0x38U
#define KEY_CAPS_LOCK 0x39U
#define KEY_F1 0x3AU
#define KEY_F2 0x3BU
#define KEY_F3 0x3CU
#define KEY_F4 0x3DU
#define KEY_F5 0x3EU
#define KEY_F6 0x3FU
#define KEY_F7 0x40U
#define KEY_F8 0x41U
#define KEY_F9 0x42U
#define KEY_F10 0x43U
#define KEY_F11 0x44U
#define KEY_F12 0x45U
#define KEY_PRINTSCREEN 0x46U
#define KEY_SCROLL_LOCK 0x47U
#define KEY_PAUSE 0x48U
#define KEY_INSERT 0x49U
#define KEY_HOME 0x4AU
#define KEY_PAGEUP 0x4BU
#define KEY_DELETE 0x4CU
#define KEY_END1 0x4DU
#define KEY_PAGEDOWN 0x4EU
#define KEY_RIGHTARROW 0x4FU
#define KEY_LEFTARROW 0x50U
#define KEY_DOWNARROW 0x51U
#define KEY_UPARROW 0x52U
#define KEY_KEYPAD_NUM_LOCK_AND_CLEAR 0x53U
#define KEY_KEYPAD_SLASH 0x54U
#define KEY_KEYPAD_ASTERIKS 0x55U
#define KEY_KEYPAD_MINUS 0x56U
#define KEY_KEYPAD_PLUS 0x57U
#define KEY_KEYPAD_ENTER 0x58U
#define KEY_KEYPAD_1_END 0x59U
#define KEY_KEYPAD_2_DOWN_ARROW 0x5AU
#define KEY_KEYPAD_3_PAGEDN 0x5BU
#define KEY_KEYPAD_4_LEFT_ARROW 0x5CU
#define KEY_KEYPAD_5 0x5DU
#define KEY_KEYPAD_6_RIGHT_ARROW 0x5EU
#define KEY_KEYPAD_7_HOME 0x5FU
#define KEY_KEYPAD_8_UP_ARROW 0x60U
#define KEY_KEYPAD_9_PAGEUP 0x61U
#define KEY_KEYPAD_0_INSERT 0x62U
#define KEY_KEYPAD_DECIMAL_SEPARATOR_DELETE 0x63U
#define KEY_NONUS_BACK_SLASH_VERTICAL_BAR 0x64U
#define KEY_APPLICATION 0x65U
#define KEY_POWER 0x66U
#define KEY_KEYPAD_EQUAL 0x67U
#define KEY_F13 0x68U
#define KEY_F14 0x69U
#define KEY_F15 0x6AU
#define KEY_F16 0x6BU
#define KEY_F17 0x6CU
#define KEY_F18 0x6DU
#define KEY_F19 0x6EU
#define KEY_F20 0x6FU
#define KEY_F21 0x70U
#define KEY_F22 0x71U
#define KEY_F23 0x72U
#define KEY_F24 0x73U
#define KEY_EXECUTE 0x74U
#define KEY_HELP 0x75U
#define KEY_MENU 0x76U
#define KEY_SELECT 0x77U
#define KEY_STOP 0x78U
#define KEY_AGAIN 0x79U
#define KEY_UNDO 0x7AU
#define KEY_CUT 0x7BU
#define KEY_COPY 0x7CU
#define KEY_PASTE 0x7DU
#define KEY_FIND 0x7EU
#define KEY_MUTE 0x7FU
#define KEY_VOLUME_UP 0x80U
#define KEY_VOLUME_DOWN 0x81U
#define KEY_LOCKING_CAPS_LOCK 0x82U
#define KEY_LOCKING_NUM_LOCK 0x83U
#define KEY_LOCKING_SCROLL_LOCK 0x84U
#define KEY_KEYPAD_COMMA 0x85U
#define KEY_KEYPAD_EQUAL_SIGN 0x86U
#define KEY_INTERNATIONAL1 0x87U
#define KEY_INTERNATIONAL2 0x88U
#define KEY_INTERNATIONAL3 0x89U
#define KEY_INTERNATIONAL4 0x8AU
#define KEY_INTERNATIONAL5 0x8BU
#define KEY_INTERNATIONAL6 0x8CU
#define KEY_INTERNATIONAL7 0x8DU
#define KEY_INTERNATIONAL8 0x8EU
#define KEY_INTERNATIONAL9 0x8FU
#define KEY_LANG1 0x90U
#define KEY_LANG2 0x91U
#define KEY_LANG3 0x92U
#define KEY_LANG4 0x93U
#define KEY_LANG5 0x94U
#define KEY_LANG6 0x95U
#define KEY_LANG7 0x96U
#define KEY_LANG8 0x97U
#define KEY_LANG9 0x98U
#define KEY_ALTERNATE_ERASE 0x99U
#define KEY_SYSREQ 0x9AU
#define KEY_CANCEL 0x9BU
#define KEY_CLEAR 0x9CU
#define KEY_PRIOR 0x9DU
#define KEY_RETURN 0x9EU
#define KEY_SEPARATOR 0x9FU
#define KEY_OUT 0xA0U
#define KEY_OPER 0xA1U
#define KEY_CLEAR_AGAIN 0xA2U
#define KEY_CRSEL 0xA3U
#define KEY_EXSEL 0xA4U
#define KEY_KEYPAD_00 0xB0U
#define KEY_KEYPAD_000 0xB1U
#define KEY_THOUSANDS_SEPARATOR 0xB2U
#define KEY_DECIMAL_SEPARATOR 0xB3U
#define KEY_CURRENCY_UNIT 0xB4U
#define KEY_CURRENCY_SUB_UNIT 0xB5U
#define KEY_KEYPAD_OPARENTHESIS 0xB6U
#define KEY_KEYPAD_CPARENTHESIS 0xB7U
#define KEY_KEYPAD_OBRACE 0xB8U
#define KEY_KEYPAD_CBRACE 0xB9U
#define KEY_KEYPAD_TAB 0xBAU
#define KEY_KEYPAD_BACKSPACE 0xBBU
#define KEY_KEYPAD_A 0xBCU
#define KEY_KEYPAD_B 0xBDU
#define KEY_KEYPAD_C 0xBEU
#define KEY_KEYPAD_D 0xBFU
#define KEY_KEYPAD_E 0xC0U
#define KEY_KEYPAD_F 0xC1U
#define KEY_KEYPAD_XOR 0xC2U
#define KEY_KEYPAD_CARET 0xC3U
#define KEY_KEYPAD_PERCENT 0xC4U
#define KEY_KEYPAD_LESS 0xC5U
#define KEY_KEYPAD_GREATER 0xC6U
#define KEY_KEYPAD_AMPERSAND 0xC7U
#define KEY_KEYPAD_LOGICAL_AND 0xC8U
#define KEY_KEYPAD_VERTICAL_BAR 0xC9U
#define KEY_KEYPAD_LOGIACL_OR 0xCAU
#define KEY_KEYPAD_COLON 0xCBU
#define KEY_KEYPAD_NUMBER_SIGN 0xCCU
#define KEY_KEYPAD_SPACE 0xCDU
#define KEY_KEYPAD_AT 0xCEU
#define KEY_KEYPAD_EXCLAMATION_MARK 0xCFU
#define KEY_KEYPAD_MEMORY_STORE 0xD0U
#define KEY_KEYPAD_MEMORY_RECALL 0xD1U
#define KEY_KEYPAD_MEMORY_CLEAR 0xD2U
#define KEY_KEYPAD_MEMORY_ADD 0xD3U
#define KEY_KEYPAD_MEMORY_SUBTRACT 0xD4U
#define KEY_KEYPAD_MEMORY_MULTIPLY 0xD5U
#define KEY_KEYPAD_MEMORY_DIVIDE 0xD6U
#define KEY_KEYPAD_PLUSMINUS 0xD7U
#define KEY_KEYPAD_CLEAR 0xD8U
#define KEY_KEYPAD_CLEAR_ENTRY 0xD9U
#define KEY_KEYPAD_BINARY 0xDAU
#define KEY_KEYPAD_OCTAL 0xDBU
#define KEY_KEYPAD_DECIMAL 0xDCU
#define KEY_KEYPAD_HEXADECIMAL 0xDDU
#define KEY_LEFTCONTROL 0xE0U
#define KEY_LEFTSHIFT 0xE1U
#define KEY_LEFTALT 0xE2U
#define KEY_LEFT_GUI 0xE3U
#define KEY_RIGHTCONTROL 0xE4U
#define KEY_RIGHTSHIFT 0xE5U
#define KEY_RIGHTALT 0xE6U
#define KEY_RIGHT_GUI 0xE7U

#define MODIFERKEYS_LEFT_CTRL 0x01U
#define MODIFERKEYS_LEFT_SHIFT 0x02U
#define MODIFERKEYS_LEFT_ALT 0x04U
#define MODIFERKEYS_LEFT_GUI 0x08U
#define MODIFERKEYS_RIGHT_CTRL 0x10U
#define MODIFERKEYS_RIGHT_SHIFT 0x20U
#define MODIFERKEYS_RIGHT_ALT 0x40U
#define MODIFERKEYS_RIGHT_GUI 0x80U


struct usbd_hid_callback_t
{
    uint32_t (*data_send_nty)(uint8_t ep);
    uint32_t (*data_received)(uint8_t ep, uint8_t *buf, uint32_t len);
};

uint32_t usbd_set_mouse(uint8_t btn, int8_t x, int8_t y, int8_t wheel);
uint32_t usbd_set_keyboard(uint8_t fun_key, uint8_t key);
uint32_t usbd_hid_send(uint8_t *buf, uint32_t len);

uint32_t hid_standard_request_to_intf_handler(struct usbd_t *h);
uint32_t hid_class_request_handler(struct usbd_t *h);
uint32_t hid_data_ep_handler(uint8_t ep, uint8_t dir);
void usbd_hid_set_cb(struct usbd_hid_callback_t *cb);
void usbd_hid_init(struct usbd_t *h, uint32_t intf_cnt);
    
#endif
