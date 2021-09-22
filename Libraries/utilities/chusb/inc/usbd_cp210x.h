#ifndef __USBD_CP210X_SERIAL_H_
#define	__USBD_CP210X_SERIAL_H_

#include <stdint.h>
#include <usbd.h>
#include <usb_common.h>

/* Config request codes */
#define CP210X_IFC_ENABLE	0x00
#define CP210X_SET_BAUDDIV	0x01
#define CP210X_GET_BAUDDIV	0x02
#define CP210X_SET_LINE_CTL	0x03
#define CP210X_GET_LINE_CTL	0x04
#define CP210X_SET_BREAK	0x05
#define CP210X_IMM_CHAR		0x06
#define CP210X_SET_MHS		0x07
#define CP210X_GET_MDMSTS	0x08
#define CP210X_SET_XON		0x09
#define CP210X_SET_XOFF		0x0A
#define CP210X_SET_EVENTMASK	0x0B
#define CP210X_GET_EVENTMASK	0x0C
#define CP210X_SET_CHAR		0x0D
#define CP210X_GET_CHARS	0x0E
#define CP210X_GET_PROPS	0x0F
#define CP210X_GET_COMM_STATUS	0x10
#define CP210X_RESET		0x11
#define CP210X_PURGE		0x12
#define CP210X_SET_FLOW		0x13
#define CP210X_GET_FLOW		0x14
#define CP210X_EMBED_EVENTS	0x15
#define CP210X_GET_EVENTSTATE	0x16
#define CP210X_SET_CHARS	0x19
#define CP210X_GET_BAUDRATE	0x1D
#define CP210X_SET_BAUDRATE	0x1E
#define CP210X_VENDOR_SPECIFIC	0xFF

typedef struct
{
	uint16_t	wLength;
	uint16_t	bcdVersion;
	uint32_t	ulServiceMask;
	uint32_t	_reserved8;
	uint32_t	ulMaxTxQueue;
	uint32_t	ulMaxRxQueue;
	uint32_t	ulMaxBaud;
	uint32_t	ulProvSubType;
	uint32_t	ulProvCapabilities;
	uint32_t	ulSettableParams;
	uint32_t	ulSettableBaud;
	uint16_t	wSettableData;
	uint16_t	_reserved42;
	uint32_t	ulCurrentTxQueue;
	uint32_t	ulCurrentRxQueue;
	uint32_t	_reserved52;
	uint32_t	_reserved56;
	uint16_t	uniProvName[15];
}cp210x_cpr_t;


__packed typedef struct
{
	uint32_t    ulErrors;
	uint32_t    ulHoldReasons;
	uint32_t    ulAmountInInQueue;
	uint32_t    ulAmountInOutQueue;
	uint8_t     bEofReceived;
	uint8_t     bWaitForImmediate;
	uint8_t     bReserved;
} cp210x_ssr_t;

struct usbd_cp210x_callback_t
{
    uint32_t (*get_line_coding)(struct ucdc_line_coding *line_coding);
    uint32_t (*set_line_coding)(struct ucdc_line_coding *line_coding);
    uint32_t (*set_control_line_serial_state)(uint8_t val);
    uint32_t (*recv_handler)(uint8_t *buf, uint32_t len);
    uint32_t (*send_notify)(void);
};


void usbd_vsc_cp210x_init(struct usbd_t *h);

#endif






