/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
 
#ifndef __KPTL_H__
#define __KPTL_H__

#include <stdint.h>
#include <stdbool.h>


#ifndef MAX_PACKET_LEN
#define MAX_PACKET_LEN          (64)
#endif

#define ARRAY2INT16(x)     (x[0] + (x[1] << 8))

/* header include start_byte and type */
typedef struct
{
    uint8_t start_byte;
    uint8_t packet_type;
}packet_hr_t;

typedef struct
{
    packet_hr_t     hr;
    uint8_t         len[2];
    uint8_t         crc16[2];
    uint8_t         payload[MAX_PACKET_LEN];
}frame_packet_t;

typedef struct
{
    frame_packet_t*  fp;
    uint32_t         cnt;
    void (*cb)(frame_packet_t *pkt);
    uint8_t          status;
}pkt_dec_t;

/* ping packet, ack packet, nak packet are only contain 2 bytes */
typedef packet_hr_t packet_ping_t;
typedef packet_hr_t packet_ack_t;
typedef packet_hr_t packet_nak_t;

typedef struct
{
    packet_hr_t hr;
    uint8_t bug_fix;
    uint8_t ver_minor;
    uint8_t ver_major;
    uint8_t protocol_name;
    uint8_t option_low;
    uint8_t option_high;
    uint8_t crc16[2];
}ping_resp_packet_t;

typedef struct
{
    uint8_t tag;            //!< A command tag.
    uint8_t flags;          //!< Combination of packet flags.
    uint8_t reserved;       //!< Reserved, helpful for alignment, set to zero.
    uint8_t param_cnt;      //!< Number of parameters that follow in buffer.
    uint32_t *param;
} cmd_packet_t;

/* packet type */
enum
{
    kFramingPacketStartByte         = 0x5A,
    kFramingPacketType_Ack          = 0xA1,
    kFramingPacketType_Nak          = 0xA2,
    kFramingPacketType_AckAbort     = 0xA3,
    kFramingPacketType_Command      = 0xA4,
    kFramingPacketType_Data         = 0xA5,
    kFramingPacketType_Ping         = 0xA6,
    kFramingPacketType_PingResponse = 0xA7
};

/* command tag */
enum 
{
    kCommandTag_GenericResponse             = 0xa0,
    kCommandTag_FlashEraseAll               = 0x01,
    kCommandTag_FlashEraseRegion            = 0x02,
    kCommandTag_ReadMemory                  = 0x03,
    kCommandTag_ReadMemoryResponse          = 0xa3,
    kCommandTag_WriteMemory                 = 0x04,
    kCommandTag_FillMemory                  = 0x05,
    kCommandTag_FlashSecurityDisable        = 0x06,
    kCommandTag_GetProperty                 = 0x07,
    kCommandTag_GetPropertyResponse         = 0xa7,
    kCommandTag_ReceiveSbFile               = 0x08,
    kCommandTag_Execute                     = 0x09,
    kCommandTag_Call                        = 0x0a,
    kCommandTag_Reset                       = 0x0b,
    kCommandTag_SetProperty                 = 0x0c,
    kCommandTag_FlashEraseAllUnsecure       = 0x0d,
    kCommandTag_FlashProgramOnce            = 0x0e,
    kCommandTag_FlashReadOnce               = 0x0f,
    kCommandTag_FlashReadOnceResponse       = 0xaf,
    kCommandTag_FlashReadResource           = 0x10,
    kCommandTag_FlashReadResourceResponse   = 0xb0,
    kCommandTag_ConfigureQuadSpi            = 0x11,

    kFirstCommandTag                    = kCommandTag_FlashEraseAll,

    //! Maximum linearly incrementing command tag value, excluding the response commands.
    kLastCommandTag                     = kCommandTag_ConfigureQuadSpi,

    kResponseCommandHighNibbleMask = 0xa0           //!< Mask for the high nibble of a command tag that identifies it as a response command.
};

/* frame packet API: data and command packet need to warpped in frame packet */
uint32_t kptl_frame_packet_add(frame_packet_t *pkt, uint8_t *buf, uint16_t len);
uint32_t kptl_frame_packet_begin(frame_packet_t *pkt, uint8_t type);
uint32_t kptl_frame_packet_final(frame_packet_t *pkt);
uint32_t kptl_frame_packet_get_size(frame_packet_t *p);

/* resp packet, resp packet is a speical form of command packet */
uint32_t kptl_create_generic_resp_packet(frame_packet_t *pkt, uint32_t status_code, uint32_t cmd_tag);
uint32_t kptl_create_property_resp_packet(frame_packet_t *p, uint8_t param_cnt, uint32_t *param);

/* command packet, command packet must be waprred in frame packet */
void kptl_create_cmd_packet(frame_packet_t *fp, cmd_packet_t *cp, uint32_t *param);
uint32_t kptl_cmd_packet_get_size(cmd_packet_t *cp);

/* ping and ping resp packet */
void kptl_create_ping(packet_ping_t *p);
void kptl_create_ack(packet_ack_t *p);
void kptl_create_nak(packet_nak_t *p);
void kptl_create_ping_resp_packet(ping_resp_packet_t *p, uint8_t major, uint8_t minor, uint8_t bugfix, uint8_t opt_low, uint8_t opt_high);

/* packet decode API */
int kptl_decode_init(pkt_dec_t *d);
uint32_t kptl_decode(pkt_dec_t *d, uint8_t c);
void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes);

#endif

