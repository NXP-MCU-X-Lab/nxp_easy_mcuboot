#include <string.h>
#include <stdio.h>

#include "kptl.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))
#endif

#ifndef CH_OK
#define CH_OK   (0)
#endif

#ifndef CH_ERR
#define CH_ERR  (1)
#endif

/* generate CRC16
    @param  currectCrc:     previous buffer pointer, if is a new start, pointer should refer a zero uint16_t
    @param  src:            current buffer pointer
    @param  lengthInBytes:  length of current buf
*/
void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
    uint32_t crc = *currectCrc;
    uint32_t j;
    for (j=0; j < lengthInBytes; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    } 
    *currectCrc = crc;
}

void kptl_create_ping(packet_ping_t *p)
{
    p->start_byte = kFramingPacketStartByte;
    p->packet_type = kFramingPacketType_Ping;
}

void kptl_create_ack(packet_ack_t *p)
{
    p->start_byte = kFramingPacketStartByte;
    p->packet_type = kFramingPacketType_Ack;
}

void kptl_create_nak(packet_nak_t *p)
{
    p->start_byte = kFramingPacketStartByte;
    p->packet_type = kFramingPacketType_Nak;
}
    
void kptl_create_cmd_packet(frame_packet_t *fp, cmd_packet_t *cp, uint32_t *param)
{
    int i;
    kptl_frame_packet_begin(fp, kFramingPacketType_Command);
    kptl_frame_packet_add(fp, (uint8_t*)cp, 4);
    cp->param = param;
    for(i=0; i<cp->param_cnt; i++)
    {
        kptl_frame_packet_add(fp, (uint8_t *)&cp->param[i], sizeof(uint32_t));
    }
    
    kptl_frame_packet_final(fp);
}

uint32_t kptl_cmd_packet_get_size(cmd_packet_t *cp)
{
    return 4 + cp->param_cnt*sizeof(uint32_t); 
}

uint32_t kptl_create_generic_resp_packet(frame_packet_t *fp, uint32_t status_code, uint32_t cmd_tag)
{
    uint32_t param[2];
    cmd_packet_t cp;
    
    cp.tag = kCommandTag_GenericResponse;
    cp.flags = 0x00;
    cp.param_cnt = 2;
    cp.reserved = 0x00;
    
    param[0] = status_code;
    param[1] = cmd_tag;
    kptl_create_cmd_packet(fp, &cp, param);

    return CH_OK;
}

uint32_t kptl_create_property_resp_packet(frame_packet_t *p, uint8_t param_cnt, uint32_t *param)
{
    cmd_packet_t cp;
    cp.tag = kCommandTag_GetPropertyResponse;
    cp.flags = 0x00;
    cp.reserved = 0x00;
    cp.param_cnt = param_cnt;

    kptl_create_cmd_packet(p, &cp, param);

    return CH_OK;
}

void kptl_create_ping_resp_packet(ping_resp_packet_t *p, uint8_t major, uint8_t minor, uint8_t bugfix, uint8_t opt_low, uint8_t opt_high)
{
    p->hr.start_byte = kFramingPacketStartByte;
    p->hr.packet_type = kFramingPacketType_PingResponse;
    
    p->bug_fix = bugfix;
    p->ver_minor = minor;
    p->ver_major  = major;
    p->protocol_name = 'P';
    p->option_low = opt_low;
    p->option_high = opt_high;
    
    /* crc */
    uint16_t crc;
    crc = 0;
    crc16_update(&crc, (uint8_t*)&p->hr, 8);
    p->crc16[0] = (crc & 0x00FF)>>0;
    p->crc16[1] = (crc & 0xFF00)>>8;
}

uint32_t kptl_frame_packet_begin(frame_packet_t *p, uint8_t frame_type)
{
    p->hr.start_byte = kFramingPacketStartByte;
    p->hr.packet_type = frame_type;
    p->len[0] = 0;
    p->len[1] = 0;
    p->crc16[0] = 0;
    p->crc16[1] = 0;
    memset(p->payload, 0, sizeof(p->payload));
    return CH_OK;
}

uint32_t kptl_frame_packet_add(frame_packet_t *p, uint8_t *buf, uint16_t len)
{
    /* add item content into buffer */
    if(ARRAY2INT16(p->len) > MAX_PACKET_LEN)
    {
        return CH_ERR;
    }
    
    memcpy(p->payload + ARRAY2INT16(p->len), buf, len);
    p->len[0] += (len >>0) & 0xFF;
    p->len[1] += (len >>8) & 0xFF;
    return CH_OK;
}

uint32_t kptl_frame_packet_final(frame_packet_t *p)
{
    
    /* crc */
    uint16_t crc;
    crc = 0;
    crc16_update(&crc, (uint8_t*)&p->hr, 2);
    crc16_update(&crc, (uint8_t*)p->len, 2);
    crc16_update(&crc, (uint8_t*)p->payload, ARRAY2INT16(p->len));
    
    p->crc16[0] = (crc & 0x00FF) >> 0;
    p->crc16[1] = (crc & 0xFF00) >> 8;
    return CH_OK;
}

uint32_t kptl_frame_packet_get_size(frame_packet_t *p)
{
    return p->len[0] + (p->len[1]<<8) + 6;
}

enum status
{
    kStatus_Idle,
    kStatus_Cmd,
    kStatus_LenLow,
    kStatus_LenHigh,
    kStatus_CRCLow,
    kStatus_CRCHigh,
    kStatus_Data,
};

 /**
 * @brief  初始化姿态解码模块
 * @note   完成初始化一个引脚配置
 * @param  pkt 接收包指针
 * @param  接收成功回调函数
 * @code

 *      void OnDataReceived(frame_packet_t *pkt)
 *      {
 *          pkt->payload 为数据 pkt->payload_len 为接收到的字节长度 
 *      }
 *
 *      frame_packet_t pkt;
 *      Packet_DecodeInit(&pkt, OnDataReceived);
 * @endcode
 * @retval None
 */
int kptl_decode_init(pkt_dec_t *d)
{
    d->cnt = 0;
    d->status = kStatus_Idle;
    if(!d->fp)
    {
        return 1;
    }
    memset(d->fp, 0, sizeof(frame_packet_t));
    return 0;
}

#define SAFE_CALL_CB    if(d->cb) d->cb(p)
    
 /**
 * @brief  decode any type of packet
 * @note   once a pcaket is decoded, the cb function will be called
 * @param  d: decode handle, c: one byte received
 * @retval CH_OK
 */

uint32_t kptl_decode(pkt_dec_t *d, uint8_t c)
{
    int ret = CH_ERR;
    uint16_t crc_calculated = 0;          /* CRC value caluated from a frame */
    frame_packet_t *p = d->fp;
    uint8_t *payload_buf = (uint8_t*)d->fp->payload;
    
    switch(d->status)
    {
        case kStatus_Idle:
            if(c == kFramingPacketStartByte)
            {
                d->status = kStatus_Cmd;
                p->hr.start_byte = c;
            }
            break;
        case kStatus_Cmd:
            p->hr.packet_type = c;
            switch(c)
            {
                case kFramingPacketType_Command:
                    d->status = kStatus_LenLow;
                    break;
                case kFramingPacketType_Data:
                    d->status = kStatus_LenLow;
                    break;
                case kFramingPacketType_Ping:
                    SAFE_CALL_CB;
                    ret = CH_OK;
                    d->status = kStatus_Idle;
                    break;
                case kFramingPacketType_PingResponse:
                    p->len[0] = 0;
                    p->len[1] = 0;
                    d->cnt = 0;
                    d->status = kStatus_Data;
                    break;
                case kFramingPacketType_Ack:
                case kFramingPacketType_Nak:
                    d->status = kStatus_Idle;
                    SAFE_CALL_CB;
                    return CH_OK;
            }
            break;
        case kStatus_LenLow:
            p->len[0] = c;
            d->status = kStatus_LenHigh;
            break;
        case kStatus_LenHigh:
            p->len[1] = c;
            if(ARRAY2INT16(p->len) <= MAX_PACKET_LEN)
            {
                d->status = kStatus_CRCLow;
            }
            else
            {
                d->status = kStatus_Idle;
            }
            break;
        case kStatus_CRCLow:
            p->crc16[0] = c;
            d->status = kStatus_CRCHigh;
            break;
        case kStatus_CRCHigh:
            p->crc16[1] = c;
            d->cnt = 0;
            d->status = kStatus_Data;
            break;
        case kStatus_Data:
            payload_buf[d->cnt++] = c;
                   
            if((p->hr.packet_type == kFramingPacketType_Command || p->hr.packet_type == kFramingPacketType_Data) && d->cnt >= ARRAY2INT16(p->len))
            {
                /* calculate CRC */
                crc_calculated = 0;
                crc16_update(&crc_calculated, (uint8_t*)&p->hr, 2);
                crc16_update(&crc_calculated, p->len, 2);
                crc16_update(&crc_calculated, payload_buf, d->cnt);
                /* CRC match */
                if(crc_calculated == ARRAY2INT16(p->crc16))
                {
                    SAFE_CALL_CB;
                    ret = CH_OK;
                }
                d->status = kStatus_Idle;
            }
            
            if(p->hr.packet_type == kFramingPacketType_PingResponse && d->cnt >= 8) /* ping response */
            {
                p->len[0] = 8;
                p->len[1] = 0;
                SAFE_CALL_CB;
                d->status = kStatus_Idle;
            }
            
            break;
        default:
            d->status = kStatus_Idle;
            break;
    }
    return ret;
}

