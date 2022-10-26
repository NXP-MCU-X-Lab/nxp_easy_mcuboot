/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
 
#include "mcuboot.h"
#include <string.h>

static int evt = 0;

static void handle_cmd(mcuboot_t *ctx, frame_packet_t *pkt)
{
    packet_ack_t ack;
    cmd_packet_t rx_cp;
    uint32_t tx_param[7];
    uint32_t rx_param[7];
    uint8_t tx_param_cnt = 0;
   
    memcpy(&rx_cp, pkt->payload, 4);
    memcpy(rx_param, &pkt->payload[4], rx_cp.param_cnt*sizeof(uint32_t));
    rx_cp.param = rx_param;
    
    /* reply ack */
    kptl_create_ack(&ack);
    ctx->op_send((uint8_t*)&ack, sizeof(ack));
    
    switch(rx_cp.tag)
    {
        case kCommandTag_GetProperty:
            tx_param[0] = 0x00000000;
            switch(rx_cp.param[0])
            {
                case 0x01:  /* GetCurrentVersion */
                    tx_param[1] = 0x4b010400;
                    tx_param_cnt = 2;
                    break;
                case 0x02:  /* available periperals */
                    tx_param[1] = 0x00000001;
                    tx_param_cnt = 2;
                    break;
                case 0x03:  /* start of the flash */
                    tx_param[1] = ctx->cfg_flash_start;
                    tx_param_cnt = 2;
                    break;
                case 0x04:  /* flash size */
                    tx_param[1] = ctx->cfg_flash_size;
                    tx_param_cnt = 2;
                    break;
                case 0x05:  /* flash sector size */
                    tx_param[1] = ctx->cfg_flash_sector_size;
                    tx_param_cnt = 2;
                    break;
                case 0x06:  /* flash block count */
                    tx_param[1] = 1;
                    tx_param_cnt = 2;
                    break;
                case 0x07:  /* avaiable command */
                    tx_param[1] =  0xFFFF;
                    tx_param_cnt = 2;
                    break;
                case 0x0B:  /* MaxPacketSize, <=512 */
					tx_param[1] = MAX_PACKET_LEN<512?MAX_PACKET_LEN:512;
                    tx_param_cnt = 2;
                    break;
                case 0x0C:  /* ReservedRegions */
                    tx_param[1] = 0;
                    tx_param[2] = 0;
                    tx_param[3] = 0;
                    tx_param[4] = 0;
                    tx_param_cnt = 5;
                    break;
                case 0x0E:  /* RAMStartAddress */
                    tx_param[1] = ctx->cfg_ram_start;
                    tx_param_cnt = 2;
                    break;
                case 0x0F:  /* RAMSizeInBytes */
                    tx_param[1] = ctx->cfg_ram_size;
                    tx_param_cnt = 2;
                    break;
                case 0x10:  /* device id */
                    tx_param[1] = ctx->cfg_device_id;
                    tx_param_cnt = 2;
                    break;
                case 0x11:  /* security state */
                    tx_param[1] = 0;
                    tx_param_cnt = 2;
                    break;
                case 0x12:  /* uuid */
                    tx_param[1] = ctx->cfg_uuid;
                    tx_param_cnt = 2;
                    break;
                default:
                    /* not supported */
                    //printf("command not supported\r\n");
                    break;
            }
            
            kptl_create_property_resp_packet(&ctx->tx_pkt, tx_param_cnt, tx_param);
            ctx->op_send((uint8_t*)&ctx->tx_pkt, kptl_frame_packet_get_size(&ctx->tx_pkt));
            break;
        case kCommandTag_FlashEraseRegion:
            ctx->mem_start_addr = rx_cp.param[0];
            ctx->mem_len = rx_cp.param[1];
            ctx->op_mem_erase(ctx->mem_start_addr, ctx->mem_len);
            kptl_create_generic_resp_packet(&ctx->tx_pkt, 0, kCommandTag_FlashEraseRegion);
            ctx->op_send((uint8_t*)&ctx->tx_pkt, kptl_frame_packet_get_size(&ctx->tx_pkt));
            break;
        case kCommandTag_FlashEraseAll: /* not support */
            kptl_create_generic_resp_packet(&ctx->tx_pkt, 0, kCommandTag_FlashEraseAll);
            ctx->op_send((uint8_t*)&ctx->tx_pkt, kptl_frame_packet_get_size(&ctx->tx_pkt));
            break;
        case kCommandTag_WriteMemory:
            ctx->mem_start_addr = rx_cp.param[0];
            ctx->mem_len = rx_cp.param[1];
            ctx->mem_cur_addr = ctx->mem_start_addr;

            kptl_create_generic_resp_packet(&ctx->tx_pkt, 0x00000000, kCommandTag_WriteMemory);
            ctx->op_send((uint8_t*)&ctx->tx_pkt, kptl_frame_packet_get_size(&ctx->tx_pkt));
            break;
        case kCommandTag_Reset:
            kptl_create_generic_resp_packet(&ctx->tx_pkt, 0x00000000, kCommandTag_Reset);
            ctx->op_send((uint8_t*)&ctx->tx_pkt, kptl_frame_packet_get_size(&ctx->tx_pkt));
            ctx->op_reset();
            break;
        case kCommandTag_Execute:
            kptl_create_generic_resp_packet(&ctx->tx_pkt, 0x00000000, kCommandTag_Execute);
            ctx->op_send((uint8_t*)&ctx->tx_pkt, kptl_frame_packet_get_size(&ctx->tx_pkt));
        
            uint32_t addr, arg, sp;
        
            addr = rx_cp.param[0];
            arg = rx_cp.param[1];
            sp = rx_cp.param[2];
        
            ctx->op_jump(addr, arg, sp);
            break;
        default:
            //printf("unknown command tag:0x%X\r\n", rx_cp.param[0]);
            break;
    }
}

static void dec_cb(frame_packet_t *rx)
{
    evt = 1;
}

uint32_t mcuboot_is_connected(mcuboot_t *ctx)
{
    return ctx->is_connected;
}

void mcuboot_proc(mcuboot_t *ctx)
{
    if(evt)
    {
        ctx->is_connected = 1;
        switch(ctx->rx_pkt.hr.packet_type)
        {
            case kFramingPacketType_Ping:
            {
                ping_resp_packet_t pr;
                kptl_create_ping_resp_packet(&pr, 1, 2, 0, 0, 0);
                ctx->op_send((uint8_t*)&pr, sizeof(ping_resp_packet_t));
                break;
            }
            case kFramingPacketType_Command:
            {
                handle_cmd(ctx, &ctx->rx_pkt);
                break;
            }

            case kFramingPacketType_Data:
            {
                packet_ack_t ack;
    
                int len;
                len = ARRAY2INT16(ctx->rx_pkt.len);
                
                ctx->op_mem_write(ctx->mem_cur_addr, ctx->rx_pkt.payload, len);
                ctx->mem_cur_addr += len;
                
                /* reply ack */
                kptl_create_ack(&ack);
                ctx->op_send((uint8_t*)&ack, sizeof(ack));
                
                /* send final generic resp packet */
                
                if(ctx->mem_cur_addr >= (ctx->mem_start_addr + ctx->mem_len))
                {
                    kptl_create_generic_resp_packet(&ctx->tx_pkt, 0x00000000, kCommandTag_WriteMemory);
                    ctx->op_send((uint8_t*)&ctx->tx_pkt, kptl_frame_packet_get_size(&ctx->tx_pkt));
                    
                    /* callback: complete */
                    ctx->op_complete();
                }
                break;
            }
            case kFramingPacketType_Ack:
                break;
            case kFramingPacketType_Nak:
                break;
            default:
                break;
        }
        evt = 0;
    }
}

void mcuboot_recv(mcuboot_t *ctx, uint8_t *buf, uint32_t len)
{
    int i;
    
    for(i=0; i<len; i++)
    {
        kptl_decode(&ctx->dec, buf[i]);
    }
}

void mcuboot_init(mcuboot_t *ctx)
{
    ctx->dec.fp = &ctx->rx_pkt;
    ctx->dec.cb = dec_cb;
    kptl_decode_init(&ctx->dec);
    ctx->is_connected = 0;
    evt = 0;
}

