/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
 
#ifndef __MCUBOOT_H__
#define __MCUBOOT_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "kptl.h"

typedef struct
{
    /* packet handing resource */
	  uint8_t reservedrx[2];//To make sure payload array in frame_packet is 4bytes aligned
    frame_packet_t rx_pkt;
	  uint8_t reservedtx[2];//To make sure payload array in frame_packet is 4bytes aligned
    frame_packet_t tx_pkt;   
   	pkt_dec_t dec;
    /* transmit callback */
    int (*op_send)(uint8_t* buf, uint32_t len);
    
    /* configuartion */
    uint32_t cfg_flash_start;
    uint32_t cfg_flash_size;
    uint32_t cfg_flash_sector_size;
    uint32_t cfg_ram_start;
    uint32_t cfg_ram_size;
    uint32_t cfg_device_id;
    uint32_t cfg_uuid;
    
    /* memory operation */
    int (*op_mem_write)(uint32_t addr, uint8_t* buf, uint32_t len);
    int (*op_mem_erase)(uint32_t addr, uint32_t len);
    int (*op_mem_read)(uint32_t addr, uint8_t* buf, uint32_t len);
    void(*op_reset)(void);
    void(*op_jump)(uint32_t addr, uint32_t arg, uint32_t sp);
    void(*op_complete)(void);
    
    /* mcu boot private resource */
    uint32_t mem_start_addr;
    uint32_t mem_len;
    uint32_t mem_cur_addr;
    uint32_t is_connected;
}mcuboot_t;


void mcuboot_init(mcuboot_t *ctx);
void mcuboot_recv(mcuboot_t *ctx, uint8_t *buf, uint32_t len);
void mcuboot_proc(mcuboot_t *ctx);
uint32_t mcuboot_is_connected(mcuboot_t *ctx);


#ifdef __cplusplus
}
#endif

#endif

