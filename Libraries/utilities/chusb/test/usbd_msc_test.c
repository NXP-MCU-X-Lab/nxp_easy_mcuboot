#include <string.h>

#include "usbd_msc.h"

extern struct usbd_t usbd;


#define BLOCK_SIZE      (512)
#define BLOCK_CNT       (100)
uint8_t DiskBuf[BLOCK_SIZE*BLOCK_CNT];

uint32_t msc_read_sector (uint32_t block, uint8_t *buf, uint32_t block_cnt)
{
   // printf("r block:%d cnt:%d\r\n", block, block_cnt);
    memcpy(buf, DiskBuf + block*BLOCK_SIZE, block_cnt*BLOCK_SIZE);
    return 0;
}

uint32_t msc_write_sector(uint32_t block, uint8_t *buf, uint32_t block_cnt)
{
    //printf("w block:%d cnt:%d buf:%X\r\n", block, block_cnt, buf);
    memcpy(DiskBuf + block*BLOCK_SIZE, buf, BLOCK_SIZE*block_cnt);
    return 0;
}

uint32_t msc_get_disk_info(uint32_t *total_block_cnt, uint32_t *block_size)
{
    *total_block_cnt = BLOCK_CNT;
    *block_size = BLOCK_SIZE;
    return 0;
}

struct usbd_msc_callback_t msc_cb = 
{
    msc_read_sector,
    msc_write_sector,
    msc_get_disk_info,
};

void usbd_msc_test(struct usbd_t *h)
{
    usbd_msc_set_cb(&msc_cb);
    usbd_msc_init(h);
}
