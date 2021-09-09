 #ifndef __HID_MCUBOOT_H__
#define __HID_MCUBOOT_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "usbd_hid.h"

#define MAX_HID_BUF_SIZE    (64)



typedef struct 
{
    uint8_t reportID; //!< The report ID.
    uint8_t _padding; //!< Pad byte necessary so that the length is 2-byte aligned and the data is 4-byte aligned. Set  //! to zero.
    uint8_t packetLengthLsb; //!< Low byte of the packet length in bytes.
    uint8_t packetLengthMsb; //!< High byte of the packet length in bytes.
}hid_hdr_t;

typedef struct 
{
    hid_hdr_t   header; 
    uint8_t     buf[MAX_HID_BUF_SIZE];
}hid_report_t;

typedef struct
{
    /* buf */
    hid_report_t _rx_report;
    
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
}hid_mcuboot_t;





void hid_mcuboot_init(hid_mcuboot_t *ctx);
uint32_t hid_mcuboot_is_connected(hid_mcuboot_t *ctx);
void hid_mcuboot_proc(hid_mcuboot_t *ctx);





#ifdef __cplusplus
}
#endif

#endif

