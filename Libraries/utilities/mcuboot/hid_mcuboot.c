#include <stdint.h>
#include <string.h>
#include "hid_mcuboot.h"
#include "bl_cfg.h"


static hid_mcuboot_t *g_ctx;

#define  HID_MCUBOOT_DEBUG  (1)

#if ( HID_MCUBOOT_DEBUG == 1 )
#define HID_MCUBOOT_TRACE	printf
#else
#define HID_MCUBOOT_TRACE(...)
#endif

/* USB HID report ID */
#define REPORT_ID_CMD_OUT       (1)
#define REPORT_ID_DATA_OUT      (2)
#define REPORT_ID_CMD_IN        (3)
#define REPORT_ID_DATA_IN       (4)


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

typedef struct
{
    uint8_t tag;            //!< A command tag.
    uint8_t flags;          //!< Combination of packet flags.
    uint8_t reserved;       //!< Reserved, helpful for alignment, set to zero.
    uint8_t param_cnt;      //!< Number of parameters that follow in buffer.
    uint32_t *param;
} cmd_t;


static void hid_send_cmd_report(hid_report_t *report)
{
    cmd_t *cp = (cmd_t*)&report->buf[0];
    report->header.reportID = REPORT_ID_CMD_IN;
    
    uint32_t report_len = sizeof(cmd_t) + cp->param_cnt*sizeof(uint32_t);
    report->header.packetLengthLsb = (report_len>>0) & 0xFF;
    report->header.packetLengthMsb = (report_len>>8) & 0xFF;
    usbd_ep_write(USBD_HID2_EP_INTIN, (uint8_t*)report, MAX_HID_BUF_SIZE);
}

static void setup_property_resp_packet(uint8_t param_cnt, uint32_t *param)
{
    hid_report_t report;
    
    cmd_t *cp = (cmd_t*)&report.buf[0];
    cp->tag = kCommandTag_GetPropertyResponse;
    cp->flags = 0x00;
    cp->reserved = 0x00;
    cp->param_cnt = param_cnt;

    memcpy(&report.buf[4], param, param_cnt*sizeof(uint32_t));
    
    hid_send_cmd_report(&report);
}

static void setup_generic_resp_packet(uint32_t status_code, uint32_t cmd_tag)
{
    hid_report_t report;
    uint32_t param[2];
    
    cmd_t *cp = (cmd_t*)&report.buf[0];
    cp->tag = kCommandTag_GenericResponse;
    cp->flags = 0x00;
    cp->reserved = 0x00;
    cp->param_cnt = 2;

    param[0] = status_code;
    param[1] = cmd_tag;
    memcpy(&report.buf[4], param, cp->param_cnt*sizeof(uint32_t));
    
    hid_send_cmd_report(&report);
}



static void hid_handle_cmd(hid_mcuboot_t *ctx, uint8_t *buf, uint32_t len)
{
    cmd_t *cmd = (cmd_t*)buf;

    uint8_t tx_param_cnt = 0;
    uint32_t tx_param[7];
    uint32_t rx_param[7];
   
    memcpy(rx_param, buf+4, cmd->param_cnt*sizeof(uint32_t));
//    HID_MCUBOOT_TRACE("tag:%d param_cnt:%d param[0]:%d\r\n", cmd->tag, cmd->param_cnt, rx_param[0]);
    switch(cmd->tag)
    {
        case kCommandTag_GetProperty:
            tx_param[0] = 0x00000000;
            switch(rx_param[0])
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
					tx_param[1] = 32;
                    tx_param_cnt = 2;
                    break;
                case 0x0C:  /* ReservedRegions */
                    tx_param[1] = 0;
                    tx_param[2] = APPLICATION_BASE & 0x0FFFFFFF;
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
                    //HID_MCUBOOT_TRACE("command not supported:%d\r\n", rx_param[0]);
                    break;
            }
            
            setup_property_resp_packet(tx_param_cnt, tx_param);
            break;
        case kCommandTag_FlashEraseRegion:
            ctx->mem_start_addr = rx_param[0];
            ctx->mem_len = rx_param[1];
            HID_MCUBOOT_TRACE("kCommandTag_FlashEraseRegion 0x%X 0x%X\r\n", ctx->mem_start_addr, ctx->mem_len);
            ctx->op_mem_erase(ctx->mem_start_addr, ctx->mem_len);
            setup_generic_resp_packet(0, kCommandTag_FlashEraseRegion);
            break;
        case kCommandTag_FlashEraseAll: /* not support */
            HID_MCUBOOT_TRACE("kCommandTag_FlashEraseAll\r\n");
            setup_generic_resp_packet( 0, kCommandTag_FlashEraseAll);
            break;
        case kCommandTag_WriteMemory:
            HID_MCUBOOT_TRACE("kCommandTag_WriteMemory\r\n");
            ctx->mem_start_addr = rx_param[0];
            ctx->mem_len = rx_param[1];
            ctx->mem_cur_addr = ctx->mem_start_addr;
            setup_generic_resp_packet(0, kCommandTag_WriteMemory);
            break;
        case kCommandTag_Reset:
            HID_MCUBOOT_TRACE("kCommandTag_Reset\r\n");
            setup_generic_resp_packet(0x00000000, kCommandTag_Reset);
            ctx->op_reset();
            break;
        case kCommandTag_Execute:
            HID_MCUBOOT_TRACE("kCommandTag_Execute\r\n");
            setup_generic_resp_packet(0x00000000, kCommandTag_Execute);
        
            uint32_t addr, arg, sp;
        
            addr = rx_param[0];
            arg = rx_param[1];
            sp = rx_param[2];
        
            ctx->op_jump(addr, arg, sp);
            break;
        default:
            HID_MCUBOOT_TRACE("unknown command tag:0x%X\r\n", cmd->tag);
            break;
    }
    
}


uint32_t hid_mcuboot_is_connected(hid_mcuboot_t *ctx)
{
    return ctx->is_connected;
}


void hid_mcuboot_proc(hid_mcuboot_t *ctx)
{
    if(1)
    {
        int len = (ctx->_rx_report.header.packetLengthLsb<<0) + (ctx->_rx_report.header.packetLengthMsb<<8);
        switch(ctx->_rx_report.header.reportID)
        {
            case REPORT_ID_CMD_OUT:
                hid_handle_cmd(ctx, ctx->_rx_report.buf, len);
                break;
            case REPORT_ID_DATA_OUT:
                ctx->op_mem_write(ctx->mem_cur_addr, ctx->_rx_report.buf, len);
                ctx->mem_cur_addr += len;
                            
                hid_report_t tx_report; 
                
                /* send final generic resp packet */
                if(ctx->mem_cur_addr >= (ctx->mem_start_addr + ctx->mem_len))
                {
                    setup_generic_resp_packet(0x00000000, kCommandTag_WriteMemory);
                    
                    /* callback: complete */
                    ctx->op_complete();
                }
                break;
            default:
                HID_MCUBOOT_TRACE("unknow report ID\r\n");
                break;            
        }
    }
    
}

/* IN TOKEN: new HID data can be send (this fun is in interrupt) */
uint32_t hid_send_ntf_cb(uint8_t ep)
{
//    HID_MCUBOOT_TRACE("%s\r\n", __FUNCTION__);
    return 0;
}

/* OUT TOKEN: new HID data recvied (this fun is in interrupt) */
static uint32_t hid_recv_cb(uint8_t ep, uint8_t *buf, uint32_t len)
{
    g_ctx->is_connected = 1;
    memcpy(&g_ctx->_rx_report, buf, len);
    hid_mcuboot_proc(g_ctx);
    return 0;
}

struct usbd_hid_callback_t hid_cb = 
{
    hid_send_ntf_cb,
    hid_recv_cb,
};


void hid_mcuboot_init(hid_mcuboot_t *ctx)
{
    ctx->is_connected = 0;
    g_ctx = ctx;
}
