#include <string.h>

#include "usb_common.h"
#include "usbd.h"
#include "usbd_msc.h"

/* MSC state machine */
enum STAT
{
    STAT_CBW,           /* receive CBW block */
    STAT_CMD,           /* process cmd */
    STAT_CSW,           /* send CSW */
    STAT_RECEIVE,       /* receive data */
    STAT_SEND,          /* send data */
};

struct ustorage
{
    struct usbd_t *h;              /* usbd handle */
    uint32_t total_block_cnt;           /* total block count */
    uint32_t block_size;                /* how many bytes in one block */
    uint32_t t_start_addr;              
    uint32_t t_total_cnt;      
    uint32_t t_current_offset;
    uint32_t t_current_cnt;             
    struct ustorage_cbw cbw;            /* cbw struct */
    struct ustorage_csw csw;            /* csw struct */
    uint8_t buf[MSD_BUF_SIZE];          /* data buffer */
    int status;                         /* state machine */
    struct usbd_msc_callback_t *cb; /* callback */
};

struct ustorage msc;


void usbd_msc_set_cb(struct usbd_msc_callback_t *cb)
{
    msc.cb = cb;
}

uint32_t msc_class_request_handler(struct usbd_t *h)
{
    uint8_t lun[1];
    
    if(h->setup.index == USBD_MSC_IF_IDX)
    {
        USBD_TRACE("class request send to msc interface:%d\r\n", USBD_MSC_IF_IDX);
        switch(h->setup.request)
        {
            case USBREQ_GET_MAX_LUN:
                
                /* get disk info */
                msc.cb->msc_get_disk_info(&msc.total_block_cnt, &msc.block_size);
            
                /* disk cnt = 1 */
                USBD_TRACE("USBREQ_GET_MAX_LUN\r\n");
                lun[0] = 0;
                begin_data_in_stage(lun, 1);
                msc.status = STAT_CBW;
                break;
            case USBREQ_MASS_STORAGE_RESET:  /* set control line state */
                usbd_status_in_stage();
                USBD_TRACE("USBREQ_MASS_STORAGE_RESET\r\n");
                break;
            default:
                USBD_TRACE("MSC UNKOWN REQUEST\r\n");
                break;
        }
    }
    
    return CH_OK;
}

uint32_t msc_standard_request_to_intf_handler(struct usbd_t *h)
{
    USBD_TRACE("msc_standard_request_to_intf_handler\r\n");
    return CH_OK;
}

static void _send_status(void)
{
    msc.csw.tag = msc.cbw.tag;
    msc.csw.signature = CSW_SIGNATURE;
    msc.csw.data_reside = 0x00;
    msc.csw.status = 0x00;
    
    usbd_ep_write(USBD_MSC_EP_BULKIN, (uint8_t*)&msc.csw, SIZEOF_CSW);
}

static void cbw_dump(struct ustorage_cbw* cbw)
{
    USBD_TRACE("signature 0x%x\r\n", cbw->signature);
    USBD_TRACE("tag 0x%x\r\n", cbw->tag);
    USBD_TRACE("xfer_len %d\r\n", cbw->xfer_len);
    USBD_TRACE("dflags 0x%x\r\n", cbw->dflags);
    USBD_TRACE("lun 0x%x\r\n", cbw->lun);
    USBD_TRACE("cb_len %d\r\n", cbw->cb_len);
    USBD_TRACE("cb[0] 0x%x\r\n", cbw->cb[0]);
}

static void _inquiry_cmd (void)
{
    USBD_TRACE("inquiry\r\n");
    
    msc.buf[0] = 0x00;             /* Direct Access Device */
    msc.buf[1] = 0x80;             /* RMB = 1: Removable Medium */
    msc.buf[2] = 0x02;             /* Version: ANSI X3.131: 1994 */
    msc.buf[3] = 0x02;

    msc.buf[4] = 32;             /* Additional Length */
    msc.buf[5] = 0x00;             /* SCCS = 0: No Storage Controller Component */
    msc.buf[6] = 0x00;
    msc.buf[7] = 0x00;

    memset(&msc.buf[8], ' ', 28);
    memcpy(&msc.buf[8], "CHUSB", 5);
    memcpy(&msc.buf[16], "DISK", 4);
    
    usbd_ep_write(USBD_MSC_EP_BULKIN, msc.buf, 36);
}

static void _read_capacities(void)
{
    USBD_TRACE("read capacities\r\n");
        
    msc.buf[0] = 0x00;
    msc.buf[1] = 0x00;
    msc.buf[2] = 0x00;
    msc.buf[3] = 0x08; /* list length */
    
    /* block count */
    msc.buf[4] = (msc.total_block_cnt >> 24) & 0xFF;
    msc.buf[5] = (msc.total_block_cnt >> 16) & 0xFF;
    msc.buf[6] = (msc.total_block_cnt >>  8) & 0xFF;
    msc.buf[7] = (msc.total_block_cnt >>  0) & 0xFF;
    
    /* block size */
    msc.buf[8] = 0x02;                      /* Descriptor Code: Formatted Media */
    msc.buf[9] = (msc.block_size  >> 16) & 0xFF;
    msc.buf[10] = (msc.block_size  >>  8) & 0xFF;
    msc.buf[11] = (msc.block_size  >>  0) & 0xFF;

    usbd_ep_write(USBD_MSC_EP_BULKIN, msc.buf, 12);
}

static void _read_capacity(void)
{
    /* Last Logical Block */
    msc.buf[0] = ((msc.total_block_cnt - 1) >> 24) & 0xFF;
    msc.buf[1] = ((msc.total_block_cnt - 1) >> 16) & 0xFF;
    msc.buf[2] = ((msc.total_block_cnt - 1) >>  8) & 0xFF;
    msc.buf[3] = ((msc.total_block_cnt - 1) >>  0) & 0xFF;

    /* Block Length */
    msc.buf[4] = (msc.block_size >> 24) & 0xFF;
    msc.buf[5] = (msc.block_size >> 16) & 0xFF;
    msc.buf[6] = (msc.block_size >>  8) & 0xFF;
    msc.buf[7] = (msc.block_size >>  0) & 0xFF;

    usbd_ep_write(USBD_MSC_EP_BULKIN, msc.buf, 8);
}

static void _read_10(void)
{   
    msc.t_start_addr = msc.cbw.cb[2]<<24 | msc.cbw.cb[3]<<16 | msc.cbw.cb[4]<<8 | msc.cbw.cb[5]<<0;
    msc.t_total_cnt = msc.cbw.cb[7]<<8 | msc.cbw.cb[8]<<0;
    
    msc.t_current_offset = 0;
    msc.t_current_cnt = 0;
    //USBD_TRACE("_read10 block:%d count:%d\r\n", msc.t_start_addr, msc.t_total_cnt);
}

static void _write_10(void)
{
    msc.t_start_addr = msc.cbw.cb[2]<<24 | msc.cbw.cb[3]<<16 | msc.cbw.cb[4]<<8 | msc.cbw.cb[5]<<0;
    msc.t_total_cnt = msc.cbw.cb[7]<<8 | msc.cbw.cb[8]<<0;
    
    msc.t_current_offset = 0;
    msc.t_current_cnt = 0;
    USBD_TRACE("WRITE10 addr:%d cnt:%d\r\n", msc.t_start_addr, msc.t_total_cnt);
}

static void _memory_read(void)
{
    uint32_t buck_len;
    uint32_t block_cnt; /* how many block r/w in callback function, msc_read_sector */
    
    if((msc.t_total_cnt - msc.t_current_cnt) > (sizeof(msc.buf)/msc.block_size))
    {
        block_cnt = (sizeof(msc.buf)/msc.block_size);
    }
    else
    {
        block_cnt =  msc.t_total_cnt - msc.t_current_cnt;
    }

    if(msc.t_current_offset == 0 || msc.t_current_offset == (block_cnt*msc.block_size))
    {
        msc.cb->msc_read_sector(msc.t_start_addr + msc.t_current_cnt, msc.buf, block_cnt);
        msc.t_current_cnt += block_cnt;
        msc.t_current_offset = 0;
    }

    (msc.csw.data_reside > MSD_EP_SIZE)?(buck_len = MSD_EP_SIZE):(buck_len = msc.csw.data_reside);

    //USBD_TRACE("read meory: current_ofs:%d buck_len:%d %d data_reside: %d\r\n", msc.t_current_offset, buck_len, msc.t_current_cnt, msc.csw.data_reside);

    usbd_ep_write(USBD_MSC_EP_BULKIN, msc.buf + msc.t_current_offset, buck_len);

    msc.t_current_offset += buck_len;
    msc.csw.data_reside -= buck_len;
}


static void _memory_write(void)
{
    uint32_t buck_len;
    uint32_t write_cnt;

    uint32_t buf_block_cnt = sizeof(msc.buf)/msc.block_size;
    
    if((msc.t_total_cnt - msc.t_current_cnt) > buf_block_cnt)
    {
        write_cnt = buf_block_cnt;
    }
    else
    {
        write_cnt =  msc.t_total_cnt - msc.t_current_cnt;
    }
    
    buck_len = usbd_ep_read(USBD_MSC_EP_BULKOUT, msc.buf + msc.t_current_offset);
    
    //USBD_TRACE("we  len:%d write_cnt:%d ofs:%d\r\n", buck_len, write_cnt, msc.t_current_offset);
    
    msc.csw.data_reside -= buck_len;
    msc.t_current_offset += buck_len;
    
    if(msc.t_current_offset == write_cnt*msc.block_size)
    {
        msc.cb->msc_write_sector(msc.t_start_addr + msc.t_current_cnt, msc.buf, write_cnt);
        msc.t_current_cnt += write_cnt;
        msc.t_current_offset = 0;
    }
}

static void _mode_sense_6(void)
{
    //USBD_TRACE("_mode_sense_6\r\n");
    msc.buf[0] = 3;
    msc.buf[1] = 0;
    msc.buf[2] = 0;
    msc.buf[3] = 0;
    usbd_ep_write(USBD_MSC_EP_BULKIN, msc.buf, SIZEOF_MODE_SENSE_6);
}

static void _log_sense(void)
{
    USBD_TRACE("_log_sense\r\n");
    msc.buf[0] = 0;
    msc.buf[1] = 0;
    msc.buf[2] = 0;
    msc.buf[3] = 0;
    usbd_ep_write(USBD_MSC_EP_BULKIN, msc.buf, 4);
}

uint32_t msc_data_ep_handler(uint8_t ep, uint8_t dir)
{
    uint32_t size;
    if(ep == USBD_MSC_EP_BULKOUT && dir == 0) /* OUT TOKEN */
    {
        switch(msc.status)
        {
            case STAT_CBW:
                size = usbd_ep_read(ep, (uint8_t*)&msc.cbw);
                if(size != SIZEOF_CBW)
                {
                    USBD_TRACE("size of CBW error! %d != %d \r\n", size, SIZEOF_CBW);
                    while(1);
                }
                //cbw_dump(&msc.cbw);
                if(msc.cbw.signature == CBW_SIGNATURE)
                {
                    msc.csw.data_reside = msc.cbw.xfer_len;
                    msc.csw.tag = msc.cbw.tag;
                    switch (msc.cbw.cb[0])
                    {
                        case SCSI_INQUIRY_CMD:
                            _inquiry_cmd();
                            msc.status = STAT_CSW;
                            break;
                        case SCSI_READ_CAPACITIES:
                            _read_capacities();
                            msc.status = STAT_CSW;
                            break;
                        case SCSI_MODE_SENSE_6:
                            _mode_sense_6();
                            msc.status = STAT_CSW;
                            break;
                        case SCSI_READ_CAPACITY:
                        case 0x9E:
                            _read_capacity();
                            msc.status = STAT_CSW;
                            break;
                        case SCSI_READ_10: /* PC read data, IN transfer */
                            _read_10();
                            _memory_read();
                            if(msc.csw.data_reside == 0)
                            {
                                msc.status = STAT_CSW;
                            }
                            else
                            {
                                msc.status = STAT_SEND;
                            }
                            break;
                        case SCSI_WRITE_10: /* PC send data, out transfer */
                            _write_10();
                            msc.status = STAT_RECEIVE;
                            break;
                        case SCSI_TEST_UNIT_READY:
                            _send_status();
                            msc.status = STAT_CBW;
                            break;
                        case SCSI_ALLOW_REMOVAL:
                            _send_status();
                            msc.status = STAT_CBW;
                            break;
                        case 0x4D:
                            _log_sense();
                            msc.status = STAT_CSW;
                            break;
                        default:
                            USBD_TRACE("Unknown CBW command 0x%X\r\n", msc.cbw.cb[0]);
                            cbw_dump(&msc.cbw);
                            _send_status();
                            msc.status = STAT_CBW;
                            //while(1);
                            break;
                    }
                }
                break;
            case STAT_RECEIVE:
                _memory_write();
                if(msc.csw.data_reside == 0)
                {
                    _send_status();
                    msc.status = STAT_CBW;
                }
                break;
        }
    }
    if(ep == USBD_MSC_EP_BULKIN && dir == 1) /* IN TOKEN */
    {
        switch(msc.status)
        {
            case STAT_CMD:
                USBD_TRACE("STAT_CMD\r\n");
                break;
            case STAT_SEND:
                _memory_read();
                if(msc.csw.data_reside == 0)
                {
                    msc.status = STAT_CSW;
                }
                break;
            case STAT_CSW:
                _send_status();
                 msc.status = STAT_CBW;
                break;
            default:
                //USBD_TRACE("Unknwon In BuckIn event %d\r\n", msc.status);
                break;
        }
    }
    
    return CH_OK;
}

void usbd_msc_init(struct usbd_t *h)
{
    msc.h = h;
    uint8_t *p;
    struct uconfig_descriptor *uconfiguration_descriptor;
    desc_t d;
    
    
    if(msc.block_size > MSD_BUF_SIZE)
    {
        USBD_TRACE("MSD_BUF_SIZE must greater then msc.block_size");
        return;
    }
    
    get_descriptor_data("configuration_descriptor", &d);
    uconfiguration_descriptor = (struct uconfig_descriptor *)d.buf;
    
    uconfiguration_descriptor->bLength = USB_DESC_LENGTH_CONFIG;
    uconfiguration_descriptor->type = USB_DESC_TYPE_CONFIGURATION;
    uconfiguration_descriptor->wTotalLength = USB_DESC_LENGTH_CONFIG;
    uconfiguration_descriptor->bNumInterfaces = 1;
    uconfiguration_descriptor->bConfigurationValue = 1;
    uconfiguration_descriptor->iConfiguration = 0;
    uconfiguration_descriptor->bmAttributes = 0x80;
    uconfiguration_descriptor->MaxPower = 0x32;
    
    /* add configuation data */
    p = uconfiguration_descriptor->data;
    
    get_descriptor_data("msc_descriptor", &d);
    memcpy(p, d.buf, d.len);
    p += d.len;
    uconfiguration_descriptor->wTotalLength += d.len;
    
    /* install class callback */
    msc.h->class_request_handler = msc_class_request_handler;
    msc.h->standard_request_to_intf_handler = msc_standard_request_to_intf_handler;
    msc.h->data_ep_handler = msc_data_ep_handler;
    msc.h->vender_request_handler = NULL;
    
    msc.status = STAT_CBW;
}

