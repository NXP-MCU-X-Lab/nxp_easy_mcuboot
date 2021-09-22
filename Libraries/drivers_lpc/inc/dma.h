/**
  ******************************************************************************
  * @file    dma.h
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.05.31
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#ifndef __CH_LIB_DMA_H__
#define __CH_LIB_DMA_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
 extern "C" {
#endif

#define MAX_DMA_CHANNEL     (DMA_CHANNEL_XFERCFG_COUNT)
     
typedef enum {
	DMA_CH0,                             /* DMA Request for channel 0 */
	DMA_CH1,                             /* DMA Request for channel 1 */
	DMA_CH2,                             /* DMA Request for channel 2 */
	DMA_CH3,                             /* DMA Request for channel 3 */
	DMA_CH4,                             /* DMA Request for channel 4 */
	DMA_CH5,                             /* DMA Request for channel 5 */
	DMA_CH6,                             /* DMA Request for channel 6 */
	DMA_CH7,                             /* DMA Request for channel 7 */
	DMA_CH8,                             /* DMA Request for channel 8 */
	DMA_CH9,                             /* DMA Request for channel 9 */
	DMA_CH10,                            /* DMA Request for channel 10 */
	DMA_CH11,                            /* DMA Request for channel 11 */
	DMA_CH12,                            /* DMA Request for channel 12 */
	DMA_CH13,                            /* DMA Request for channel 13 */
	DMA_CH14,                            /* DMA Request for channel 14 */
	DMA_CH15,                            /* DMA Request for channel 15 */
	DMA_CH16,                            /* DMA Request for channel 16 */
	DMA_CH17,                            /* DMA Request for channel 17 */
	DMA_CH18,                            /* DMA Request for channel 18 */
	DMA_CH19,                            /* DMA Request for channel 19 */

	/* Alias DMAREQ defines */
	DMAREQ_FLEXCOMM0_RX = DMA_CH0,       /* DMA Request for flexcomm 0 RX */
	DMAREQ_FLEXCOMM0_TX,                 /* DMA Request for flexcomm 0 TX */
	DMAREQ_FLEXCOMM1_RX,                 /* DMA Request for flexcomm 1 RX */
	DMAREQ_FLEXCOMM1_TX,                 /* DMA Request for flexcomm 1 TX */
	DMAREQ_FLEXCOMM2_RX,                 /* DMA Request for flexcomm 2 RX */
	DMAREQ_FLEXCOMM2_TX,                 /* DMA Request for flexcomm 2 TX */
	DMAREQ_FLEXCOMM3_RX,                 /* DMA Request for flexcomm 3 RX */
	DMAREQ_FLEXCOMM3_TX,                 /* DMA Request for flexcomm 3 TX */
	DMAREQ_FLEXCOMM4_RX,                 /* DMA Request for flexcomm 4 RX */
	DMAREQ_FLEXCOMM4_TX,                 /* DMA Request for flexcomm 4 TX */
	DMAREQ_FLEXCOMM5_RX,                 /* DMA Request for flexcomm 5 RX */
	DMAREQ_FLEXCOMM5_TX,                 /* DMA Request for flexcomm 5 TX */
	DMAREQ_FLEXCOMM6_RX,                 /* DMA Request for flexcomm 6 RX */
	DMAREQ_FLEXCOMM6_TX,                 /* DMA Request for flexcomm 6 TX */
	DMAREQ_FLEXCOMM7_RX,                 /* DMA Request for flexcomm 7 RX */
	DMAREQ_FLEXCOMM7_TX,                 /* DMA Request for flexcomm 7 TX */
    
	DMAREQ_DMIC0,                        /* DMA Request for Digital MIC-0 */
	DMAREQ_DMIC1,                        /* DMA Request for Digital MIC-1 */
	DMAREQ_SPIFI,                        /* DMA Request for SPIFI controller */
	DMAREQ_SHA,
	DMAREQ_FLEXCOMM8_RX,
	DMAREQ_FLEXCOMM8_TX,
	DMAREQ_FLEXCOMM9_RX,
	DMAREQ_FLEXCOMM9_TX,
}DMA_Chl_t;

/* DMA channel source/address/next descriptor */
typedef struct
{
	uint32_t  xfercfg;      /*!< Transfer configuration (only used in linked lists and ping-pong configs) */
	uint32_t  src;          /*!< DMA transfer source end address */
	uint32_t  dest;			/*!< DMA transfer desintation end address */
	uint32_t  next;			/*!< Link to next DMA descriptor, must be 16 byte aligned */
} DMA_Desc_t;

typedef struct 
{
    uint8_t                     chl;
    bool                        isPeriph;
    uint16_t                    transferCnt;        /* numbers of one transfer */
    uint16_t                    dataWidth;          /* 1,2 or 4 unit in byte */

    uint16_t                    sAddrInc;           /*  0:no increasment, 1: 1 x width, 2: 2 x width, 3: 4 x width */
    uint32_t                    sAddr;

    uint16_t                    dAddrInc;                
    uint32_t                    dAddr;
}DMA_ChlSetup_t;

     
void DMA_Init(void);
void DMA_SetChlIntMode(uint32_t chl, bool val);
void DMA_SWTrigger(uint32_t chl);
void DMA_SetupChl(DMA_ChlSetup_t *setup);
uint32_t DMA_GetTransferCnt(uint8_t chl);
void DMA_SetChlLink(uint8_t chl, DMA_Desc_t *list);
void DMA_CopyDesc(uint8_t ch, DMA_Desc_t *desc);
DMA_Desc_t *DMA_GetDesc(uint8_t ch);
#endif


