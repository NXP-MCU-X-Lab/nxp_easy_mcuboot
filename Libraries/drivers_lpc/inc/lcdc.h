/**
  ******************************************************************************
  * @file    lcdc.h
  * @author  YANDLD
  * @version V3.0.0
  * @date    2016.6.2
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
#ifndef __CH_LIB_LPC_LCDC_H__
#define __CH_LIB_LPC_LCDC_H__

#ifdef __cplusplus
 extern "C" {
#endif


#include <stdint.h>
#include <stdbool.h>

/*!
 * @brief LCD bits per pixel.
 */
typedef enum
{
    kLCDC_1BPP = 0U,     /*!< 1 bpp. */
    kLCDC_2BPP = 1U,     /*!< 2 bpp. */
    kLCDC_4BPP = 2U,     /*!< 4 bpp. */
    kLCDC_8BPP = 3U,     /*!< 8 bpp. */
    kLCDC_16BPP = 4U,    /*!< 16 bpp. */
    kLCDC_24BPP = 5U,    /*!< 24 bpp, TFT panel only. */
    kLCDC_16BPP565 = 6U, /*!< 16 bpp, 5:6:5 mode. */
    kLCDC_12BPP = 7U,    /*!< 12 bpp, 4:4:4 mode. */
} lcdc_bpp_t;


/*!
 * @brief LCD panel buffer data format.
 */
typedef enum
{
    kLCDC_LittleEndian = 0U,                                   /*!< Little endian byte, little endian pixel. */
    kLCDC_BigEndian = LCD_CTRL_BEPO_MASK | LCD_CTRL_BEBO_MASK, /*!< Big endian byte, big endian pixel. */
    kLCDC_WinCeMode = LCD_CTRL_BEPO_MASK, /*!< little-endian byte, big-endian pixel for Windows CE mode. */
} lcdc_data_format_t;

/*!
 * @brief LCD sigal polarity flags.
 */
enum
{
    kLCDC_InvertVsyncPolarity = LCD_POL_IVS_MASK, /*!< Invert the VSYNC polarity, set to active low. */
    kLCDC_InvertHsyncPolarity = LCD_POL_IHS_MASK, /*!< Invert the HSYNC polarity, set to active low. */
    kLCDC_InvertClkPolarity = LCD_POL_IPC_MASK,   /*!< Invert the panel clock polarity, set to
                                                      drive data on falling edge. */
    kLCDC_InvertDePolarity = LCD_POL_IOE_MASK,    /*!< Invert the data enable (DE) polarity, set to active low. */
};

typedef struct
{
    uint16_t ppl;            /*!< Pixels per line, it must could be divided by 16. */
    uint8_t hsw;             /*!< HSYNC pulse width. */
    uint8_t hfp;             /*!< Horizontal front porch. */
    uint8_t hbp;             /*!< Horizontal back porch. */
    uint16_t lpp;            /*!< Lines per panal. */
    uint8_t vsw;             /*!< VSYNC pulse width. */
    uint8_t vfp;             /*!< Vrtical front porch. */
    uint8_t vbp;             /*!< Vertical back porch. */
    uint8_t acBiasFreq;      /*!< The number of line clocks between AC bias pin toggling. Only used for STN display. */
    uint16_t polarityFlags;  /*!< OR'ed value of @ref _lcdc_polarity_flags, used to contol the signal polarity. */
    bool enableLineEnd;      /*!< Enable line end or not, the line end is a positive pulse with 4 panel clock. */
    uint8_t lineEndDelay;    /*!< The panel clocks between the last pixel of line and the start of line end. */
    uint32_t upperPanelAddr; /*!< LCD upper panel base address, must be double-word(64-bit) align. */
    uint32_t lowerPanelAddr; /*!< LCD lower panel base address, must be double-word(64-bit) align. */
    lcdc_bpp_t bpp;           /*!< LCD bits per pixel. */
    lcdc_data_format_t dataFormat; /*!< Data format. */
    bool swapRedBlue;             /*!< Set true to use BGR format, set false to choose RGB format. */
    uint8_t fps;
}LCDC_Config_t;

void LCDC_GetDefaultConfig(LCDC_Config_t *config);
void LCDC_Init(LCDC_Config_t *config);
void LCDC_SetIntMode(bool val);
void LCDC_CursorInit(void);
void LCDC_CursorSetPos(int32_t x, int32_t y);
void LCDC_CursorShow(bool val);
void LCDC_CursorSelectImage(uint8_t index);


#endif

