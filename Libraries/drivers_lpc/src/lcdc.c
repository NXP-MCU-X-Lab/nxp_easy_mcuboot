/**
  ******************************************************************************
  * @file    lcdc.c
  * @author  YANDLD
  * @version V3.0.0
  * @date    2015.6.21
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
  ******************************************************************************
  */
  
#include "common.h"
#include "lcdc.h"

#if defined(LCD)

/*!@brief How many hardware cursors supports. */
#define LCDC_CURSOR_COUNT 4

/*!@brief LCD cursor image bits per pixel. */
#define LCDC_CURSOR_IMG_BPP 2

/*!@brief LCD 32x32 cursor image size in word(32-bit). */
#define LCDC_CURSOR_IMG_32X32_WORDS (32 * 32 * LCDC_CURSOR_IMG_BPP / (8 * sizeof(uint32_t)))

/*!@brief LCD 64x64 cursor image size in word(32-bit). */
#define LCDC_CURSOR_IMG_64X64_WORDS (64 * 64 * LCDC_CURSOR_IMG_BPP / (8 * sizeof(uint32_t)))

/*!@brief LCD palette size in words(32-bit). */
#define LCDC_PALETTE_SIZE_WORDS (ARRAY_SIZE(((LCD_Type *)0)->PAL))

static ALIGN(4) const uint8_t CursorDefault32Img0[] = {
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 1.  */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 2.  */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 3.  */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 4.  */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 5.  */
    0xAA, 0xAA, 0xAA, 0xFA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 6.  */
    0xAA, 0xAA, 0xAB, 0xFE, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 7.  */
    0xAA, 0xAA, 0xAB, 0xFE, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 8.  */
    0xAA, 0xAA, 0xAB, 0xFE, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 9.  */
    0xAA, 0xAA, 0xAB, 0xFE, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 10  */
    0xAA, 0xAA, 0xAB, 0xFF, 0xEA,
    0xAA, 0xAA, 0xAA, /* Line 11. */
    0xAA, 0xAA, 0xAB, 0xFF, 0xFF,
    0xAA, 0xAA, 0xAA, /* Line 12. */
    0xAA, 0xAA, 0xAB, 0xFF, 0xFF,
    0xFA, 0xAA, 0xAA, /* Line 13. */
    0xAA, 0xAA, 0xAB, 0xFF, 0xFF,
    0xFE, 0xAA, 0xAA, /* Line 14. */
    0xAA, 0xAB, 0xFB, 0xFF, 0xFF,
    0xFF, 0xAA, 0xAA, /* Line 15. */
    0xAA, 0xAB, 0xFF, 0xFF, 0xFF,
    0xFF, 0xAA, 0xAA, /* Line 16. */
    0xAA, 0xAB, 0xFF, 0xFF, 0xFF,
    0xFF, 0xAA, 0xAA, /* Line 17. */
    0xAA, 0xAA, 0xFF, 0xFF, 0xFF,
    0xFF, 0xAA, 0xAA, /* Line 18. */
    0xAA, 0xAA, 0xBF, 0xFF, 0xFF,
    0xFF, 0xAA, 0xAA, /* Line 19. */
    0xAA, 0xAA, 0xBF, 0xFF, 0xFF,
    0xFF, 0xAA, 0xAA, /* Line 20. */
    0xAA, 0xAA, 0xAF, 0xFF, 0xFF,
    0xFF, 0xAA, 0xAA, /* Line 21. */
    0xAA, 0xAA, 0xAF, 0xFF, 0xFF,
    0xFE, 0xAA, 0xAA, /* Line 22. */
    0xAA, 0xAA, 0xAB, 0xFF, 0xFF,
    0xFE, 0xAA, 0xAA, /* Line 23. */
    0xAA, 0xAA, 0xAB, 0xFF, 0xFF,
    0xFE, 0xAA, 0xAA, /* Line 24. */
    0xAA, 0xAA, 0xAA, 0xFF, 0xFF,
    0xFA, 0xAA, 0xAA, /* Line 25. */
    0xAA, 0xAA, 0xAA, 0xFF, 0xFF,
    0xFA, 0xAA, 0xAA, /* Line 26. */
    0xAA, 0xAA, 0xAA, 0xFF, 0xFF,
    0xFA, 0xAA, 0xAA, /* Line 27. */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 28. */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 29. */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 30. */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, /* Line 31. */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA /* Line 32. */
};


void LCDC_SetIntMode(bool val)
{
    LCD->INTMSK = LCD_INTMSK_LNBUIM_MASK;
    NVIC_EnableIRQ(LCD_IRQn);
}

void LCDC_GetDefaultConfig(LCDC_Config_t *config)
{
    config->ppl = 0U;
    config->hsw = 0U;
    config->hfp = 0U;
    config->hbp = 0U;
    config->lpp = 0U;
    config->vsw = 0U;
    config->vfp = 0U;
    config->vbp = 0U;
    config->acBiasFreq = 1U;
    config->polarityFlags = 0U;
    config->enableLineEnd = false;
    config->lineEndDelay = 0U;
    config->upperPanelAddr = 0U;
    config->lowerPanelAddr = 0U;
    config->bpp = kLCDC_16BPP565;
    config->dataFormat = kLCDC_LittleEndian;
    config->swapRedBlue = true;
    config->fps = 50;
}

void LCDC_Init(LCDC_Config_t *config)
{
    int i;
    uint32_t reg, pixel_clk;
    
    SYSCON->AHBCLKCTRL[2] |= SYSCON_AHBCLKCTRL_LCD_MASK;
    
    /*
    0x0 Main clock (main_clk).
    0x1 LCD external clock input (LCD_CLKIN).
    0x2 FRO 96 or 48 MHz (fro_hf).
    0x3 None, this may be selected to reduce power when no output is needed.
    */
    /* main clock */
    SYSCON->LCDCLKSEL = SYSCON_LCDCLKSEL_SEL(0);
    
    pixel_clk = config->fps * (config->hbp + config->hfp + config->ppl) * (config->vbp + config->vfp + config->lpp);
    LIB_TRACE("width:%d heigt:%d\r\n", config->ppl, config->lpp);
    LIB_TRACE("pixel clk need:%dKHz\r\n", pixel_clk/1000);
    LIB_TRACE("lcd: %dx%d\r\n", config->ppl, config->lpp);
    
    /* load a defualt value */
    SYSCON->LCDCLKDIV = SYSCON_LCDCLKDIV_DIV_MASK;
    for(i=0; i<256; i++)
    {
        if((ABS((int)GetClock(kCoreClock) / i) - (int)pixel_clk) < (pixel_clk / 10)) /* if reach 10% of accuricy */
        {
            LIB_TRACE("lcd clock divider found:%d\r\n", i);
            SYSCON->LCDCLKDIV = SYSCON_LCDCLKDIV_DIV(i - 1);
            break;
        }
    }
    
    /* Set register CTRL. */
    reg = (LCD_CTRL_LCDVCOMP(3) | LCD_CTRL_WATERMARK_MASK);
    reg |= (uint32_t)(config->dataFormat) | LCD_CTRL_LCDTFT_MASK | LCD_CTRL_LCDBPP(config->bpp);

    if (config->swapRedBlue)
    {
        reg |= LCD_CTRL_BGR_MASK;
    }
    
    /* Configure timing. */
    LCD->TIMH = LCD_TIMH_PPL((config->ppl / 16U) - 1U) | LCD_TIMH_HSW(config->hsw - 1U) | LCD_TIMH_HFP(config->hfp - 1U) | LCD_TIMH_HBP(config->hbp - 1U);
    LCD->TIMV = LCD_TIMV_LPP(config->lpp - 1U) | LCD_TIMV_VSW(config->vsw - 1U) | LCD_TIMV_VFP(config->vfp - 1U) | LCD_TIMV_VBP(config->vbp - 1U);
    LCD->POL = (uint32_t)(config->polarityFlags) | LCD_POL_ACB(config->acBiasFreq - 1U) | LCD_POL_BCD_MASK | LCD_POL_CPL(config->ppl - 1U);

    /* Line end configuration. */
    if (config->enableLineEnd)
    {
        LCD->LE = LCD_LE_LED(config->lineEndDelay - 1U) | LCD_LE_LEE_MASK;
    }
    else
    {
        LCD->LE = 0U;
    }

    /* Set panel frame base address. */
    LCD->UPBASE = config->upperPanelAddr;
    LCD->LPBASE = config->lowerPanelAddr;
    
    LCD->CTRL = reg | LCD_CTRL_LCDPWR_MASK | LCD_CTRL_LCDEN_MASK;
}

void LCDC_CursorSetImage(LCD_Type *base, uint8_t index, const uint32_t *image)
{
    uint32_t regStart;
    uint32_t i;
    uint32_t len;

    regStart = index * LCDC_CURSOR_IMG_32X32_WORDS;
    len = LCDC_CURSOR_IMG_32X32_WORDS;

    for (i = 0U; i < len; i++)
    {
        base->CRSR_IMG[regStart + i] = image[i];
    }
}

void LCDC_CursorInit(void)
{

    /* size = 32x32, */
    LCD->CRSR_CFG = LCD_CRSR_CFG_CRSRSIZE(0) | LCD_CRSR_CFG_FRAMESYNC_MASK;

    /* Set position. */
    LCDC_CursorSetPos(0, 0);

    /* Palette. */
    //LCD->CRSR_PAL0 = ((uint32_t)config->palette0.red << LCD_CRSR_PAL0_RED_SHIFT) | ((uint32_t)config->palette0.blue << LCD_CRSR_PAL0_BLUE_SHIFT) | ((uint32_t)config->palette0.green << LCD_CRSR_PAL0_GREEN_SHIFT);   
    //LCD->CRSR_PAL1 = ((uint32_t)config->palette1.red << LCD_CRSR_PAL1_RED_SHIFT) | ((uint32_t)config->palette1.blue << LCD_CRSR_PAL1_BLUE_SHIFT) | ((uint32_t)config->palette1.green << LCD_CRSR_PAL1_GREEN_SHIFT);s

    LCDC_CursorSetImage(LCD, 0, (const uint32_t *)CursorDefault32Img0);
    
    /* select image 0 */
    LCD->CRSR_CTRL = LCD_CRSR_CTRL_CRSRNUM1_0(0);
    LCDC_CursorShow(true);
}

void LCDC_CursorSetPos(int32_t x, int32_t y)
{
    //LCD->CRSR_CLIP = LCD_CRSR_CLIP_CRSRCLIPX(32/2) | LCD_CRSR_CLIP_CRSRCLIPY(32/2);
    LCD->CRSR_XY = LCD_CRSR_XY_CRSRX(x) | LCD_CRSR_XY_CRSRY(y);
}

void LCDC_CursorSelectImage(uint8_t index)
{
    LCD->CRSR_CTRL = (LCD->CRSR_CTRL & ~LCD_CRSR_CTRL_CRSRNUM1_0_MASK) | LCD_CRSR_CTRL_CRSRNUM1_0(index);
}

void LCDC_CursorShow(bool val)
{
    (val)?(LCD->CRSR_CTRL |= LCD_CRSR_CTRL_CRSRON_MASK):(LCD->CRSR_CTRL &= ~LCD_CRSR_CTRL_CRSRON_MASK);
}

#endif

