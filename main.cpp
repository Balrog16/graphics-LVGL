/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "stlogo.h"
#include "stm32f4xx.h"
#include <chrono>
#include "lvgl/lvgl.h"

#include "BSP/STM32F429I-Discovery/stm32f429i_discovery_lcd.h"

DigitalOut led(LED1, 1);
DigitalOut led1(LED2, 0);
// Blinking rate in milliseconds
#define BLINKING_RATE 500ms
uint32_t pDest[64];
Timer timeKeeper;

void LOGERR(const std::string errMSG, const uint32_t &err, const std::string errFunc)
{
    printf("[%s] %s %d\n", errFunc.c_str(), errMSG.c_str(), err);
}
void drawBoy(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
     /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one
     *`put_px` is just an example, it needs to implemented by you.*/
    int32_t x, y;
    for(y = area->y1; y <= area->y2; y++) {
        for(x = area->x1; x <= area->x2; x++) {
            BSP_LCD_DrawPixel(x, y, color_p->full);
            color_p++;
        }
    }
    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
}

void display_init()
{
    auto ret = 0U;
    ret = BSP_LCD_Init();
    LOGERR("BSP_LCD_Init", ret, "display_init");
    // Apply the Layer configuration using LCD_LayerDefaultInit() function
    BSP_LCD_LayerDefaultInit(0, SDRAM_DEVICE_ADDR);
    // Select the LCD layer to be used using LCD_SelectLayer() function.
    BSP_LCD_SelectLayer(0);
    // Enable the LCD display using LCD_DisplayOn() function.
    BSP_LCD_SetLayerVisible(0, ENABLE);
    //
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGREEN);
    BSP_LCD_DisplayOff();
    HAL_Delay(2000);
    BSP_LCD_DisplayOn();
    BSP_LCD_SetBackColor(LCD_COLOR_LIGHTYELLOW);
    BSP_LCD_FillRect(10, 10, 40, 40);
    BSP_LCD_FillCircle(100, 100, 20);
}
void lubdub()
{
    led = !led;
}

int main()
{

    printf("DMA2D Experiments - 3\n");
    Ticker alive;
    alive.attach(&lubdub, BLINKING_RATE);

    
    timeKeeper.start();

    // Check if clock is already enabled
    uint32_t tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2DEN);
    printf("At powerON...DMA2D clock enable status...%x\n", tmpreg);

    // Enable the clock
    //__HAL_RCC_DMA2D_CLK_ENABLE();
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2DEN;

    // Check now!
    tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2DEN);
    printf("After enabling...DMA2D clock enable status...%x\n", tmpreg);

    // Enable Interrupts
    HAL_NVIC_SetPriority(DMA2D_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2D_IRQn);

    // Fill a rectangle
    // Set DMA2D config
    DMA2D->CR = 0x00030000UL;                        // Only fill a memory not copy
    DMA2D->OMAR = (uint32_t)pDest;                   // Starting pixel address of rect
    DMA2D->NLR = (uint32_t)(32 << 16) | (uint16_t)2; // 32X2 area
    DMA2D->OOR = 0;                                  // Don't skip any pixels
    DMA2D->OPFCCR = 0;                               // ARGB8888
    DMA2D->OCOLR = 0x00005a5a;                       // some 32 bit number
    // Set the interrupt flag
    DMA2D->CR |= DMA2D_CR_TCIE;
    // Start filling
    DMA2D->CR |= DMA2D_CR_START;

    // while(DMA2D->CR & DMA2D_CR_START);

    // Check ISR reg
    printf("ISR status is ..%x and CR is ...%x\n", DMA2D->ISR, DMA2D->CR);

    printf("Data in Dest is ...%x %x %x\n", pDest[0], pDest[1], pDest[2]);
    // printf("Data in Source is ...%x %x %x\n", stlogo[0], stlogo[1], stlogo[2] );
    HAL_Delay(2000);
    led1 = 0;
    display_init();

    lv_init();
    /*A static or global variable to store the buffers*/
    static lv_disp_draw_buf_t disp_buf;
    /*Static or global buffer(s). The second buffer is optional*/
    static lv_color_t buf_1[240 * 10];
    //static lv_color_t buf_2[MY_DISP_HOR_RES * 10];

    /*Initialize `disp_buf` with the buffer(s). With only one buffer use NULL instead buf_2 */
    lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, 240 * 10);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    // Initialize the structure - disp_drv
    disp_drv.draw_buf = &disp_buf;
    disp_drv.hor_res  = 240;
    disp_drv.ver_res  = 320;
    disp_drv.flush_cb = drawBoy;
    lv_disp_drv_register(&disp_drv);

    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_bg_color(&style, lv_palette_main(LV_PALETTE_GREEN));
    lv_style_set_border_color(&style, lv_palette_lighten(LV_PALETTE_GREEN, 3));
    lv_style_set_border_width(&style, 3);

    lv_obj_t * obj = lv_obj_create(lv_scr_act());
    lv_obj_add_style(obj, &style, 0);

    /*Overwrite the background color locally*/
    lv_obj_set_style_bg_color(obj, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_MAIN);

    lv_obj_center(obj);
    lv_disp_load_scr(obj);



    while (1)
    {
        // printf("\n Time elapsed since the beginning is %llu ms \n",
        // std::chrono::duration_cast<std::chrono::milliseconds>(t.elapsed_time()).count());
        lv_timer_handler();
        HAL_Delay(5);
    }
}

extern "C" void DMA2D_IRQHandler(void)
{
    led1 = 1;
    // Clear the flag
    DMA2D->IFCR |= DMA2D_ISR_TCIF;
    // printf("DMA2D IRQ\n");
    // HAL_DMA2D_IRQHandler();
}