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
#include "BSP/STM32F429I-Discovery/stm32f429i_discovery_ts.h"

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
void drawBoy(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one
     *`put_px` is just an example, it needs to implemented by you.*/
    printf("\n To print  from %d and %d  To %d and %d\n", area->x1, area->y1, area->x2, area->y2);
    int32_t x, y;
    for (y = area->y1; y <= area->y2; y++)
    {
        for (x = area->x1; x <= area->x2; x++)
        {
            BSP_LCD_DrawPixel(x, y, color_p->full);
            // printf("\n Pixel Val is %x\n", color_p->full);
            color_p++;
        }
    }
    led1 = !led1;
    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
}

static void event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED)
    {
        LV_LOG_USER("Clicked");
    }
    else if (code == LV_EVENT_VALUE_CHANGED)
    {
        LV_LOG_USER("Toggled");
    }
}

void lv_example_btn_1(void)
{
    lv_obj_t *label;

    lv_obj_t *btn1 = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -40);

    label = lv_label_create(btn1);
    lv_label_set_text(label, "Button");
    lv_obj_center(label);

    lv_obj_t *btn2 = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn2, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 40);
    lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_set_height(btn2, LV_SIZE_CONTENT);

    label = lv_label_create(btn2);
    lv_label_set_text(label, "Toggle");
    lv_obj_center(label);
}

void my_input_read(lv_indev_drv_t * drv, lv_indev_data_t*data)
{
    TS_StateTypeDef tsState;
    BSP_TS_GetState(&tsState);
    printf("\n States is %d %d %d %d\n", tsState.TouchDetected, tsState.X, tsState.Y, tsState.Z );
  if(tsState.TouchDetected==1) {
    data->point.x = tsState.X;
    data->point.y = tsState.Y;
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

static lv_obj_t * label;

static void slider_event_cb(lv_event_t * e)
{
    lv_obj_t * slider = lv_event_get_target(e);

    /*Refresh the text*/
    lv_label_set_text_fmt(label, "%"LV_PRId32, lv_slider_get_value(slider));
    lv_obj_align_to(label, slider, LV_ALIGN_OUT_TOP_MID, 0, -15);    /*Align top of the slider*/
}

/**
 * Create a slider and write its value on a label.
 */
void lv_example_get_started_3(void)
{
    /*Create a slider in the center of the display*/
    lv_obj_t * slider = lv_slider_create(lv_scr_act());
    lv_obj_set_width(slider, 200);                          /*Set the width*/
    lv_obj_center(slider);                                  /*Align to the center of the parent (screen)*/
    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);     /*Assign an event function*/

    /*Create a label above the slider*/
    label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "0");
    lv_obj_align_to(label, slider, LV_ALIGN_OUT_TOP_MID, 0, -15);    /*Align top of the slider*/
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
    // BSP_LCD_DisplayOff();
    // HAL_Delay(2000);
    BSP_LCD_DisplayOn();
    BSP_LCD_SetBackColor(LCD_COLOR_LIGHTYELLOW);
    BSP_LCD_FillRect(0, 0, 240, 320);
    // BSP_LCD_FillCircle(100, 100, 20);
    HAL_Delay(5000);
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

    /*
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
        led1 = 0;*/
    display_init();
    BSP_TS_Init(240, 320);
    lv_init();

    // /*A static or global variable to store the buffers*/
    static lv_disp_draw_buf_t disp_buf;
    // /*Static or global buffer(s). The second buffer is optional*/
    static lv_color_t buf_1[240 * 10];
    // //static lv_color_t buf_2[MY_DISP_HOR_RES * 10];

    // /*Initialize `disp_buf` with the buffer(s). With only one buffer use NULL instead buf_2 */
    lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, 240 * 10);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    // Initialize the structure - disp_drv
    disp_drv.draw_buf = &disp_buf;
    disp_drv.hor_res = 240;
    disp_drv.ver_res = 320;
    disp_drv.flush_cb = drawBoy;
    lv_disp_drv_register(&disp_drv);

    //  static lv_style_t style;
    //  lv_style_init(&style);
    //  lv_style_set_bg_color(&style, lv_palette_main(LV_PALETTE_GREEN));
    //  lv_style_set_border_color(&style, lv_palette_lighten(LV_PALETTE_GREEN, 3));
    //  lv_style_set_border_width(&style, 3);

    // lv_obj_t * obj = lv_obj_create(lv_scr_act());
    // lv_obj_add_style(obj, &style, 0);

    // /*Overwrite the background color locally*/
    // lv_obj_set_style_bg_color(obj, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_MAIN);

    // lv_obj_center(obj);
    // lv_disp_load_scr(obj);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);           /*Basic initialization*/
    indev_drv.type = LV_INDEV_TYPE_POINTER;                     /*See below.*/
    indev_drv.read_cb = my_input_read; /*See below.*/
        /*Register the driver in LVGL and save the created input device object*/
        lv_indev_t *my_indev = lv_indev_drv_register(&indev_drv);

    //lv_example_btn_1();
    lv_example_get_started_3();

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