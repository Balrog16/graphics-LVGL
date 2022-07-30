/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "stlogo.h"

DigitalOut led(LED1, 1);
// Blinking rate in milliseconds
#define BLINKING_RATE 500ms
unsigned char pDest[9174];

void lubdub()
{
    led = !led;
}

int main()
{

    printf("DMA2D Experiments - 1\n");
    Ticker alive;
    alive.attach(&lubdub, BLINKING_RATE);

    // Check if clock is already enabled
    uint32_t tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2DEN);
    printf("At powerON...DMA2D clock enable status...%x\n", tmpreg);

    // Enable the clock
    //__HAL_RCC_DMA2D_CLK_ENABLE();
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2DEN;

    // Check now!
    tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2DEN);
    printf("After enabling...DMA2D clock enable status...%x\n", tmpreg);

    // Set DMA2D config
    DMA2D->FGPFCCR = DMA2D_RGB565;
    DMA2D->FGCOLR = 0;
    DMA2D->FGMAR = (uint32_t)stlogo;
    DMA2D->FGOR = 0;
    DMA2D->CR = DMA2D_M2M;
    DMA2D->OPFCCR = DMA2D_RGB565;
    DMA2D->OCOLR = 0;
    DMA2D->OMAR = (uint32_t)pDest;
    DMA2D->OOR = 0;
    DMA2D->NLR = (uint32_t)((80 < 16) | 1);
    

    printf("Pixel Count ..%x\n", (80<<16|1));
    // Now enable a transfer
    DMA2D->CR = DMA2D->CR | DMA2D_CR_START;
    while(DMA2D->CR & DMA2D_CR_START);

    
    // Check ISR reg
    printf("ISR status is ..%x and CR is ...%x\n", DMA2D->ISR, DMA2D->CR);

    printf("Data in Dest is ...%x %x %x\n", pDest[0], pDest[1], pDest[2] );
    printf("Data in Source is ...%x %x %x\n", stlogo[0], stlogo[1], stlogo[2] );
    while (1)
        ;
}

extern "C" void DMA2D_IRQHandler(void)
{
    printf("DMA2D IRQ\n");
    // HAL_DMA2D_IRQHandler();
}