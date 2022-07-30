/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "stlogo.h"

DigitalOut led(LED1, 1);
DigitalOut led1(LED2, 0);
// Blinking rate in milliseconds
#define BLINKING_RATE 500ms
uint32_t pDest[64];

void lubdub()
{
    led = !led;
}

int main()
{

    printf("DMA2D Experiments - 2\n");
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

    // Enable Interrupts
    HAL_NVIC_SetPriority(DMA2D_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2D_IRQn);
    
    // Fill a rectangle
    // Set DMA2D config
    DMA2D->CR = 0x00030000UL; //Only fill a memory not copy
    DMA2D->OMAR = (uint32_t)pDest; // Starting pixel address of rect
    DMA2D->NLR = (uint32_t)(32<<16)|(uint16_t)2;//32X2 area
    DMA2D->OOR = 0; // Don't skip any pixels
    DMA2D->OPFCCR = 0; // ARGB8888
    DMA2D->OCOLR = 0x00005a5a; //some 32 bit number
    // Set the interrupt flag
    DMA2D->CR |= DMA2D_CR_TCIE;
    //Start filling
    DMA2D->CR |= DMA2D_CR_START; 

    //while(DMA2D->CR & DMA2D_CR_START);

    
    // Check ISR reg
    printf("ISR status is ..%x and CR is ...%x\n", DMA2D->ISR, DMA2D->CR);

    printf("Data in Dest is ...%x %x %x\n", pDest[0], pDest[1], pDest[2] );
    //printf("Data in Source is ...%x %x %x\n", stlogo[0], stlogo[1], stlogo[2] );
    
    while (1)
        ;
}

extern "C" void DMA2D_IRQHandler(void)
{
    led1 = 1;
    //printf("DMA2D IRQ\n");
    //HAL_DMA2D_IRQHandler();
}