/*
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    MK02FN128VFM10_ex00_basic_MK02FN128VFM10.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK02F12810.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "fsl_dac.h"
#include "fsl_uart.h"
#include "MK02FN128VFM10_uart.h"
#include "xprintf.h"

/* TODO: insert other definitions and declarations here. */
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_DAC_USED_BUFFER_SIZE 	DAC_DATL_COUNT

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION
volatile bool g_DacBufferWatermarkInterruptFlag;
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION */
volatile bool g_DacBufferReadPointerTopPositionInterruptFlag;
volatile bool g_DacBufferReadPointerBottomPositionInterruptFlag;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief IRQ function for DAC buffer interrupt
 */
/* DAC0_IRQn interrupt handler */
void DAC0_IRQHANDLER(void) {
  /*  Place your code here */
    uint32_t flags = DAC_GetBufferStatusFlags(DAC0);

#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION
    if (kDAC_BufferWatermarkFlag == (kDAC_BufferWatermarkFlag & flags)) {
        g_DacBufferWatermarkInterruptFlag = true;
    }
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION */
    if (kDAC_BufferReadPointerTopPositionFlag == (kDAC_BufferReadPointerTopPositionFlag & flags)) {
        g_DacBufferReadPointerTopPositionInterruptFlag = true;
    }
    if (kDAC_BufferReadPointerBottomPositionFlag == (kDAC_BufferReadPointerBottomPositionFlag & flags)) {
        g_DacBufferReadPointerBottomPositionInterruptFlag = true;
    }
    DAC_ClearBufferStatusFlags(DAC0, flags); /* Clear flags. */

    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}

/*
 * @brief   Application entry point.
 */
int main(void) {
    dac_buffer_config_t dacBufferConfigStruct;
    uint8_t index;
    uint16_t dacValue;

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    xdev_out(UART0_PutByte);
    xdev_in(UART0_GetByte);
    xprintf("\r\nMK02FN128VFM10_ex14_dac_buffer_interrupt\r\n");

    /* Configure the DAC. */
    /*
     * dacConfigStruct.referenceVoltageSource = kDAC_ReferenceVoltageSourceVref2;
     * dacConfigStruct.enableLowPowerMode = false;
     */
    /* Configure the DAC buffer. */
    DAC_GetDefaultBufferConfig(&dacBufferConfigStruct);
    DAC_SetBufferConfig(DAC0, &dacBufferConfigStruct);
    DAC_SetBufferReadPointer(DAC0_PERIPHERAL, 0U); /* Make sure the read pointer to the start. */
    dacValue = 0U;
    for (index = 0U; index < DEMO_DAC_USED_BUFFER_SIZE; index++) {
        DAC_SetBufferValue(DAC0_PERIPHERAL, index, dacValue);
        dacValue += (0xFFFU / DEMO_DAC_USED_BUFFER_SIZE);
    }

/* Clear flags. */
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION
    g_DacBufferWatermarkInterruptFlag = false;
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION */
    g_DacBufferReadPointerTopPositionInterruptFlag    = false;
    g_DacBufferReadPointerBottomPositionInterruptFlag = false;

    xprintf("\r\nDAC Buffer Information\r\n");
    xprintf("\t  Buffer index max  : %d\r\n", dacBufferConfigStruct.upperLimit);
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION
    xprintf("\t  Buffer watermark  : %d\r\n", dacBufferConfigStruct.upperLimit - (uint8_t)(dacBufferConfigStruct.watermark) - 1U);
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION */
    xprintf("Press any key in terminal to trigger the buffer ...\r\n");
    index = 0U;

    /* Force the counter to be placed into memory. */
    volatile static int cnt = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        if (0U == index)  {
            xprintf("\r\n");
         }
        xprintf("Buffer Index %2d: ", index);
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION
        if (g_DacBufferWatermarkInterruptFlag) {
            xprintf("WatermarkEvent \t");
            g_DacBufferWatermarkInterruptFlag = false;
         }
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION */
        if (g_DacBufferReadPointerTopPositionInterruptFlag) {
            xprintf("ReadPointerTopPositionEvent \t");
            g_DacBufferReadPointerTopPositionInterruptFlag = false;
         }
        if (g_DacBufferReadPointerBottomPositionInterruptFlag) {
            xprintf("ReadPointerBottomPositionEvent \t");
            g_DacBufferReadPointerBottomPositionInterruptFlag = false;
         }
        xprintf("\r\n");

        /* Trigger the buffer and move the pointer. */
        UART0_GetByte();
        DAC_DoSoftwareTriggerBuffer(DAC0_PERIPHERAL);
        index++;
        if (index >= DEMO_DAC_USED_BUFFER_SIZE) {
            index = 0U;
         }
        cnt++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}
