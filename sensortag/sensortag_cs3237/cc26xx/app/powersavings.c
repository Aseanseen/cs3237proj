/*
    2  * Copyright (c) 2018, Texas Instruments Incorporated
    3  * All rights reserved.
    4  *
    5  * Redistribution and use in source and binary forms, with or without
    6  * modification, are permitted provided that the following conditions
    7  * are met:
    8  *
    9  * *  Redistributions of source code must retain the above copyright
   10  *    notice, this list of conditions and the following disclaimer.
   11  *
   12  * *  Redistributions in binary form must reproduce the above copyright
   13  *    notice, this list of conditions and the following disclaimer in the
   14  *    documentation and/or other materials provided with the distribution.
   15  *
   16  * *  Neither the name of Texas Instruments Incorporated nor the names of
   17  *    its contributors may be used to endorse or promote products derived
   18  *    from this software without specific prior written permission.
   19  *
   20  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   21  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
   22  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
   23  * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
   24  * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   25  * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   26  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
   27  * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
   28  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
   29  * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
   30  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
   31  */
    
  
    
    #include <stdbool.h>
    #include <stddef.h>
    #include <stdint.h>
    
    //#include <ti/devices/CC26XX.h>
    #include <driverlib/ioc.h>
    #include <driverlib/cpu.h>
    
    #include <ti/drivers/pin/PINCC26XX.h>
    
    #include "Board.h"
    
 
    void CC2650_LAUNCHXL_sendExtFlashByte(PIN_Handle pinHandle, uint8_t byte)
  {
        uint8_t i;
    
        /* SPI Flash CS */
        PIN_setOutputValue(pinHandle, IOID_20, 0);
    
        for (i = 0; i < 8; i++) {
            PIN_setOutputValue(pinHandle, IOID_10, 0);  /* SPI Flash CLK */
    
            /* SPI Flash MOSI */
            PIN_setOutputValue(pinHandle, IOID_9, (byte >> (7 - i)) & 0x01);
            PIN_setOutputValue(pinHandle, IOID_10, 1);  /* SPI Flash CLK */
    
            /*
             * Waste a few cycles to keep the CLK high for at
             * least 45% of the period.
             * 3 cycles per loop: 8 loops @ 48 Mhz = 0.5 us.
             */
            CPUdelay(8);
        }
    
        PIN_setOutputValue(pinHandle, IOID_10, 0);  /* CLK */
        PIN_setOutputValue(pinHandle, IOID_20, 1);  /* CS */
    
        /*
         * Keep CS high at least 40 us
         * 3 cycles per loop: 700 loops @ 48 Mhz ~= 44 us
         */
        CPUdelay(700);
    }
    
    /*
     *  ======== CC2650_LAUNCHXL_wakeUpExtFlash ========
     */
    void CC2650_LAUNCHXL_wakeUpExtFlash(void)
    {
        PIN_Config extFlashPinTable[] = {
            /* SPI Flash CS */
            IOID_20 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL |
                    PIN_INPUT_DIS | PIN_DRVSTR_MED,
            PIN_TERMINATE
        };
        PIN_State extFlashPinState;
        PIN_Handle extFlashPinHandle = PIN_open(&extFlashPinState, extFlashPinTable);
    
        /*
     /* Toggle chip select for ~20ns to wake ext. flash */
       PIN_setOutputValue(extFlashPinHandle, IOID_20, 0);
       /* 3 cycles per loop: 1 loop @ 48 Mhz ~= 62 ns */
       CPUdelay(1);
       PIN_setOutputValue(extFlashPinHandle, IOID_20, 1);
       /* 3 cycles per loop: 560 loops @ 48 Mhz ~= 35 us */
       CPUdelay(560);
   
       PIN_close(extFlashPinHandle);
   }
   
   /*
    *  ======== CC2650_LAUNCHXL_shutDownExtFlash ========
    */
   void CC2650_LAUNCHXL_shutDownExtFlash(void)
   {
       /*
       *  To be sure we are putting the flash into sleep and not waking it,
        *  we first have to make a wake up call
        */
       CC2650_LAUNCHXL_wakeUpExtFlash();
   
       PIN_Config extFlashPinTable[] = {
           /* SPI Flash CS*/
           IOID_20 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL |
                   PIN_INPUT_DIS | PIN_DRVSTR_MED,
           /* SPI Flash CLK */
           IOID_10 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL |
                   PIN_INPUT_DIS | PIN_DRVSTR_MED,
           /* SPI Flash MOSI */
          IOID_9 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL |
                   PIN_INPUT_DIS | PIN_DRVSTR_MED,
           /* SPI Flash MISO */
           IOID_8 | PIN_INPUT_EN | PIN_PULLDOWN,
          PIN_TERMINATE
       };
       PIN_State extFlashPinState;
       PIN_Handle extFlashPinHandle = PIN_open(&extFlashPinState, extFlashPinTable);
   
       uint8_t extFlashShutdown = 0xB9;
   
       CC2650_LAUNCHXL_sendExtFlashByte(extFlashPinHandle, extFlashShutdown);
   
       PIN_close(extFlashPinHandle);
   }
   
  /*
    *  ======== Board_initHook ========
    *  Called by Board_init() to perform board-specific initialization.
    */
   void Board_initHook()
   {
       CC2650_LAUNCHXL_shutDownExtFlash();
   }   