/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *    ======== i2ctmp006.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include "Board.h"

#define TASKSTACKSIZE       640
Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

Void taskFxn(UArg arg0, UArg arg1)
{
    uint16_t        temperature;
    uint8_t         txBuffer[6];
    uint8_t         rxBuffer[6];
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;

    // Create I2C interface for sensor usage
    //
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;  // It can be I2C_400kHz orI2C_100kHz

    // Let's open the I2C interface
    //
    i2c = I2C_open(Board_I2C_TMP, &i2cParams);  // Board_I2C_TMP is actually I2C7
    if (i2c == NULL) {
        // error initializing IIC
        //
        System_abort("Error Initializing I2C\n");
    }

    System_printf("I2C Initialized!\n");

    // Point to the T ambient register and read its 2 bytes (actually 14 bits)
    // register number is 0x01.
    //
    txBuffer[0] = 0x01;                                 // Ambient temperature register
    i2cTransaction.slaveAddress = Board_TMP006_ADDR;    // For SENSHUB it is 0x41
    i2cTransaction.writeBuf = txBuffer;                 // transmit buffer
    i2cTransaction.writeCount = 1;                      // only one byte will be sent
    i2cTransaction.readBuf = rxBuffer;                  // receive buffer
    i2cTransaction.readCount = 2;                       // we are expecting 2 bytes

    // carry out the I2C transfer. The received 16 bits is in big endian format since IIC
    // protocol sends the most significant byte first (i.e. rxBuffer[0]) and then
    // least significant byte (i.e. rxBuffer[1]).
    //
    // Remember that temperature register is 14 bits and we need to shift right 2 bits
    // to get a number. We need to divide it by 32 to get the temperature value.
    //
    if (I2C_transfer(i2c, &i2cTransaction)) {

       // 14 bit to 16 bit conversion since least 2 bits are 0s
        //
       temperature = (rxBuffer[0] << 6) | (rxBuffer[1] >> 2);

       // This time we are going to check whether the number is negative or not.
       // If it is negative, we will sign extend to 16 bits.
       //
       if (rxBuffer[0] & 0x80) {
           temperature |= 0xF000;
       }

       // We need to divide by 32 to get the actual temperature value.
       // Check with the TMP006 datasheet
       //
       temperature /= 32;
       System_printf("Sample %d (C)\n", temperature);
    }
    else {

        // no response from TMP006. Is it there?
        //
       System_printf("I2C Bus fault\n");
    }

    // flush everything to the console
    //
    System_flush();

    // close the interface
    //
    I2C_close(i2c);
}

/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params taskParams;

    // Call board init functions
    Board_initGeneral();
    Board_initGPIO();
    Board_initI2C();

    // Construct tmp006 Task thread
    //
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)taskFxn, &taskParams, NULL);

    // Turn on user LED
    //
    GPIO_write(Board_LED0, Board_LED_ON);

    // Let TI-RTOS scheduler work
    //
    BIOS_start();

    return (0);
}
