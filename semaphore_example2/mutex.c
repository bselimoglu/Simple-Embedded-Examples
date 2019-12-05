/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
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
 *
 * Added synchronization of two tasks by Hakan Guray Senel 26.11.2019
 * Added v2 of synchronization example 27.11.2019
 */

/* XDC module Headers */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

/* Example/Board Header files */
#include "Board.h"

/* task prototypes */
Void task1(UArg arg0, UArg arg1);
Void task2(UArg arg0, UArg arg1);

/* two semaphores handles */
Semaphore_Handle aArrived;
Semaphore_Handle bArrived;

/* two task handles */
Task_Handle tsk1;
Task_Handle tsk2;

/* stacks for tasks 1 and 2 */
#define STACKSIZE   512
Char task1Stack[STACKSIZE], task2Stack[STACKSIZE];

/* we will count up to 8 and
   finish to see logs */
int finishCount = 0;

/*
 *  ======== main ========
 */
Int main()
{
    Task_Params tskprm;
    Board_initGeneral();
    aArrived=Semaphore_create(0,NULL,NULL);
    bArrived=Semaphore_create(0,NULL,NULL);

    /* create first task */
    Task_Params_init(&tskprm);
    tskprm.priority = 1;
    tskprm.stackSize = STACKSIZE;
    tskprm.stack = &task1Stack;
    tsk1 = Task_create(task1, &tskprm, NULL);

    /* create the second task */
    Task_Params_init(&tskprm);
    tskprm.priority = 1;
    tskprm.stackSize = STACKSIZE;
    tskprm.stack = &task2Stack;
    tsk2 = Task_create(task2, &tskprm, NULL);

    BIOS_start();    /* does not return */
    return(0);
}

Void task1(UArg arg0, UArg arg1)
{
    for (;;) {

        /* statement a1 */

        System_printf("Running task1 function\n");
        Semaphore_post(aArrived);
        Semaphore_pend(bArrived, BIOS_WAIT_FOREVER);

        /* statement a2 */

        Task_sleep(2);
    }
}

Void task2(UArg arg0, UArg arg1)
{
    for (;;) {

        /* statement b1 */

        System_printf("Running task2 function\n");
        Semaphore_post(bArrived);
        Semaphore_post(bArrived);
        Semaphore_pend(aArrived, BIOS_WAIT_FOREVER);
        Task_sleep(10);

        /* statement b2 */

        finishCount++;
        if (finishCount == 8) {
            // This task will run only 8 times in the loop then
            // the whole system will abort to see the logs on the Console
            System_printf("Calling BIOS_exit\n");
            BIOS_exit(0);
        }
    }
}

