/*
   Copyright (c) 2014-2020 by Plexim GmbH
   All rights reserved.

   A free license is granted to anyone to use this software for any legal
   non safety-critical purpose, including commercial applications, provided
   that:
   1) IT IS NOT USED TO DIRECTLY OR INDIRECTLY COMPETE WITH PLEXIM, and
   2) THIS COPYRIGHT NOTICE IS PRESERVED in its entirety.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
   OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
 */

#include "includes.h"

// Pil includes
#include "pil.h"

#include "main.h"
#include "sci.h"
#include "dispatcher.h"
#include "dio.h"
#include "pwm.h"
#include "gatedriver.h"
#include "power.h"

#include "hal.h"

// function prototypes
extern void DevInit(Uint16 clock_source, Uint16 imult, Uint16 fmult);
extern void MemCopy(Uint16 *SourceAddr, Uint16 *SourceEndAddr, Uint16 *DestAddr);
extern void InitFlashHz(Uint32 clkHz);

// linker addresses, needed to copy code from flash to ram
extern Uint16 RamfuncsLoadStart, RamfuncsLoadEnd, RamfuncsRunStart;

#define SCOPE_BUF_SIZE 2000
#define SCOPE_MAX_TRACE_WIDTH_IN_WORDS 5
#pragma DATA_SECTION(ScopeBuffer, "scope")
uint16_t ScopeBuffer[SCOPE_BUF_SIZE];
extern void PIL_setAndConfigScopeBuffer(PIL_Handle_t aPilHandle, uint16_t* aBufPtr, uint16_t aBufSize, uint16_t aMaxTraceWidthInWords);

PIL_Obj_t PilObj;
PIL_Handle_t PilHandle;

DIO_Obj_t DoutDrvEnableObj;
GDRV_Obj_t GdrvObj;
GDRV_Handle_t GdrvHandle;

SCI_Obj_t SciObj;
SCI_Handle_t SciHandle;

#ifdef LED_GPIO
DIO_Obj_t DoutLedObj;
DIO_Handle_t DoutLedHandle;

#define LED_BLINK_PERIOD_2 (TASK2_HZ/2)
volatile uint16_t LedTimer = 0;

static void BlinkLed(void)
{
    LedTimer++;
    if(LedTimer > LED_BLINK_PERIOD_2)
    {
        DIO_toggle(DoutLedHandle);
        LedTimer = 0;
    }
}
#endif

static void SciPoll(PIL_Handle_t aHandle)
{
    if(SCI_breakOccurred(SciHandle)){
        SCI_reset(SciHandle);
    }

    while(SCI_rxReady(SciHandle))
    {
        // assuming that there will be a "break" when FIFO is empty
        PIL_SERIAL_IN(aHandle, (int16)SCI_getChar(SciHandle));
    }

    int16_t ch;
    if(!SCI_txIsBusy(SciHandle)){
        if(PIL_SERIAL_OUT(aHandle, &ch))
        {
            SCI_putChar(SciHandle, ch);
        }
    }
}

void Task1()
{
    CONTROL_TASK1();
    PWR_syncEnableSwitching();
}

void Task2()
{
    CONTROL_TASK2();
    PWR_fsm();
#ifdef LED_GPIO
    BlinkLed();
#endif
}

DISPR_TaskObj_t Tasks[2];
static void Tasks12(void * const aParam)
{
    switch(*(int *)aParam){
        case 0:
            Task1();
            break;

        case 1:
            Task2();
            break;
    }
}

void TaskB()
{
    CONTROL_BACKGROUND();
    PWR_background();
}

void main(void)
{
	// low level hardware configuration
	DevInit(PLL_SRC, PLL_IMULT, PLL_FMULT);
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	InitFlashHz(SYSCLK_HZ); // this assumes that clock is exact, use 3% safety margin for INT OSC

	// disable all interrupts
	DINT;
	IER = 0x0000;
	IFR = 0x0000;

	EALLOW;
	CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 1;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER1 = 1;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	EDIS;

    // configure serial line
    SciHandle = SCI_init(&SciObj, sizeof(SciObj));
    SCI_configure(SciHandle, SCI_PORT, LSPCLK_HZ);
    (void)SCI_setupPort(SciHandle, SCI_BAUD_RATE);

	// initialize PIL framework
    PilHandle = PIL_init(&PilObj, sizeof(PilObj));
    PIL_setGuid(PilHandle, PIL_GUID_PTR);
    PIL_setCtrlCallback(PilHandle, (PIL_CtrlCallbackPtr_t)PilCallback);
#ifdef PARALLEL_COM_PROTOCOL
    PIL_configureParallelCom(PilHandle, PARALLEL_COM_PROTOCOL, PARALLEL_COM_BUF_ADDR, PARALLEL_COM_BUF_LEN);
#else
    PIL_setSerialComCallback(PilHandle, (PIL_CommCallbackPtr_t)SciPoll);
#endif
#ifdef SCOPE_BUF_SIZE
    PIL_setAndConfigScopeBuffer(PilHandle, (uint16_t *)&ScopeBuffer, SCOPE_BUF_SIZE, SCOPE_MAX_TRACE_WIDTH_IN_WORDS);
#else
    PIL_setAndConfigScopeBuffer(PilHandle, (uint16_t *)0, 0, 0);
#endif

#ifndef NO_PREP_TOOL
    PilInitOverrideProbes();
    PilInitCalibrations();
#endif

    // Led
#ifdef LED_GPIO
    DIO_sinit();
    DoutLedHandle = DIO_init(&DoutLedObj,sizeof(DoutLedObj));
    DIO_configureOut(DoutLedHandle, LED_GPIO, false);
#endif

	// gate-driver
	GdrvHandle = GDRV_init(&GdrvObj,sizeof(GdrvObj));
	DIO_Handle_t dioHandle = DIO_init(&DoutDrvEnableObj,sizeof(DoutDrvEnableObj));
#ifdef DOUT_PS_ENABLE_GPIO
    DIO_configureOut(dioHandle, DOUT_PS_ENABLE_GPIO, true);
#else
    dioHandle = DIO_obtainDummyWrite();
#endif
    GDRV_assignPin(GdrvHandle, GDRV_Pin_EnableOut, dioHandle, GDRV_Pin_ActiveLow);
	GDRV_powerup(GdrvHandle);

	// powerstage
	PWR_sinit();
	PWR_configure(GdrvHandle, TASK2_HZ);

	// HAL for controls
	HAL_sinit();
	HAL_configure();

	// dispatcher
	DISPR_sinit();
	DISPR_configure((uint32_t)(SYSCLK_HZ/TASK1_HZ), PilHandle, &Tasks[0], sizeof(Tasks)/sizeof(DISPR_TaskObj_t));
	DISPR_registerIdleTask(&TaskB);
	DISPR_setPowerupDelay(1);

	// controls
	CTRL_sinit();

	DISPR_setupTimer(0, (uint32_t)(SYSCLK_HZ/TASK1_HZ));

	{
	    static int taskId = 0;
	    DISPR_registerTask(0, &Tasks12, (uint32_t)(SYSCLK_HZ/TASK1_HZ), (void *)&taskId);
	}

	{
	    static int taskId = 1;
	    DISPR_registerTask(1, &Tasks12, (uint32_t)(SYSCLK_HZ/TASK2_HZ), (void *)&taskId);
	}

	DISP_CONFIGURE_TRIGGER();

	// get ready for normal operation
	PIL_requestReadyMode(PilHandle);

	// enable interrupts
	EINT;   // global
	ERTM;   // real-time

	// go!
	DISPR_start(); // will not return

	PLX_ASSERT(0); // should never get here
}
