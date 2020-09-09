/*
   Copyright (c) 2020 by Plexim GmbH
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

#include "control.h"
#include "pu.h"
#include "calib.h"

#include "hal.h"
#include "power.h"
#include "dispatcher.h"
#include "macros.h"

#include "svpwm.h"
#include "svpwm3level.h"
#include "pidq.h"
#include "vector.h"
#include "filter.h"
#include "angle.h"
#include "pll.h"
#include "fp_math.h"

#pragma diag_suppress 179 // enter_NPCSOL_SYSTEM_FSM_STATE_CRITICAL_FAULT, enter_NPCSOL_SYSTEM_FSM_STATE_CALIBRATING declared but not referenced

extern void PIL_SCOPE_trigger(void);
typedef bool (*WriteInterlockPtr_t)(uint16_t flag);
extern PIL_setWriteInterlockHandler(WriteInterlockPtr_t writeIntLockPtr);

#ifndef PIL_PREP_TOOL
static void CTRL_configureTask1(CTRL_Handle_t aHandle);
static void CTRL_resetTask1(CTRL_Handle_t aHandle);
static void CTRL_resetTask2(CTRL_Handle_t aHandle);
static void CTRL_resetTaskB(CTRL_Handle_t aHandle);
static void CTRL_resetSystemFsm(CTRL_Handle_t aHandle);
static void CTRL_systemFsm(CTRL_Handle_t aHandle);
#endif

static uint16_t CtrlCalibsAreDirty;
static bool CTRL_CalibWriteInterlock(uint16_t flag);

#define CTRL_INV_SQRT3_Q15 (int16_t)(1.0 / 1.73 * 32768.0)

CTRL_Obj_t Ctrl;
#ifndef PIL_PREP_TOOL
CTRL_Handle_t CtrlHandle;

static CTRL_Handle_t CTRL_init(void *aMemory, const size_t aNumBytes);
static void CTRL_configure(CTRL_Handle_t aHandle, uint16_t aControlHz);
#endif

static bool CTRL_CalibWriteInterlock(uint16_t flag){
    // we register that calibrations have been modified
    CtrlCalibsAreDirty = true;
    return true;
}

void CTRL_sinit(){
    CtrlHandle = CTRL_init(&Ctrl,sizeof(Ctrl));
    CTRL_configure(CtrlHandle, TASK1_HZ);

    CtrlCalibsAreDirty = false;
    PIL_setWriteInterlockHandler(CTRL_CalibWriteInterlock);
}

static CTRL_Handle_t CTRL_init(void *aMemory, const size_t aNumBytes)
{
	CTRL_Handle_t handle;

	if(aNumBytes < sizeof(CTRL_Obj_t))
		return((CTRL_Handle_t)NULL);

	// set handle
	handle = (CTRL_Handle_t)aMemory;

	return handle;
}

static void CTRL_configure(CTRL_Handle_t aHandle, uint16_t aControlHz)
{
	CTRL_Obj_t *obj = (CTRL_Obj_t *)aHandle;

	// enable signal
	HAL_setupDigitalIn(0, 40); // pin 89, DS21 - enable signal
	HAL_setupDigitalIn(1, 41); // pin 91, DS20 - set-point (not used)

	EALLOW;
	InputXbarRegs.INPUT5SELECT = 24; // connect GPIO24 to XBAR5 (DOUT25 on RT-Box)
	EDIS;

	// configure PWM1 in triangle mode (soc='z')
	{
	    PWM_Params_t *params = HAL_getDefaultPwmParams();
	    params->outMode = PWM_OUTPUT_MODE_DUAL;
	    params->reg.TBPRD = CONTROL_PWM_PERIOD;
	    params->reg.TBCTL.bit.CTRMODE = 2;
	    params->reg.ETSEL.bit.SOCASEL = 1;
	    HAL_setupPwm(0, 1, params);
        HAL_setPwmDeadtimeCounts(0, CONTROL_PWM_DEADTIME, CONTROL_PWM_DEADTIME);
        HAL_setPwmSequence(0, 1);
	}
	// configure PWM2 in triangle mode (soc='z')
	{
        PWM_Params_t *params = HAL_getDefaultPwmParams();
        params->outMode = PWM_OUTPUT_MODE_DUAL;
        params->reg.TBPRD = CONTROL_PWM_PERIOD;
        params->reg.TBCTL.bit.CTRMODE = 2;
        params->reg.ETSEL.bit.SOCASEL = 1;
	    HAL_setupPwm(1, 2, params);
        HAL_setPwmDeadtimeCounts(1, CONTROL_PWM_DEADTIME, CONTROL_PWM_DEADTIME);
        HAL_setPwmSequence(1, 1);
	}
	// configure PWM3 in triangle mode (soc='z')
	{
        PWM_Params_t *params = HAL_getDefaultPwmParams();
        params->outMode = PWM_OUTPUT_MODE_DUAL;
        params->reg.TBPRD = CONTROL_PWM_PERIOD;
        params->reg.TBCTL.bit.CTRMODE = 2;
        params->reg.ETSEL.bit.SOCASEL = 1;
	    HAL_setupPwm(2, 3, params);
        HAL_setPwmDeadtimeCounts(2, CONTROL_PWM_DEADTIME, CONTROL_PWM_DEADTIME);
        HAL_setPwmSequence(2, 1);
	}
	// configure PWM4 in triangle mode (soc='z')
	{
        PWM_Params_t *params = HAL_getDefaultPwmParams();
        params->outMode = PWM_OUTPUT_MODE_DUAL;
        params->reg.TBPRD = CONTROL_PWM_PERIOD;
        params->reg.TBCTL.bit.CTRMODE = 2;
        params->reg.ETSEL.bit.SOCASEL = 1;
	    HAL_setupPwm(3, 4, params);
        HAL_setPwmDeadtimeCounts(3, CONTROL_PWM_DEADTIME, CONTROL_PWM_DEADTIME);
        HAL_setPwmSequence(3, 1);
	}
	// configure PWM5 in triangle mode (soc='z')
	{
        PWM_Params_t *params = HAL_getDefaultPwmParams();
        params->outMode = PWM_OUTPUT_MODE_DUAL;
        params->reg.TBPRD = CONTROL_PWM_PERIOD;
        params->reg.TBCTL.bit.CTRMODE = 2;
        params->reg.ETSEL.bit.SOCASEL = 1;
	    HAL_setupPwm(4, 5, params);
        HAL_setPwmDeadtimeCounts(4, CONTROL_PWM_DEADTIME, CONTROL_PWM_DEADTIME);
        HAL_setPwmSequence(4, 1);
	}
	// configure PWM6 in triangle mode (soc='z')
	{
        PWM_Params_t *params = HAL_getDefaultPwmParams();
        params->outMode = PWM_OUTPUT_MODE_DUAL;
        params->reg.TBPRD = CONTROL_PWM_PERIOD;
        params->reg.TBCTL.bit.CTRMODE = 2;
        params->reg.ETSEL.bit.SOCASEL = 1;
	    HAL_setupPwm(5, 6, params);
        HAL_setPwmDeadtimeCounts(5, CONTROL_PWM_DEADTIME, CONTROL_PWM_DEADTIME);
        HAL_setPwmSequence(5, 1);
	}
	// configure PWM7 in triangle mode (soc='z')
	{
        PWM_Params_t *params = HAL_getDefaultPwmParams();
        params->outMode = PWM_OUTPUT_MODE_DUAL;
        params->reg.TBPRD = CONTROL_PWM_PERIOD;
        params->reg.TBCTL.bit.CTRMODE = 2;
        params->reg.ETSEL.bit.SOCASEL = 1;
	    HAL_setupPwm(6, 7, params);
        HAL_setPwmDeadtimeCounts(6, CONTROL_PWM_DEADTIME, CONTROL_PWM_DEADTIME);
        HAL_setPwmSequence(6, 1);
	}
	// configure ADC B
	{
	    AIN_AdcParams_t *params = HAL_getDefaultAdcParams();
	    HAL_setupAdc(1, 1, params);
	}
	// configure SOC1 of ADC-B to measure ADCIN1
	{
	    AIN_ChannelParams_t *paramsChannel = HAL_getDefaultAinChannelParams();
	    // set SOC trigger to CPU Timer 0
	    paramsChannel->ADCSOCxCTL.bit.TRIGSEL = 5;
	    paramsChannel->ADCSOCxCTL.bit.ACQPS = 15;
	    HAL_setupAnalogIn(1, 1, 1, paramsChannel);
	}
	// configure SOC2 of ADC-B to measure ADCIN2
	{
	    AIN_ChannelParams_t *paramsChannel = HAL_getDefaultAinChannelParams();
	    // set SOC trigger to CPU Timer 0
	    paramsChannel->ADCSOCxCTL.bit.TRIGSEL = 5;
	    paramsChannel->ADCSOCxCTL.bit.ACQPS = 15;
	    HAL_setupAnalogIn(1, 2, 2, paramsChannel);
	}
	// configure SOC0 of ADC-B to measure ADCIN0
	{
	    AIN_ChannelParams_t *paramsChannel = HAL_getDefaultAinChannelParams();
	    // set SOC trigger to CPU Timer 0
	    paramsChannel->ADCSOCxCTL.bit.TRIGSEL = 5;
	    paramsChannel->ADCSOCxCTL.bit.ACQPS = 15;
	    HAL_setupAnalogIn(1, 0, 0, paramsChannel);
	}
	// configure ADC A
	{
	    AIN_AdcParams_t *params = HAL_getDefaultAdcParams();
	    HAL_setupAdc(0, 0, params);
	}
	// configure SOC1 of ADC-A to measure ADCIN1
	{
	    AIN_ChannelParams_t *paramsChannel = HAL_getDefaultAinChannelParams();
	    // set SOC trigger to CPU Timer 0
	    paramsChannel->ADCSOCxCTL.bit.TRIGSEL = 5;
	    paramsChannel->ADCSOCxCTL.bit.ACQPS = 15;
	    HAL_setupAnalogIn(0, 1, 1, paramsChannel);
	}
	// configure SOC2 of ADC-A to measure ADCIN2
	{
	    AIN_ChannelParams_t *paramsChannel = HAL_getDefaultAinChannelParams();
	    // set SOC trigger to CPU Timer 0
	    paramsChannel->ADCSOCxCTL.bit.TRIGSEL = 5;
	    paramsChannel->ADCSOCxCTL.bit.ACQPS = 15;
	    HAL_setupAnalogIn(0, 2, 2, paramsChannel);
	}
	// configure SOC0 of ADC-A to measure ADCIN0
	{
	    AIN_ChannelParams_t *paramsChannel = HAL_getDefaultAinChannelParams();
	    // set SOC trigger to CPU Timer 0
	    paramsChannel->ADCSOCxCTL.bit.TRIGSEL = 5;
	    paramsChannel->ADCSOCxCTL.bit.ACQPS = 15;
	    HAL_setupAnalogIn(0, 0, 0, paramsChannel);
	}
	// configure ADC D
	{
	    AIN_AdcParams_t *params = HAL_getDefaultAdcParams();
	    // configure SOC2 to trigger interrupt
	    params->ADCINTSEL1N2.bit.INT1SEL = 2;
	    HAL_setupAdc(2, 3, params);
	}
	// configure SOC1 of ADC-D to measure ADCIN1
	{
	    AIN_ChannelParams_t *paramsChannel = HAL_getDefaultAinChannelParams();
	    // set SOC trigger to CPU Timer 0
	    paramsChannel->ADCSOCxCTL.bit.TRIGSEL = 5;
	    paramsChannel->ADCSOCxCTL.bit.ACQPS = 15;
	    HAL_setupAnalogIn(2, 1, 1, paramsChannel);
	}
	// configure SOC2 of ADC-D to measure ADCIN2
	{
	    AIN_ChannelParams_t *paramsChannel = HAL_getDefaultAinChannelParams();
	    // set SOC trigger to CPU Timer 0
	    paramsChannel->ADCSOCxCTL.bit.TRIGSEL = 5;
	    paramsChannel->ADCSOCxCTL.bit.ACQPS = 15;
	    HAL_setupAnalogIn(2, 2, 2, paramsChannel);
	}
	// configure SOC0 of ADC-D to measure ADCIN0
	{
	    AIN_ChannelParams_t *paramsChannel = HAL_getDefaultAinChannelParams();
	    // set SOC trigger to CPU Timer 0
	    paramsChannel->ADCSOCxCTL.bit.TRIGSEL = 5;
	    paramsChannel->ADCSOCxCTL.bit.ACQPS = 15;
	    HAL_setupAnalogIn(2, 0, 0, paramsChannel);
	}

	uint16_t pwmPeriod = CONTROL_PWM_PERIOD;

	obj->tsk1.Fs = aControlHz;

	obj->PuVars.wrefTsScaling = (int16_t)(NPCSOL_W_REF / (double)(obj->tsk1.Fs) * (double)(PLX_PIDQ_R_ONE) + 0.5);
	obj->PuVars.frefTsScaling = (int16_t)(NPCSOL_F_REF / (double)(obj->tsk1.Fs) * (double)(PLX_ANGLE_R_ONE) + 0.5);

	obj->Svpwm3Handle = PLX_SVPWM3L_init(&obj->Svpwm3Obj,sizeof(obj->Svpwm3Obj));
	PLX_SVPWM3L_configure(obj->Svpwm3Handle, pwmPeriod);

	obj->PiDqHandle = PLX_PIDQ_init(&obj->PiDqObj,sizeof(obj->PiDqObj));
	obj->PllHandle = PLX_PLL_init(&obj->PllObj, sizeof(obj->PllObj));

	// this is for feed-forward angle reference (PLL bypass)
	obj->fluxAngleHandle = PLX_ANGLE_init(&obj->fluxAngleObj,sizeof(obj->fluxAngleObj));
	PLX_ANGLE_configure(obj->fluxAngleHandle, obj->PuVars.frefTsScaling, 0);

	CTRL_reset(aHandle);

	obj->vdcFilterHandle = PLX_FILT_init(&obj->vdcFilterObj, sizeof(obj->vdcFilterObj));
	obj->vmidFilterHandle = PLX_FILT_init(&obj->vmidFilterObj, sizeof(obj->vmidFilterObj));
}

static void CTRL_configureTask1(CTRL_Handle_t aHandle)
{
	CTRL_Obj_t *obj = (CTRL_Obj_t *)aHandle;

	PLX_PIDQ_configure(obj->PiDqHandle, obj->PuVars.wrefTsScaling, Calibs.KpD, Calibs.KiD, Calibs.KpQ, Calibs.KiQ);

	PLX_PLL_configure(obj->PllHandle, obj->PuVars.frefTsScaling, Calibs.KpPll, Calibs.KiPll,
			-Calibs.omegaMaxPll, Calibs.omegaMaxPll,
			Calibs.lockAnglePll, Calibs.lockTicksPll, Calibs.unlockAnglePll, Calibs.unlockTicksPll);
	PLX_PLL_reset(obj->PllHandle, Calibs.omega0Pll);

	PLX_FILT_configure(obj->vdcFilterHandle, Calibs.KdcFilter, 0);//Standard K = 0.177799/2916
	PLX_FILT_configure(obj->vmidFilterHandle, Calibs.KmidFilter, 0);//Standard K = 0.177799/2916
}

static void CTRL_resetTask1(CTRL_Handle_t aHandle)
{
	CTRL_Obj_t *obj = (CTRL_Obj_t *)aHandle;

	obj->tsk1.stepCtr = 0;

	CTRL_configureTask1(aHandle);

	// offset calibration
	obj->tsk1.IuAccu = 0;
	obj->tsk1.IvAccu = 0;
	obj->tsk1.IwAccu = 0;

	obj->tsk1.IuOffset = Calibs.IuOffset;
	obj->tsk1.IvOffset = Calibs.IvOffset;
	obj->tsk1.IwOffset = Calibs.IwOffset;

	obj->tsk1.VguvOffset = Calibs.VguvOffset;
	obj->tsk1.VgvwOffset = Calibs.VgvwOffset;
	obj->tsk1.VgwuOffset = Calibs.VgwuOffset;

	obj->tsk1.AdcAccuCtr = 0;

	obj->tsk1.Cmp0 = 0;
	obj->tsk1.Cmp1 = 0;
	obj->tsk1.Cmp2 = 0;
	obj->tsk1.Cmp3 = 0;
	obj->tsk1.Cmp4 = 0;
	obj->tsk1.Cmp5 = 0;

	obj->tsk1.gridPllIsLocked = false;
	obj->tsk1.iqSetLast = 0;

	obj->tsk1.calibrateOffsetReq = true;

	obj->tsk1.closedLoopEnabled = 0;
	obj->tsk1.pllEnabled = true;
	obj->tsk1.IdSet = 0;
	obj->tsk1.IqSet = 0;
	obj->tsk1.VdFF = 0;
	obj->tsk1.VqFF = 0;
	obj->tsk1.weFF = 0;

	// for display purposes
	obj->Ain.Iu = 0;
	obj->Ain.Iv = 0;
	obj->Ain.Iw = 0;
	obj->Ain.Vguv = 0;
	obj->Ain.Vgvw = 0;
	obj->Ain.Vgwu = 0;
	obj->Ain.Vdc = 0;
	obj->Ain.Vmid = 0;
	obj->tsk1.uncalibrated = TRUE;

	//reset DC measurement filters
	PLX_FILT_reset(obj->vmidFilterHandle, 0);
	PLX_FILT_reset(obj->vdcFilterHandle, 0);
}

static void CTRL_resetTask2(CTRL_Handle_t aHandle)
{
	CTRL_Obj_t *obj = (CTRL_Obj_t *)aHandle;

	obj->tsk2.active = 0;
	obj->tsk2.stepCtr = 0;
	CTRL_resetSystemFsm(aHandle);

	// for display purposes
	obj->tsk2.thetaSms = 0;

}

static void CTRL_resetTaskB(CTRL_Handle_t aHandle)
{
	CTRL_Obj_t *obj = (CTRL_Obj_t *)aHandle;

	obj->tskB.backgroundTaskNumber = 0;
}

void CTRL_reset(CTRL_Handle_t aHandle)
{
	CTRL_resetTask1(aHandle);
	CTRL_resetTask2(aHandle);
	CTRL_resetTaskB(aHandle);
}

void CTRL_task1(CTRL_Handle_t aHandle)
{
	CTRL_Obj_t *obj = (CTRL_Obj_t *)aHandle;

	obj->tsk1.stepCtr++;

    // handle enabling of powerstage
    if(PWR_isEnabled()) {
        // flag will be reset further below, after ePWM update
        obj->tsk1.calibrateOffsetReq = FALSE;
    } else {
        PLX_PIDQ_reset(obj->PiDqHandle);
    }

	//adding code

    int16_t VALUE_VARin;
    VALUE_VARin = obj -> tsk1.VARin;
    SET_OPROBE(obj->tsk1.VARout, VALUE_VARin + 1);

	//end of adding

	// digital inputs

	// analog inputs
	SET_OPROBE(obj->Ain.IuADC, HAL_getAnalogIn(0, 0));
	SET_OPROBE(obj->Ain.IvADC, HAL_getAnalogIn(0, 1));
	SET_OPROBE(obj->Ain.IwADC, HAL_getAnalogIn(0, 2));

	SET_OPROBE(obj->Ain.VguvADC, HAL_getAnalogIn(1, 0));
	SET_OPROBE(obj->Ain.VgvwADC, HAL_getAnalogIn(1, 1));
	SET_OPROBE(obj->Ain.VgwuADC, HAL_getAnalogIn(1, 2));

	SET_OPROBE(obj->Ain.VdcADC, HAL_getAnalogIn(2, 0));
	SET_OPROBE(obj->Ain.VmidADC, HAL_getAnalogIn(2, 1));

	// process current measurements
	if(obj->tsk1.calibrateOffsetReq)
	{
		obj->tsk1.IuAccu += obj->Ain.IuADC;
		obj->tsk1.IvAccu += obj->Ain.IvADC;
		obj->tsk1.IwAccu += obj->Ain.IwADC;
		obj->tsk1.AdcAccuCtr++;
		if(obj->tsk1.AdcAccuCtr == 256)
		{
			obj->tsk1.IuOffset = (uint16_t)(obj->tsk1.IuAccu >> 8);
			obj->tsk1.IvOffset = (uint16_t)(obj->tsk1.IvAccu >> 8);
			obj->tsk1.VguvOffset = Calibs.VguvOffset; // cannot really calibrate unless we have output breaker
			obj->tsk1.VgvwOffset = Calibs.VgvwOffset; // cannot really calibrate unless we have output breaker
			obj->tsk1.VgwuOffset = Calibs.VgwuOffset; // cannot really calibrate unless we have output breaker
			obj->tsk1.IuAccu = 0;
			obj->tsk1.IvAccu = 0;
			obj->tsk1.IwAccu = 0;
			obj->tsk1.AdcAccuCtr = 0;
			obj->tsk1.uncalibrated = FALSE;//at least one calibration finished
		}
	}

	// process current measurements
	int16_t iu, iv, iw;
	iu = obj->Ain.IuADC - obj->tsk1.IuOffset;
	iv = obj->Ain.IvADC - obj->tsk1.IvOffset;
	iw = -(iu+iv);

	iu = Q_MUL(iu, Calibs.currentGain, NPCSOL_Q_IV);
	iv = Q_MUL(iv, Calibs.currentGain, NPCSOL_Q_IV);
	iw = Q_MUL(iw, Calibs.currentGain, NPCSOL_Q_IV);

	SET_OPROBE(obj->Ain.Iu, iu);
	SET_OPROBE(obj->Ain.Iv, iv);
	SET_OPROBE(obj->Ain.Iw, iw);

	// process DC voltage measurements
	SET_OPROBE(obj->Ain.Vdc_p,  Q_MUL(obj->Ain.VdcADC, Calibs.dcVoltageGain, NPCSOL_Q_IV));
	SET_OPROBE(obj->Ain.Vmid_p, Q_MUL(obj->Ain.VmidADC, Calibs.dcVoltageGain, NPCSOL_Q_IV));

	// process DC voltage filtering
	//PLX_FILT_calculateOutput(obj->vdcFilterHandle, obj->Ain.Vdc_p, &obj->Ain.Vdc);
	SET_OPROBE(obj->Ain.Vdc,  obj->Ain.Vdc_p);
	//PLX_FILT_calculateOutput(obj->vmidFilterHandle, obj->Ain.Vmid_p, &obj->Ain.Vmid);
	SET_OPROBE(obj->Ain.Vmid, obj->Ain.Vmid_p);

	// process grid voltage measurement
	int16_t vguv, vgvw, vgwu;
	vguv = obj->Ain.VguvADC - obj->tsk1.VguvOffset;
	vgvw = obj->Ain.VgvwADC - obj->tsk1.VgvwOffset;
	vgwu = obj->Ain.VgwuADC - obj->tsk1.VgwuOffset; // -(vguv+vgvw);

	vguv = Q_MUL(vguv, Calibs.acVoltageGain, NPCSOL_Q_IV);
	vgvw = Q_MUL(vgvw, Calibs.acVoltageGain, NPCSOL_Q_IV);
	vgwu = Q_MUL(vgwu, Calibs.acVoltageGain, NPCSOL_Q_IV);

	SET_OPROBE(obj->Ain.Vguv, vguv);
	SET_OPROBE(obj->Ain.Vgvw, vgvw);
	SET_OPROBE(obj->Ain.Vgwu, vgwu);

	int16_t vga, vgb;
	vga = obj->Ain.Vguv;
	vgb = obj->Ain.Vgvw - obj->Ain.Vgwu;
	vgb = Q_MUL(vgb, CTRL_INV_SQRT3_Q15, 15);

	SET_OPROBE(obj->tsk1.VgridA, vga);
	SET_OPROBE(obj->tsk1.VgridB, vgb);

	// calculate phase voltage amplitude (correct for line-line measurement)
	int16_t vga2 = Q_MUL(obj->tsk1.VgridA, obj->tsk1.VgridA, NPCSOL_Q_IV);
	int16_t vgb2 = Q_MUL(obj->tsk1.VgridB, obj->tsk1.VgridB, NPCSOL_Q_IV);
	int16_t vg = Q_MUL(PLX_FPMATH_sqrtQ10(vga2+vgb2), CTRL_INV_SQRT3_Q15, 15);
	SET_OPROBE(obj->tsk1.Vgrid, vg);

	// PLL
	obj->tsk1.gridVoltageAngle = PLX_FPMATH_atan2(obj->tsk1.VgridA, obj->tsk1.VgridB)
									- (PLX_ANGLE_R_ANGLE/12); // compensate for line-line measurement
	bool_t isLocked;
	if(obj->tsk1.pllEnabled)
	{
		PLX_PLL_udpate(obj->PllHandle, obj->tsk1.gridVoltageAngle, &obj->tsk1.gridPllAngle, &obj->tsk1.gridPllOmega, &isLocked);
	}
	else
	{
		// Feedforward operation
		PLX_PLL_reset(obj->PllHandle, Calibs.omega0Pll);
		PLX_ANGLE_accumulate(obj->fluxAngleHandle, obj->tsk1.weFF, &obj->tsk1.gridPllAngle);
		obj->tsk1.gridPllOmega = obj->tsk1.weFF;
		isLocked = true;
	}
	SET_OPROBE(obj->tsk1.gridPllIsLocked, isLocked);

	// align synchronous frame
	SET_OPROBE(obj->tsk1.fluxPosition, obj->tsk1.gridPllAngle - (PLX_ANGLE_R_ANGLE/4)); // make q-axis produce real power

	// calculate maximal achievable phase voltage based on DC-link voltage (max is 1/sqrt(3))
	int16_t vsmax = Q_MUL(obj->Ain.Vdc, CTRL_INV_SQRT3_Q15, 15);
	// apply margin for current regulator
	vsmax = Q_MUL(vsmax, (int16_t)(0.95 * 32768.0), 15);
	SET_OPROBE(obj->tsk1.Vsmax, vsmax);

	// current clark transformation
	int16_t ia, ib;
	ia = obj->Ain.Iu; // (2*obj->Ain.Iu - obj->Ain.Iv - obj->Ain.Iw)/3
	ib = obj->Ain.Iv - obj->Ain.Iw;
	ib = Q_MUL(ib, CTRL_INV_SQRT3_Q15, 15);

	SET_OPROBE(obj->tsk1.Ia, ia);
	SET_OPROBE(obj->tsk1.Ib, ib);

	// transform currents into rotating frame
	int16_t id, iq;
	PLX_FPMATH_lookupSinAndCos(obj->tsk1.fluxPosition, &obj->tsk1.fluxPosSin, &obj->tsk1.fluxPosCos);
	PLX_VECT_parkRot(obj->tsk1.Ia, obj->tsk1.Ib, &id, &iq, obj->tsk1.fluxPosSin, obj->tsk1.fluxPosCos);
	SET_OPROBE(obj->tsk1.Id, id);
	SET_OPROBE(obj->tsk1.Iq, iq);

	// determine errors
	SET_OPROBE(obj->tsk1.IdErr, obj->tsk1.IdSet - obj->tsk1.Id);
	SET_OPROBE(obj->tsk1.IqErr, obj->tsk1.IqSet - obj->tsk1.Iq);

	// update decoupling gains of current regulator (could happen at a lower rate, e.g. 1000 Hz)
	SET_OPROBE(obj->tsk1.we, obj->tsk1.gridPllOmega);
	PLX_PIDQ_updateCrossCouplingGains(obj->PiDqHandle,  obj->tsk1.we);
	PLX_PIDQ_applyCrossCouplingGains(obj->PiDqHandle); // this function must not be interrupted!

	// calculate regulator output
	int16_t vd = 0;
	int16_t vq = 0;
	if(obj->tsk1.closedLoopEnabled)
	{
		PLX_PIDQ_calculateOutput(obj->PiDqHandle, obj->tsk1.IdErr, obj->tsk1.IqErr, &vd, &vq);
	}
	// add feedforward term
	vd += obj->tsk1.VdFF;
	vq += obj->tsk1.VqFF;

	// saturate output based on maximal allowable phase voltage
	PLX_VECT_satVector(obj->tsk1.Vsmax, &vd, &vq);

	SET_OPROBE(obj->tsk1.Vd, vd);
	SET_OPROBE(obj->tsk1.Vq, vq);

	// transform voltages back into stationary frame
	int16_t va, vb;
	PLX_VECT_invParkRot(obj->tsk1.Vd, obj->tsk1.Vq, &va, &vb, obj->tsk1.fluxPosSin, obj->tsk1.fluxPosCos);
	SET_OPROBE(obj->tsk1.Va, va);
	SET_OPROBE(obj->tsk1.Vb, vb);

	// mid-point balancing (might make more sense to move this to task2 and filter voltages)
	SET_OPROBE(obj->tsk1.VmidErr,  (obj->Ain.Vdc >> 1) - obj->Ain.Vmid);
	int16_t midErr = obj->tsk1.VmidErr;

	if(midErr > Calibs.MidErrSat)
	{
		midErr = Calibs.MidErrSat; // saturate to prevent overflow
	}
	else if (midErr < -Calibs.MidErrSat)
	{
		midErr = -Calibs.MidErrSat; // saturate to prevent overflow
	}
	int16_t redVect2Adj = Q_MUL((midErr << 5), Calibs.KpMid, 0);
	if(redVect2Adj > (int16_t)(0.5 * (double)(32768)))
	{
		redVect2Adj = (int16_t)(0.5 * (double)(32768));
	}
	else if(redVect2Adj < (int16_t)(-0.5 * (double)(32768)))
	{
		redVect2Adj = (int16_t)(-0.5 * (double)(32768));
	}
	SET_OPROBE(obj->tsk1.redVector2Adj , redVect2Adj);

	// 3-level SVPWM
	int16_t svpwmLimitingGain;
	SET_OPROBE(obj->tsk1.redVector1Split, (uint16_t)(0.5 * (double)(32768)));
	SET_OPROBE(obj->tsk1.redVector2Split, (uint16_t)(0.5 * (double)(32768)- obj->tsk1.redVector2Adj)); // center of sub-hex

	uint16_t cmp0, cmp1, cmp2, cmp3, cmp4, cmp5;
	PLX_SVPWM3L_calcCompareValues(obj->Svpwm3Handle,
			obj->tsk1.Va, obj->tsk1.Vb, obj->Ain.Vdc,
			obj->tsk1.redVector1Split, obj->tsk1.redVector2Split,
			&cmp0, &cmp1, &cmp2,
			&cmp3, &cmp4, &cmp5,
			&svpwmLimitingGain);

	SET_OPROBE(obj->tsk1.Cmp0, cmp0);
	SET_OPROBE(obj->tsk1.Cmp1, cmp1);
	SET_OPROBE(obj->tsk1.Cmp2, cmp2);
	SET_OPROBE(obj->tsk1.Cmp3, cmp3);
	SET_OPROBE(obj->tsk1.Cmp4, cmp4);
	SET_OPROBE(obj->tsk1.Cmp5, cmp5);

	HAL_setPwm(1, obj->tsk1.Cmp0);
	HAL_setPwm(2, obj->tsk1.Cmp1);
	HAL_setPwm(3, obj->tsk1.Cmp2);
	HAL_setPwm(4, obj->tsk1.Cmp3);
	HAL_setPwm(5, obj->tsk1.Cmp4);
	HAL_setPwm(6, obj->tsk1.Cmp5);

	// make sure we got there in time
	obj->tsk1.thisPwmUpdatedTime = DISPR_getTimeStamp();

	// for debugging
	PLX_SVPWM3L_getSector(obj->Svpwm3Handle, &obj->tsk1.subHexagon, &obj->tsk1.zone);
	PLX_SVPWM3L_getVabT(obj->Svpwm3Handle, &obj->tsk1.VaT, &obj->tsk1.VbT);
	PLX_SVPWM3L_getCmprT(obj->Svpwm3Handle, &obj->tsk1.Cmp0T, &obj->tsk1.Cmp1T, &obj->tsk1.Cmp2T);

	// apply SVPWM saturation to regulator outputs
	SET_OPROBE(obj->tsk1.svpwmLimitingGain, svpwmLimitingGain);
	obj->tsk1.VdSat = Q_MUL(obj->tsk1.Vd, obj->tsk1.svpwmLimitingGain, 14);
	obj->tsk1.VqSat = Q_MUL(obj->tsk1.Vq, obj->tsk1.svpwmLimitingGain, 14);

	// update regulator integrator based on saturated regulator outputs
	if(obj->tsk1.closedLoopEnabled)
	{
		PLX_PIDQ_updateIntegrator(obj->PiDqHandle, obj->tsk1.VdSat - obj->tsk1.VdFF, obj->tsk1.VqSat - obj->tsk1.VqFF);
	}
	else
	{
		PLX_PIDQ_reset(obj->PiDqHandle);
	}

	// trigger scope (to analyze step response)
	if(obj->tsk1.iqSetLast != obj->tsk1.IqSet)
	{
		PIL_SCOPE_trigger();
		obj->tsk1.iqSetLast =obj->tsk1.IqSet;
	}
}

static void CTRL_resetSystemFsm(CTRL_Handle_t aHandle)
{
	CTRL_Obj_t *obj = (CTRL_Obj_t *)aHandle;

	obj->tsk2.sysFsm.state = NPCSOL_SYSTEM_FSM_STATE_POWERUP;
	obj->tsk2.sysFsm.fault = SYSTEM_FSM_FAULT_NONE;
}

static void CTRL_systemFsm(CTRL_Handle_t aHandle)
{
	CTRL_Obj_t *obj = (CTRL_Obj_t *)aHandle;

	int16_t vmidAbsOffset = (obj->Ain.Vdc >> 1) - obj->Ain.Vmid;
	if(vmidAbsOffset < 0){
		vmidAbsOffset = -vmidAbsOffset;
	}

	switch(obj->tsk2.sysFsm.state)
	{
		case NPCSOL_SYSTEM_FSM_STATE_POWERUP:
			// fallthrough

		enter_NPCSOL_SYSTEM_FSM_STATE_CALIBRATE: // TODO: onsider enforcing initial calib
			obj->tsk2.sysFsm.state = NPCSOL_SYSTEM_FSM_STATE_CALIBRATE;
		break;
		case NPCSOL_SYSTEM_FSM_STATE_CALIBRATE:
			goto enter_NPCSOL_SYSTEM_FSM_STATE_PRECHARGE_DC;

		enter_NPCSOL_SYSTEM_FSM_STATE_PRECHARGE_DC:
			obj->tsk2.sysFsm.state = NPCSOL_SYSTEM_FSM_STATE_PRECHARGE_DC;
			break;
		case NPCSOL_SYSTEM_FSM_STATE_PRECHARGE_DC:
			if(obj->Ain.Vdc >= Calibs.VdcOn)
			{
				goto enter_NPCSOL_SYSTEM_FSM_STATE_SYNCHRONIZE_AC;
			}
			break;

		enter_NPCSOL_SYSTEM_FSM_STATE_SYNCHRONIZE_AC:
			obj->tsk2.sysFsm.state = NPCSOL_SYSTEM_FSM_STATE_SYNCHRONIZE_AC;
			break;
		case NPCSOL_SYSTEM_FSM_STATE_SYNCHRONIZE_AC:
			if(obj->Ain.Vdc <= Calibs.VdcOff)
			{
				goto enter_NPCSOL_SYSTEM_FSM_STATE_PRECHARGE_DC;
			}
			if((obj->tsk1.Vgrid >= Calibs.VGridMin) && obj->tsk1.gridPllIsLocked && PWR_isReadyForEnable())
			{
				goto enter_NPCSOL_SYSTEM_FSM_STATE_DISABLED;
			}
			break;

		enter_NPCSOL_SYSTEM_FSM_STATE_DISABLED:
			obj->tsk2.sysFsm.state = NPCSOL_SYSTEM_FSM_STATE_DISABLED;
			PWR_enable(false);
			SET_OPROBE(obj->tsk2.IdSet, 0);
			SET_OPROBE(obj->tsk2.IqSet, 0);
		break;
		case NPCSOL_SYSTEM_FSM_STATE_DISABLED:
			if(!obj->tsk2.enable)
			{
				goto enter_NPCSOL_SYSTEM_FSM_STATE_READY;
			}
			break;

		enter_NPCSOL_SYSTEM_FSM_STATE_READY:
			obj->tsk2.sysFsm.state = NPCSOL_SYSTEM_FSM_STATE_READY;
			PWR_enable(false);
			SET_OPROBE(obj->tsk2.IdSet, 0);
			SET_OPROBE(obj->tsk2.IqSet, 0);
			DINT;
			// update offset calibration
			obj->tsk1.calibrateOffsetReq = true;
			obj->tsk1.uncalibrated = true;
			EINT;
			break;
		case NPCSOL_SYSTEM_FSM_STATE_READY:
			if(obj->Ain.Vdc <= Calibs.VdcOff)
			{
				obj->tsk2.sysFsm.fault = SYSTEM_FSM_FAULT_VDC_OUT_OF_RANGE;
				goto enter_NPCSOL_SYSTEM_FSM_STATE_FAULT;
			}
			if(obj->tsk1.Vgrid < Calibs.VGridMin)
			{
				obj->tsk2.sysFsm.fault = SYSTEM_FSM_FAULT_GRID_OUT_OF_RANGE;
				goto enter_NPCSOL_SYSTEM_FSM_STATE_FAULT;
			}
			if(!obj->tsk1.gridPllIsLocked)
			{
				obj->tsk2.sysFsm.fault = SYSTEM_FSM_FAULT_PLL_UNLOCKED;
				goto enter_NPCSOL_SYSTEM_FSM_STATE_FAULT;
			}
			if(obj->tsk2.enable && obj->tsk1.uncalibrated == FALSE)
			{
				goto enter_NPCSOL_SYSTEM_FSM_STATE_ENABLING;
			}
			break;

		enter_NPCSOL_SYSTEM_FSM_STATE_ENABLING:
			obj->tsk2.sysFsm.state = NPCSOL_SYSTEM_FSM_STATE_ENABLING;
			if(CtrlCalibsAreDirty)
			{
				// calibrations have been modified while powerstage was disabled
				CTRL_configureTask1(aHandle);
				CtrlCalibsAreDirty = false;
			}
			PWR_enable(true);
			break;
		case NPCSOL_SYSTEM_FSM_STATE_ENABLING:
			if(!obj->tsk2.enable)
			{
				goto enter_NPCSOL_SYSTEM_FSM_STATE_DISABLED;
			}
			if(PWR_isEnabled())
			{
				goto enter_NPCSOL_SYSTEM_FSM_STATE_ENABLED;
			}
			break;

		enter_NPCSOL_SYSTEM_FSM_STATE_ENABLED:
			obj->tsk2.sysFsm.state = NPCSOL_SYSTEM_FSM_STATE_ENABLED;
			break;
		case NPCSOL_SYSTEM_FSM_STATE_ENABLED:
			if(!obj->tsk2.enable)
			{
				goto enter_NPCSOL_SYSTEM_FSM_STATE_READY;
			}
			if(!PWR_isEnabled())
			{
				obj->tsk2.sysFsm.fault = SYSTEM_FSM_FAULT_PS_OFF;
				goto enter_NPCSOL_SYSTEM_FSM_STATE_FAULT;
			}
			if(obj->Ain.Vdc <= Calibs.VdcOff)
			{
				obj->tsk2.sysFsm.fault = SYSTEM_FSM_FAULT_VDC_OUT_OF_RANGE;
				goto enter_NPCSOL_SYSTEM_FSM_STATE_FAULT;
			}
			if(obj->tsk1.Vgrid < Calibs.VGridMin)
			{
				obj->tsk2.sysFsm.fault = SYSTEM_FSM_FAULT_GRID_OUT_OF_RANGE;
				goto enter_NPCSOL_SYSTEM_FSM_STATE_FAULT;
			}
			if(!obj->tsk1.gridPllIsLocked)
			{
				obj->tsk2.sysFsm.fault = SYSTEM_FSM_FAULT_PLL_UNLOCKED;
				goto enter_NPCSOL_SYSTEM_FSM_STATE_FAULT;
			}
			if(vmidAbsOffset > Calibs.MidErrMax)
			{
				obj->tsk2.sysFsm.fault = SYSTEM_FSM_FAULT_MID_OUT_OF_RANGE;
				goto enter_NPCSOL_SYSTEM_FSM_STATE_FAULT;
			}
			{
				// Slip mode frequency shift
				int16_t dF = Calibs.omega0Pll - obj->tsk1.gridPllOmega;
				if(dF > Calibs.SmsErrSat)
				{
					dF = Calibs.SmsErrSat;
				}
				else if (dF < -Calibs.SmsErrSat)
				{
					dF = -Calibs.SmsErrSat;
				}
				SET_OPROBE(obj->tsk2.thetaSms, Q_MUL(dF, Calibs.KpSms, 10 - (16-10)));

				int16_t sin, cos;
				PLX_FPMATH_lookupSinAndCos(obj->tsk2.thetaSms, &sin, &cos);
				int16_t idSet, iqSet;
				PLX_VECT_invParkRot(obj->tsk2.IdDesired, obj->tsk2.IqDesired, &idSet, &iqSet, sin, cos);

				SET_OPROBE(obj->tsk2.IdSet, idSet);
				SET_OPROBE(obj->tsk2.IqSet, iqSet);
			}
			break;

		enter_NPCSOL_SYSTEM_FSM_STATE_FAULT:
			obj->tsk2.sysFsm.state = NPCSOL_SYSTEM_FSM_STATE_FAULT;
			PWR_enable(false);
			break;
		case NPCSOL_SYSTEM_FSM_STATE_FAULT:
			if(!obj->tsk2.enable)
			{
				goto enter_NPCSOL_SYSTEM_FSM_STATE_FAULT_ACKN;
			}
			break;

		enter_NPCSOL_SYSTEM_FSM_STATE_FAULT_ACKN:
			obj->tsk2.sysFsm.state = NPCSOL_SYSTEM_FSM_STATE_FAULT_ACKN;
			obj->tsk2.sysFsm.fault = SYSTEM_FSM_FAULT_NONE;
			PWR_enable(false);
			break;
		case NPCSOL_SYSTEM_FSM_STATE_FAULT_ACKN:
			if(PWR_isReadyForEnable())
			{
				goto enter_NPCSOL_SYSTEM_FSM_STATE_DISABLED;
			}
			break;

		enter_NPCSOL_SYSTEM_FSM_STATE_CRITICAL_FAULT:
			obj->tsk2.sysFsm.state = NPCSOL_SYSTEM_FSM_STATE_CRITICAL_FAULT;
			PWR_enable(false);
			break;
		case NPCSOL_SYSTEM_FSM_STATE_CRITICAL_FAULT:
			break;
	}
}

void CTRL_task2(CTRL_Handle_t aHandle)
{
	CTRL_Obj_t *obj = (CTRL_Obj_t *)aHandle;

	obj->tsk2.active = true;
	obj->tsk2.stepCtr++;

	SET_OPROBE(obj->tsk2.VdFF, 0);
	SET_OPROBE(obj->tsk2.VdFF, 0);
	SET_OPROBE(obj->tsk2.VqFF, obj->tsk1.Vgrid); // note: this assumes positive frequency!
	SET_OPROBE(obj->tsk2.IdDesired, 0);
	SET_OPROBE(obj->tsk2.weFF, Calibs.omega0Pll);
	SET_OPROBE(obj->tsk2.enableClosedLoop, 1);
	SET_OPROBE(obj->tsk2.enablePll, 1);

#ifdef RUN_WITH_RTBOX
	if(!HAL_getDigitalIn(0))
	{
		SET_OPROBE(obj->tsk2.IqDesired, 0);
	}
	else
	{
		SET_OPROBE(obj->tsk2.IqDesired, Calibs.IqDesiredFixed);
#if 0
		if(!HAL_getDigitalIn(1))
		{
			//Low Reference
			SET_OPROBE(obj->tsk2.IqDesired, Calibs.IqDesiredFixed/2);
		}
		else
		{
			//High Reference
			SET_OPROBE(obj->tsk2.IqDesired, Calibs.IqDesiredFixed);
		}
#endif
	}
#else
	SET_OPROBE(obj->tsk2.IqDesired, 0);
#endif

	SET_OPROBE(obj->tsk2.enable, (obj->tsk2.IdDesired != 0) || (obj->tsk2.IqDesired != 0));
	CTRL_systemFsm(aHandle);

	// export set-points
	DINT;
	obj->tsk1.IdSet = obj->tsk2.IdSet;
	obj->tsk1.IqSet = obj->tsk2.IqSet;
	obj->tsk1.VdFF = obj->tsk2.VdFF;
	obj->tsk1.VqFF = obj->tsk2.VqFF;
	obj->tsk1.weFF = obj->tsk2.weFF;
	obj->tsk1.closedLoopEnabled = obj->tsk2.enableClosedLoop;
	obj->tsk1.pllEnabled = obj->tsk2.enablePll;
	EINT;

	obj->tsk2.active = false;
}

void CTRL_background(CTRL_Handle_t aHandle)
{
	CTRL_Obj_t *obj = (CTRL_Obj_t *)aHandle;

	switch(obj->tskB.backgroundTaskNumber)
	{
		case 0:
			break;
		case 1:
			break;
		case 2:
			break;
	}
	obj->tskB.backgroundTaskNumber++;
	if(obj->tskB.backgroundTaskNumber > 2)
	{
		obj->tskB.backgroundTaskNumber = 0;
	}
}
