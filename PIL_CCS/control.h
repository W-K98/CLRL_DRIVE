/*
   Copyright (c) 2014-2016 by Plexim GmbH
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
#include "pil.h"

#ifndef CTRL_TEST_H_
#define CTRL_TEST_H_

#define CTRL_TS 0.0001

#include "svpwm3level.h"
#include "pidq.h"
#include "pll.h"
#include "angle.h"
#include "filter.h"

struct CTRL_ANALOG_INPUTS {
	PIL_OVERRIDE_PROBE(uint16_t, IuADC, 0, 1.0, "");
	PIL_OVERRIDE_PROBE(uint16_t, IvADC, 0, 1.0, "");
	PIL_OVERRIDE_PROBE(uint16_t, IwADC, 0, 1.0, "");
	PIL_OVERRIDE_PROBE(uint16_t, VdcADC, 0, 1.0, "");
	PIL_OVERRIDE_PROBE(uint16_t, VmidADC, 0, 1.0, "");
	PIL_OVERRIDE_PROBE(uint16_t, VguvADC, 0, 1.0, "");
	PIL_OVERRIDE_PROBE(uint16_t, VgvwADC, 0, 1.0, "");
	PIL_OVERRIDE_PROBE(uint16_t, VgwuADC, 0, 1.0, "");

	PIL_OVERRIDE_PROBE(int16_t, Iu, NPCSOL_Q_IV, NPCSOL_I_REF, "A");
	PIL_OVERRIDE_PROBE(int16_t, Iv, NPCSOL_Q_IV, NPCSOL_I_REF, "A");
	PIL_OVERRIDE_PROBE(int16_t, Iw, NPCSOL_Q_IV, NPCSOL_I_REF, "A");
	PIL_OVERRIDE_PROBE(int16_t, Vdc_p, NPCSOL_Q_IV, NPCSOL_VDC_REF, "V");//unfiltered/plain
	PIL_OVERRIDE_PROBE(int16_t, Vmid_p, NPCSOL_Q_IV, NPCSOL_VDC_REF, "V");//unfiltered/plain
	PIL_OVERRIDE_PROBE(int16_t, Vdc, NPCSOL_Q_IV, NPCSOL_VDC_REF, "V");
	PIL_OVERRIDE_PROBE(int16_t, Vmid, NPCSOL_Q_IV, NPCSOL_VDC_REF, "V");
	PIL_OVERRIDE_PROBE(int16_t, Vguv, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");
	PIL_OVERRIDE_PROBE(int16_t, Vgvw, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");
	PIL_OVERRIDE_PROBE(int16_t, Vgwu, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");
};

typedef enum
{
	NPCSOL_SYSTEM_FSM_STATE_POWERUP,
	NPCSOL_SYSTEM_FSM_STATE_CALIBRATE,
	NPCSOL_SYSTEM_FSM_STATE_PRECHARGE_DC,
	NPCSOL_SYSTEM_FSM_STATE_SYNCHRONIZE_AC,
	NPCSOL_SYSTEM_FSM_STATE_DISABLED,
	NPCSOL_SYSTEM_FSM_STATE_READY,
	NPCSOL_SYSTEM_FSM_STATE_ENABLING,
	NPCSOL_SYSTEM_FSM_STATE_ENABLED,
	NPCSOL_SYSTEM_FSM_STATE_FAULT,
	NPCSOL_SYSTEM_FSM_STATE_CRITICAL_FAULT,
	NPCSOL_SYSTEM_FSM_STATE_FAULT_ACKN
} CTRL_SystemFsmState_t;

typedef enum
{
	SYSTEM_FSM_FAULT_NONE,
	SYSTEM_FSM_FAULT_PLL_UNLOCKED,
	SYSTEM_FSM_FAULT_GRID_OUT_OF_RANGE,
	SYSTEM_FSM_FAULT_VDC_OUT_OF_RANGE,
	SYSTEM_FSM_FAULT_MID_OUT_OF_RANGE,
	SYSTEM_FSM_FAULT_PS_OFF
} CTRL_SystemFault_t;

struct CTRL_PU_VARS {
	int16_t wrefTsScaling;
	int16_t frefTsScaling;
};

typedef struct CTRL_SYS_FSM_VARS {
	PIL_READ_PROBE(int16_t, state, 0, 1.0, "");
	PIL_READ_PROBE(uint16_t, fault, 0, 1.0, "");
} CTRL_SystemFsm_t;

typedef struct CTRL_TASK1_VARS {
	PIL_READ_PROBE(uint16_t, stepCtr, 0, 1.0, "");
	PIL_READ_PROBE(uint32_t, thisPwmUpdatedTime, 0, 1.0, "");
	PIL_READ_PROBE(uint16_t, Fs, 0, 1, "");
	PIL_OVERRIDE_PROBE(int16_t, Vsmax, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");
	PIL_READ_PROBE(int16_t, IdSet, NPCSOL_Q_IV, NPCSOL_I_REF, "A");
	PIL_READ_PROBE(int16_t, IqSet, NPCSOL_Q_IV, NPCSOL_I_REF, "A");
	PIL_OVERRIDE_PROBE(int16_t, we, PLX_PIDQ_Q_WE, NPCSOL_W_REF, "");
	PIL_READ_PROBE(int16_t, weFF, PLX_PIDQ_Q_WE, NPCSOL_W_REF, "");
	PIL_OVERRIDE_PROBE(int16_t, IdErr, NPCSOL_Q_IV, NPCSOL_I_REF, "A");
	PIL_OVERRIDE_PROBE(int16_t, IqErr, NPCSOL_Q_IV, NPCSOL_I_REF, "A");
	PIL_OVERRIDE_PROBE(int16_t, Ia, NPCSOL_Q_IV, NPCSOL_I_REF, "A");
	PIL_OVERRIDE_PROBE(int16_t, Ib, NPCSOL_Q_IV, NPCSOL_I_REF, "A");
	PIL_OVERRIDE_PROBE(int16_t, Id, NPCSOL_Q_IV, NPCSOL_I_REF, "A");
	PIL_OVERRIDE_PROBE(int16_t, Iq, NPCSOL_Q_IV, NPCSOL_I_REF, "A");
	PIL_OVERRIDE_PROBE(int16_t, Vd, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");
	PIL_OVERRIDE_PROBE(int16_t, Vq, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");
	PIL_READ_PROBE(int16_t, VdFF, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");
	PIL_READ_PROBE(int16_t, VqFF, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");
	PIL_OVERRIDE_PROBE(int16_t, VmidErr, NPCSOL_Q_IV, NPCSOL_VDC_REF, "V");
	PIL_READ_PROBE(int16_t, VdSat, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");
	PIL_READ_PROBE(int16_t, VqSat, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");
	PIL_OVERRIDE_PROBE(int16_t, Va, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");
	PIL_OVERRIDE_PROBE(int16_t, Vb, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");

	PIL_OVERRIDE_PROBE(uint16_t, redVector1Split, 15, 1, "");
	PIL_OVERRIDE_PROBE(uint16_t, redVector2Split, 15, 1, "");
	PIL_OVERRIDE_PROBE(int16_t, redVector2Adj, 15, 1, "");

	PIL_READ_PROBE(uint16_t, Cmp0T,  0, 1, "");
	PIL_READ_PROBE(uint16_t, Cmp1T,  0, 1, "");
	PIL_READ_PROBE(uint16_t, Cmp2T,  0, 1, "");

	PIL_OVERRIDE_PROBE(uint16_t, Cmp0,  0, 1, "");
	PIL_OVERRIDE_PROBE(uint16_t, Cmp1,  0, 1, "");
	PIL_OVERRIDE_PROBE(uint16_t, Cmp2,  0, 1, "");
	PIL_OVERRIDE_PROBE(uint16_t, Cmp3,  0, 1, "");
	PIL_OVERRIDE_PROBE(uint16_t, Cmp4,  0, 1, "");
	PIL_OVERRIDE_PROBE(uint16_t, Cmp5,  0, 1, "");

	PIL_READ_PROBE(uint16_t, switchingEnabled, 0, 1, "");

	PIL_OVERRIDE_PROBE(int16_t, svpwmLimitingGain, 14, 1, "");
	PIL_OVERRIDE_PROBE(uint16_t, fluxPosition, 16, 1, "rev");
	PIL_READ_PROBE(int16_t, fluxPosSin, 15, 1, "");
	PIL_READ_PROBE(int16_t, fluxPosCos, 15, 1, "");

	PIL_READ_PROBE(uint16_t, subHexagon, 0, 1, "");
	PIL_READ_PROBE(uint16_t, zone, 0, 1, "");
	PIL_READ_PROBE(int16_t, VaT, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");
	PIL_READ_PROBE(int16_t, VbT, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");

	PIL_OVERRIDE_PROBE(int16_t, VgridA, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");
	PIL_OVERRIDE_PROBE(int16_t, VgridB, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");
	PIL_OVERRIDE_PROBE(int16_t, Vgrid, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");

	PIL_READ_PROBE(uint16_t, gridVoltageAngle, 16, 1, "rev");

	PIL_READ_PROBE(uint16_t, gridPllAngle, 16, 1, "rev");
	PIL_READ_PROBE(int16_t, gridPllOmega, 10, NPCSOL_W_REF, "rad/s");
	PIL_OVERRIDE_PROBE(uint16_t, gridPllIsLocked, 0, 1, "");

	PIL_READ_PROBE(uint32_t, IuAccu, 0, 1, "");
	PIL_READ_PROBE(uint32_t, IvAccu, 0, 1, "");
	PIL_READ_PROBE(uint32_t, IwAccu, 0, 1, "");

	PIL_READ_PROBE(uint16_t, AdcAccuCtr, 0, 1, "");

	PIL_READ_PROBE(uint16_t, IuOffset, 0, 1, "");
	PIL_READ_PROBE(uint16_t, IvOffset, 0, 1, "");
	PIL_READ_PROBE(uint16_t, IwOffset, 0, 1, "");
	PIL_READ_PROBE(uint16_t, VguvOffset, 0, 1, "");
	PIL_READ_PROBE(uint16_t, VgvwOffset, 0, 1, "");
	PIL_READ_PROBE(uint16_t, VgwuOffset, 0, 1, "");

	PIL_READ_PROBE(uint16_t, calibrateOffsetReq, 0, 1, "");

	PIL_READ_PROBE(uint16_t, closedLoopEnabled, 0, 1, "");
	PIL_READ_PROBE(uint16_t, pllEnabled, 0, 1, "");

	int16_t iqSetLast;
	PIL_READ_PROBE(uint16_t, uncalibrated, 0, 1, "");
} CTRL_Task1Vars_t;

typedef struct CTRL_TASK2_VARS {
	PIL_READ_PROBE(uint16_t, stepCtr, 0, 1.0, "");
	PIL_READ_PROBE(uint16_t, active, 0, 1.0, "");
	PIL_OVERRIDE_PROBE(uint16_t, enable, 0, 1.0, "");
	PIL_OVERRIDE_PROBE(int16_t, IdDesired, NPCSOL_Q_IV, NPCSOL_I_REF, "A");
	PIL_OVERRIDE_PROBE(int16_t, IqDesired, NPCSOL_Q_IV, NPCSOL_I_REF, "A");
	PIL_OVERRIDE_PROBE(int16_t, IdSet, NPCSOL_Q_IV, NPCSOL_I_REF, "A");
	PIL_OVERRIDE_PROBE(int16_t, IqSet, NPCSOL_Q_IV, NPCSOL_I_REF, "A");
	PIL_OVERRIDE_PROBE(int16_t, VdFF, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");
	PIL_OVERRIDE_PROBE(int16_t, VqFF, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V");
	PIL_OVERRIDE_PROBE(int16_t, weFF, PLX_PIDQ_Q_WE, NPCSOL_W_REF, "");
	PIL_OVERRIDE_PROBE(int16_t, thetaSms, 16, 1, "");
	PIL_OVERRIDE_PROBE(uint16_t, enableClosedLoop, 0, 1, "");
	PIL_OVERRIDE_PROBE(uint16_t, enablePll, 0, 1, "");

	CTRL_SystemFsm_t sysFsm;

} CTRL_Task2Vars_t;

typedef struct CTRL_TASKB_VARS {
	PIL_READ_PROBE(uint16_t, backgroundTaskNumber, 0, 1.0, "");
} CTRL_TaskBVars_t;

typedef struct CTRL_OBJ
{
	PLX_SVPWM3L_Obj_t Svpwm3Obj;
	PLX_SVPWM3L_Handle_t Svpwm3Handle;

	PLX_PIDQ_Obj_t PiDqObj;
	PLX_PIDQ_Handle_t PiDqHandle;

	PLX_PLL_Obj_t PllObj;
	PLX_PLL_Handle_t PllHandle;

	PLX_ANGLE_Obj_t fluxAngleObj;
	PLX_ANGLE_Handle_t fluxAngleHandle;

	struct CTRL_PU_VARS PuVars;
	struct CTRL_ANALOG_INPUTS Ain;

	PLX_FILT_Obj_t vdcFilterObj;
	PLX_FILT_Handle_t vdcFilterHandle;
	PLX_FILT_Obj_t vmidFilterObj;
	PLX_FILT_Handle_t vmidFilterHandle;

	CTRL_Task1Vars_t tsk1;
	CTRL_Task2Vars_t tsk2;
	CTRL_TaskBVars_t tskB;
} CTRL_Obj_t;

typedef CTRL_Obj_t *CTRL_Handle_t;

void CTRL_sinit();

extern void CTRL_reset(CTRL_Handle_t aHandle);

extern void CTRL_task1(CTRL_Handle_t aHandle);
extern void CTRL_task2(CTRL_Handle_t aHandle);
extern void CTRL_background(CTRL_Handle_t aHandle);

extern CTRL_Obj_t Ctrl;
extern CTRL_Handle_t CtrlHandle;

#define DISP_CONFIGURE_TRIGGER() DISPR_setTriggerByAdcViaPwm((AIN_Unit_t)0, 2, (PWM_Unit_t)1)
#define CONTROL_RESET() CTRL_reset(CtrlHandle)
#define CONTROL_TASK1() CTRL_task1(CtrlHandle)
#define CONTROL_TASK2() CTRL_task2(CtrlHandle)
#define CONTROL_BACKGROUND() CTRL_background(CtrlHandle)

#endif /* CTRL_TEST_H_ */
