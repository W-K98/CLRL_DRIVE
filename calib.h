/*
   Copyright (c) 2014 by Plexim GmbH
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

#ifndef _CALIB_H_
#define _CALIB_H_

#include "pil.h"
#include "pu.h"
#include "hal.h"

// R=0.05, L=0.0005
#define NPCSOL_IREG_KP 1.65
#define NPCSOL_IREG_KI 0.016

struct CALIBS {
	PIL_CALIBRATION(int16_t, VARout, 0, 1.0, " ", 0, 100.0, 0.0); //added by WK
	PIL_CALIBRATION(int16_t, VARin, 0, 1.0, " ", 0, 100.0, 0.0); //added by WK

    PIL_CALIBRATION(int16_t, dcVoltageGain, NPCSOL_Q_IV, NPCSOL_VDC_REF, "V", 0, NPCSOL_VDC_REF,
            (1000.0 / 3.0 * HAL_VOLTS_PER_ADC_BIT * (double)(1 << NPCSOL_Q_IV)));
    PIL_CALIBRATION(int16_t, acVoltageGain, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V", 0, NPCSOL_VDC_REF,
            (2.0 * 1000.0 / 3.0 * HAL_VOLTS_PER_ADC_BIT * (double)(1 << NPCSOL_Q_IV)));
    PIL_CALIBRATION(int16_t, currentGain, NPCSOL_Q_IV, NPCSOL_I_REF, "A", 0, NPCSOL_I_REF,
            (2.0 * 375.0  / 3.0 * HAL_VOLTS_PER_ADC_BIT * (double)(1 << NPCSOL_Q_IV)));

    PIL_CALIBRATION(int16_t, VdcOff, NPCSOL_Q_IV, NPCSOL_VDC_REF, "V", 0, NPCSOL_VDC_REF,
            0.5 * (double)NPCSOL_VDC_REF);
    PIL_CALIBRATION(int16_t, VdcOn, NPCSOL_Q_IV, NPCSOL_VDC_REF, "V", 0, NPCSOL_VDC_REF,
            0.6 * (double)NPCSOL_VDC_REF);
    PIL_CALIBRATION(int16_t, VGridMin, NPCSOL_Q_IV, NPCSOL_VNPCSOL_P_REF, "V", 0, NPCSOL_VNPCSOL_P_REF,
            100.0 * 1.41);

    PIL_CALIBRATION(int16_t, KpD, PLX_PIDQ_Q_KP, NPCSOL_R_REF, "Ohm", 0, NPCSOL_R_REF,
            NPCSOL_IREG_KP);
    PIL_CALIBRATION(int16_t, KiD, PLX_PIDQ_Q_KI, NPCSOL_R_REF, "Ohm", 0, NPCSOL_R_REF,
            NPCSOL_IREG_KI);
    PIL_CALIBRATION(int16_t, KpQ, PLX_PIDQ_Q_KP, NPCSOL_R_REF, "Ohm", 0, NPCSOL_R_REF,
            NPCSOL_IREG_KP);
    PIL_CALIBRATION(int16_t, KiQ, PLX_PIDQ_Q_KI, NPCSOL_R_REF, "Ohm", 0, NPCSOL_R_REF,
            NPCSOL_IREG_KI);

    PIL_CALIBRATION(int16_t, KpMid, 0, NPCSOL_INV_NPCSOL_VDC_REF, "1/V", 0, NPCSOL_INV_NPCSOL_VDC_REF*10000.0,
            0.1);
    PIL_CALIBRATION(int16_t, MidErrSat, NPCSOL_Q_IV, NPCSOL_VDC_REF, "V", 0, NPCSOL_VDC_REF,
            (0.5 / 0.1));
    PIL_CALIBRATION(int16_t, MidErrMax, NPCSOL_Q_IV, NPCSOL_VDC_REF, "V", 0, NPCSOL_VDC_REF,
            50.0);

    PIL_CALIBRATION(int16_t, KpPll, PLX_PI_Q_KP, NPCSOL_F_REF, "Hz", 0, NPCSOL_F_REF*10,
            150.0);
    PIL_CALIBRATION(int16_t, KiPll, PLX_PI_Q_KI, NPCSOL_F_REF, "Hz", 0, NPCSOL_F_REF,
            1.5);
    PIL_CALIBRATION(int16_t, omegaMaxPll, NPCSOL_Q_WF, NPCSOL_F_REF, "Hz", 0, NPCSOL_F_REF*10,
            100.0);
    PIL_CALIBRATION(int16_t, omega0Pll, NPCSOL_Q_WF, NPCSOL_F_REF, "Hz", 0, NPCSOL_F_REF,
            60.0);
    PIL_CALIBRATION(uint16_t, lockAnglePll, 16, 1, "rev", 0, 1.0,
            0.02);
    PIL_CALIBRATION(uint16_t, lockTicksPll, 0, 1, "ticks", 0, 65535,
            100);
    PIL_CALIBRATION(uint16_t, unlockAnglePll, 16, 1, "rev", 0, 1.0,
            0.03);
    PIL_CALIBRATION(uint16_t, unlockTicksPll, 0, 1, "ticks", 0, 65535,
            10);

    PIL_CALIBRATION(int16_t, KpSms,  NPCSOL_Q_WF, NPCSOL_INV_NPCSOL_F_REF, "s", -NPCSOL_INV_NPCSOL_F_REF*10, NPCSOL_INV_NPCSOL_F_REF*10,
            (-0.1 / (2.0 * 3.14)));
    PIL_CALIBRATION(int16_t, SmsErrSat, NPCSOL_Q_WF, NPCSOL_F_REF, "Hz", 0, NPCSOL_F_REF,
            (0.5 / (0.1 / (2.0 * 3.14))));

    PIL_CALIBRATION(uint16_t, IuOffset, 12, 1, "", 0, 1, 0.5);
    PIL_CALIBRATION(uint16_t, IvOffset, 12, 1, "", 0, 1, 0.5);
    PIL_CALIBRATION(uint16_t, IwOffset, 12, 1, "", 0, 1, 0.5);
    PIL_CALIBRATION(uint16_t, VguvOffset, 12, 1, "", 0, 1, 0.5);
    PIL_CALIBRATION(uint16_t, VgvwOffset, 12, 1, "", 0, 1, 0.5);
    PIL_CALIBRATION(uint16_t, VgwuOffset, 12, 1, "", 0, 1, 0.5);

    PIL_CALIBRATION(uint16_t, KdcFilter, 14, 1, "", 0, 1, 0.178);
    PIL_CALIBRATION(uint16_t, KmidFilter, 14, 1, "", 0, 1, 0.178);

    PIL_CALIBRATION(float, s_ref, 5, 10.0, "", -1000, 1000, 0);
    PIL_CALIBRATION(float, s, 5, 10.0, "", -1000, 1000, 0);

    PIL_CALIBRATION(float, diff1, 5, 10.0, "", -1000, 1000, 0);
    PIL_CALIBRATION(float, diff1_1, 5, 10.0, "", -1000, 1000, 0);
    PIL_CALIBRATION(float, reg_s, 5, 10.0, "", -1000, 1000, 0);
    PIL_CALIBRATION(float, reg_s_1, 5, 10.0, "", -1000, 1000, 0);
    PIL_CALIBRATION(float, c, 5, 10.0, "", -1000, 1000, 0);

    PIL_CALIBRATION(float, diff2, 5, 10.0, "", -1000, 1000, 0);
    PIL_CALIBRATION(float, diff2_1, 5, 10.0, "", -1000, 1000, 0);
    PIL_CALIBRATION(float, out, 5, 10.0, "", -1000, 1000, 0);
    PIL_CALIBRATION(float, out_1, 5, 10.0, "", -1000, 1000, 0);


#ifndef RUN_WITH_RTBOX
    PIL_CALIBRATION(int16_t, IqDesiredFixed, NPCSOL_Q_IV, NPCSOL_I_REF, "A", NPCSOL_I_REF,NPCSOL_I_REF, 0.0);
#else
    PIL_CALIBRATION(int16_t, IqDesiredFixed, NPCSOL_Q_IV, NPCSOL_I_REF, "A", NPCSOL_I_REF,NPCSOL_I_REF, 100.0);
#endif
};

extern struct CALIBS Calibs;

void InitCalib(void);

#endif
