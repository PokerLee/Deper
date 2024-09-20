#include "hall_bldc.h"
#include "motor_common.h"
#include "pi.h"
#include "hal.h"

float test_spd = 0.1f;
#if HALL_BLDC

void HALLBLDC_Init(MOTOR_Handle handle, MotorNum_e motorNum)
{
	MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
	HAL_MTR_Obj *hal = (HAL_MTR_Obj *)obj->halMtrHandle;
	HALL_BLDC_Obj *objHallBldc = (HALL_BLDC_Obj *)(obj->hallBldcHandle);

	if(motorNum == MTR_1)
	{
		// Hall Gpio initialize
		objHallBldc->HallGpio_A = MTR1_HALLBLDC_A_GPIO;
		objHallBldc->HallGpio_B = MTR1_HALLBLDC_B_GPIO;
		objHallBldc->HallGpio_C = MTR1_HALLBLDC_C_GPIO;

		// Hall sector initialize
		objHallBldc->Hall_PwmIndex[0] = 4;
		objHallBldc->Hall_PwmIndex[1] = 1;
		objHallBldc->Hall_PwmIndex[2] = 5;
		objHallBldc->Hall_PwmIndex[3] = 6;
		objHallBldc->Hall_PwmIndex[4] = 3;
		objHallBldc->Hall_PwmIndex[5] = 2;
		objHallBldc->Hall_PwmIndex[6] = 4;
		objHallBldc->Hall_PwmIndex[7] = 1;

		objHallBldc->Hall_Flag_EnableStartup = 1;
		objHallBldc->Hall_Flag_EnableBldc = 1;
		objHallBldc->Hall_Flag_CurrentCtrl = 0;	// default for speed control

		objHallBldc->Hall_speed_scale = (float)160000000/6;

		objHallBldc->Hall_speed_FastToBldc_low_pu = 20.0;
		objHallBldc->Hall_speed_BldcToFast_high_pu = 25.0;

		// Hall to Fast coefficient
		objHallBldc->Hall2Fast_Spd_Coef = 1.0f;
		objHallBldc->Hall2Fast_Iq_coef = 0.5f;
		objHallBldc->Hall2Fast_Ui_coef = 0.5f;

		objHallBldc->Hall_PwmCntMax = 5000;

	}
	else // motorNum == MTR_2
	{
		// Hall Gpio initialize
		objHallBldc->HallGpio_A = MTR2_HALLBLDC_A_GPIO;
		objHallBldc->HallGpio_B = MTR2_HALLBLDC_B_GPIO;
		objHallBldc->HallGpio_C = MTR2_HALLBLDC_C_GPIO;

		// Hall sector initialize
		objHallBldc->Hall_PwmIndex[0] = 4;
		objHallBldc->Hall_PwmIndex[1] = 1;
		objHallBldc->Hall_PwmIndex[2] = 5;
		objHallBldc->Hall_PwmIndex[3] = 6;
		objHallBldc->Hall_PwmIndex[4] = 3;
		objHallBldc->Hall_PwmIndex[5] = 2;
		objHallBldc->Hall_PwmIndex[6] = 4;
		objHallBldc->Hall_PwmIndex[7] = 1;

		objHallBldc->Hall_Flag_EnableStartup = 1;
		objHallBldc->Hall_Flag_EnableBldc = 1;
		objHallBldc->Hall_Flag_CurrentCtrl = 0;	// default for speed control

		objHallBldc->Hall_speed_scale = (float)160000000/6;

		objHallBldc->Hall_speed_FastToBldc_low_pu = 20.0;
		objHallBldc->Hall_speed_BldcToFast_high_pu = 25.0;

		// Hall to Fast coefficient
		objHallBldc->Hall2Fast_Spd_Coef = 1.0f;
		objHallBldc->Hall2Fast_Iq_coef = 0.5f;
		objHallBldc->Hall2Fast_Ui_coef = 0.5f;

		objHallBldc->Hall_PwmCntMax = 5000;

	}
}

void HALLBLDC_Ctrl_PwmSet(MOTOR_Handle handle, uint16_t PwmState, float PwmDuty)
{
	MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
	HAL_MTR_Obj *hal = (HAL_MTR_Obj *)obj->halMtrHandle;
	HALL_BLDC_Obj *objHallBldc = (HALL_BLDC_Obj *)(obj->hallBldcHandle);

  switch(PwmState)
  {
	case 5:		// U+/V-
	{
		PWM_clearOneShotTrip(hal->pwmHandle[0]);
		PWM_clearOneShotTrip(hal->pwmHandle[1]);
		PWM_setOneShotTrip(hal->pwmHandle[2]);

		obj->pwmData.Vabc_pu.value[0] = PwmDuty;
		obj->pwmData.Vabc_pu.value[1] = -PwmDuty;
		obj->pwmData.Vabc_pu.value[2] = 0.0;

		break;
	}
	case 1:		// U+/W-
	{
		PWM_clearOneShotTrip(hal->pwmHandle[0]);
		PWM_setOneShotTrip(hal->pwmHandle[1]);
		PWM_clearOneShotTrip(hal->pwmHandle[2]);

		obj->pwmData.Vabc_pu.value[0] = PwmDuty;
		obj->pwmData.Vabc_pu.value[1] = 0.0f;
		obj->pwmData.Vabc_pu.value[2] = -PwmDuty;
		break;
	}
	case 3:		//V+/W-
	{
		PWM_setOneShotTrip(hal->pwmHandle[0]);
		PWM_clearOneShotTrip(hal->pwmHandle[1]);
		PWM_clearOneShotTrip(hal->pwmHandle[2]);

		obj->pwmData.Vabc_pu.value[0] = 0.0f;
		obj->pwmData.Vabc_pu.value[1] = PwmDuty;
		obj->pwmData.Vabc_pu.value[2] = -PwmDuty;
		break;
	}
	case 2:		//U-/V+
	{
		PWM_clearOneShotTrip(hal->pwmHandle[0]);
		PWM_clearOneShotTrip(hal->pwmHandle[1]);
		PWM_setOneShotTrip(hal->pwmHandle[2]);

		obj->pwmData.Vabc_pu.value[0] = -PwmDuty;
		obj->pwmData.Vabc_pu.value[1] = PwmDuty;
		obj->pwmData.Vabc_pu.value[2] = 0.0f;
		break;
	}
	case 6:		//U-/W+
	{
		PWM_clearOneShotTrip(hal->pwmHandle[0]);
		PWM_setOneShotTrip(hal->pwmHandle[1]);
		PWM_clearOneShotTrip(hal->pwmHandle[2]);

		obj->pwmData.Vabc_pu.value[0] = -PwmDuty;
		obj->pwmData.Vabc_pu.value[1] = 0.0f;
		obj->pwmData.Vabc_pu.value[2] = PwmDuty;
		break;
	}
	case 4:		//V-/W+
	{
		PWM_setOneShotTrip(hal->pwmHandle[0]);
		PWM_clearOneShotTrip(hal->pwmHandle[1]);
		PWM_clearOneShotTrip(hal->pwmHandle[2]);

		obj->pwmData.Vabc_pu.value[0] = 0.0f;
		obj->pwmData.Vabc_pu.value[1] = -PwmDuty;
		obj->pwmData.Vabc_pu.value[2] = PwmDuty;
		break;
	}
	default:	// N/A
		break;
  }
}

float test_IsRef_A = 0.0f;
float test_spdKp = 0.054850627f;
float test_spdKi = 0.025132742f;
float test_IqKp = 11.858881f;
float test_IqKi = 0.011731809f;
float test_IdKp = 0.0f;
float test_IdKi = 0.0f;
void HALLBLDC_Ctrl_Run(MOTOR_Handle handle)
{
	test_IdKp = test_IqKp;
	test_IdKi = test_IqKi;

	MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
	HALL_BLDC_Obj *objHallBldc = (HALL_BLDC_Obj *)(obj->hallBldcHandle);

	if(objHallBldc->Hall_Flag_EnableStartup == true)
	{
		objHallBldc->Hall_PwmState = objHallBldc->Hall_PwmIndex[objHallBldc->Hall_State];

		if(fabsf(obj->speed_Hz) < objHallBldc->Hall_speed_FastToBldc_low_pu)	// FAST to Hall
		{
		  if(objHallBldc->Hall_Flag_EnableBldc == false)
		  {
			  if(objHallBldc->Hall_Bldc_Cnt > 20)
			  {
				  objHallBldc->Hall_Flag_EnableBldc = true;
				  objHallBldc->Hall_Bldc_Cnt = 0;

				if(objHallBldc->Hall_Flag_CurrentCtrl == true)		// Torque Control Mode
				{
					// The following instructions load the parameters for the speed PI
					// controller.
					PI_setGains(obj->piHandle_spd,0.1,0.005);

					// Set the initial condition value for the integrator output to 0
					PI_setUi(obj->piHandle_Iq_bldc, obj->piHandle_Iq->Ui*objHallBldc->Fast2Hall_Ui_coef);
				}
				else		// Speed Control Mode
				{
					// The following instructions load the parameters for the speed PI
					// controller.
					PI_setGains(obj->piHandle_spd,0.054850627,0.025132742);

					objHallBldc->Hall_PwmDuty = obj->piHandle_Iq->Ui;

					// Set the initial condition value for the integrator output to 0
					PI_setUi(obj->piHandle_spd, obj->piHandle_Iq->Ui*objHallBldc->Fast2Hall_Spd_coef);
				}
			  }
			  else
				  objHallBldc->Hall_Bldc_Cnt++;
		  }
		}
		else if(fabsf(obj->speed_Hz) > objHallBldc->Hall_speed_BldcToFast_high_pu)			// Hall to FAST
		{
		  if(objHallBldc->Hall_Flag_EnableBldc == true)
		  {
			  if(objHallBldc->Hall_Fast_Cnt > 20)
			  {
				  objHallBldc->Hall_Flag_EnableBldc = false;
				  objHallBldc->Hall_Fast_Cnt = 0;

				  if(objHallBldc->Hall_Flag_CurrentCtrl == true)		// Torque Control
				  {
					  // The following instructions load the parameters for the speed PI
					  // controller.
					  PI_setGains(obj->piHandle_spd,2.0,0.02);

					  // Set the initial condition value for the integrator output to 0, Id
					  PI_setUi(obj->piHandle_Id, 0.0);

					  // Set the initial condition value for the integrator output to 0, Iq
					  PI_setUi(obj->piHandle_Iq, obj->piHandle_Iq_bldc->Ui * 0.25);

				  }
				  else				// speed control
				  {
					  // The following instructions load the parameters for the speed PI
					  // controller.
//					  PI_setGains(obj->piHandle_spd,0.054850627,0.025132742);

					  PI_setGains(obj->piHandle_spd,test_spdKp,test_spdKi);

					  // Set the initial condition value for the integrator output to 0, speed
					  PI_setUi(obj->piHandle_spd, objHallBldc->Hall_PwmDuty*objHallBldc->Hall2Fast_Spd_Coef);

					  // Set the initial condition value for the integrator output to 0, Iq
					  PI_setUi(obj->piHandle_Iq, objHallBldc->Hall_PwmDuty*objHallBldc->Hall2Fast_Iq_coef);

					  // Set the initial condition value for the integrator output to 0, Id
					  PI_setUi(obj->piHandle_Id, objHallBldc->Hall_PwmDuty*0.0f);
				  }

				  EPWM_setADCTriggerSource(obj->halMtrHandle->pwmHandle[0],
				                               EPWM_SOC_A, EPWM_SOC_TBCTR_PERIOD);
				  HAL_enablePWM(obj->halMtrHandle);
			  }
			  else
				  objHallBldc->Hall_Fast_Cnt++;
		  }
		}

		if(objHallBldc->Hall_Flag_EnableBldc)
		{
			obj->angleFOC_rad = obj->angleEST_rad;
			obj->speed_Hz = objHallBldc->Hall_speed_fdb_Hz;

			if(objHallBldc->Hall_Flag_CurrentCtrl == true)		// Torque Control Mode
			{
				objHallBldc->Hall_BLDC_Is_fdb_pu = obj->adcData.I_A.value[objHallBldc->Hall_BLDC_Flag_Is_fdb];
				objHallBldc->Hall_BLDC_Is_ref_pu = obj->IsRef_A;

				// BLDC current loop
				PI_run(obj->piHandle_Iq_bldc,objHallBldc->Hall_BLDC_Is_ref_pu,objHallBldc->Hall_BLDC_Is_fdb_pu,&(objHallBldc->Hall_PwmDuty));

				HALLBLDC_Ctrl_PwmSet(obj, objHallBldc->Hall_PwmState, objHallBldc->Hall_PwmDuty);
			}
			else									// Speed Control Mode
			{
				objHallBldc->Hall_PwmDuty = (1 / USER_MOTOR1_MAX_CURRENT_A) * obj->IsRef_A;
//				objHallBldc->Hall_PwmDuty = test_IsRef_A;
//				HALLBLDC_Ctrl_PwmSet(obj, objHallBldc->Hall_PwmState, objHallBldc->Hall_PwmDuty);
				HALLBLDC_Ctrl_PwmSet(obj, objHallBldc->Hall_GpioData, objHallBldc->Hall_PwmDuty);
			}
		}
		else
		{
			obj->angleFOC_rad = obj->angleEST_rad;
			obj->speed_Hz = obj->speedEST_Hz;
		}
	}
	else	//(Hall_Flag_EnableStartup == false)
	{
		obj->angleFOC_rad = obj->angleEST_rad;
		obj->speed_Hz = obj->speedEST_Hz;
	}

	return;
}

void HALLBLDC_State_Check(MOTOR_Handle handle, MotorNum_e motorNum)
{
	MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
	HALL_BLDC_Obj *objHallBldc = (HALL_BLDC_Obj *)(obj->hallBldcHandle);

	// Hall_A, Hall_B, Hall_C
	objHallBldc->Hall_GpioData  = GPIO_readPin(objHallBldc->HallGpio_A) << 2;
	objHallBldc->Hall_GpioData += GPIO_readPin(objHallBldc->HallGpio_B) << 1;
	objHallBldc->Hall_GpioData += GPIO_readPin(objHallBldc->HallGpio_C);
	objHallBldc->Hall_State = objHallBldc->Hall_GpioData;

	uint8_t timerNum;
	if(motorNum == MTR_1)
	{
		timerNum = 2;
	}
	else if(motorNum == MTR_2)
	{
		timerNum = 1;
	}

	if(objHallBldc->Hall_State != objHallBldc->Hall_PrevState)
	{
		objHallBldc->Hall_timer_now = HAL_readTimerCnt(halHandle,timerNum);
		objHallBldc->Hall_time_delta_now = objHallBldc->Hall_timer_prev - objHallBldc->Hall_timer_now;
		objHallBldc->Hall_timer_prev = objHallBldc->Hall_timer_now;

		objHallBldc->Hall_time_delta = (objHallBldc->Hall_time_delta_now + objHallBldc->Hall_time_delta_prev)>>1;
		objHallBldc->Hall_time_delta_prev = objHallBldc->Hall_time_delta_now;

		objHallBldc->Hall_speed_fdb_Hz = (float)objHallBldc->Hall_speed_scale / (float)objHallBldc->Hall_time_delta;

	   	//direction check
		objHallBldc->Hall_State_delta = objHallBldc->Hall_PwmIndex[objHallBldc->Hall_State] - \
				objHallBldc->Hall_PwmIndex[objHallBldc->Hall_PrevState];

		if((objHallBldc->Hall_State_delta == -1) || \
				(objHallBldc->Hall_State_delta == 5))
		{
			objHallBldc->Hall_dir = 2; 		// positive direction
		}
	   	else if((objHallBldc->Hall_State_delta == 1) || \
	   			(objHallBldc->Hall_State_delta == -5))
		{
	   		objHallBldc->Hall_dir = 1; 		// negative direction
	   		objHallBldc->Hall_speed_fdb_Hz = -objHallBldc->Hall_speed_fdb_Hz;
		}
		else
		{
			objHallBldc->Hall_dir = 0; 		// direction change
		}

		//check if dirction is chagned.
		//if direction is changed, speed feedback is reset.
		if(objHallBldc->Hall_dir != objHallBldc->Hall_dir_prev)
		{
			objHallBldc->Hall_speed_fdb_Hz = 0.0f;
			objHallBldc->Hall_dir_change = 1;
		}

		objHallBldc->Hall_LastState = objHallBldc->Hall_PrevState;
		objHallBldc->Hall_PrevState = objHallBldc->Hall_State;
		objHallBldc->Hall_dir_prev = objHallBldc->Hall_dir;

		objHallBldc->Hall_Flag_State_Change = true;
		objHallBldc->Hall_PwmCnt = 0;
	}
	else
	{
		objHallBldc->Hall_PwmCnt++;
		if(objHallBldc->Hall_PwmCnt > objHallBldc->Hall_PwmCntMax)
		{
			objHallBldc->Hall_speed_fdb_Hz = 0.0f;
			objHallBldc->Hall_PwmCnt = 0;
		}
	}

	return;
}

void HALLBLDC_Ctrl_Stop(MOTOR_Handle handle)
{
	MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
	HALL_BLDC_Obj *objHallBldc = (HALL_BLDC_Obj *)(obj->hallBldcHandle);

	objHallBldc->Hall_Flag_EnableBldc = true;
	objHallBldc->Hall_Flag_State_Change = false;
	objHallBldc->Hall_speed_fdb_Hz = 0.0f;

    // Set the initial condition value for the integrator output to 0
    PI_setUi(obj->piHandle_spd,0.0f);
    PI_setUi(obj->piHandle_Id,0.0f);
    PI_setUi(obj->piHandle_Iq,0.0f);
    PI_setUi(obj->piHandle_Iq_bldc,0.0f);
}
#endif
//
// End of File
//
