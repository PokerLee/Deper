#ifndef _HALL_BLDC_H_
#define _HALL_BLDC_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include "hx_intrinsics.h"
#include "hw_types.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

//! \brief Defines the HALL controller object
//!
typedef struct _HALL_BLDC_Obj_
{
	// Hall GPIO initialize
	uint16_t HallGpio_A;
	uint16_t HallGpio_B;
	uint16_t HallGpio_C;

	// Hall Control Flag
	uint16_t Hall_Flag_EnableBldc;
	uint16_t Hall_Flag_EnableStartup;
	uint16_t Hall_Flag_CurrentCtrl;
	uint16_t Hall_Flag_State_Change;

	// BLDC speed
	float Hall_speed_fdb_pu;
	float Hall_speed_FastToBldc_low_pu;
	float Hall_speed_BldcToFast_high_pu;

	//BLDC current loop
	float Hall_BLDC_Is_fdb_pu;
	float Hall_BLDC_Is_ref_pu;
	float Hall_PwmDuty;

	// Coefficient for pid regulator switch from Hall to Fast
	float Hall2Fast_Spd_Coef;
	float Hall2Fast_Iq_coef;
	float Hall2Fast_Ui_coef;

	// Coefficient for pid regulator switch from Fast to Hall
	float Fast2Hall_Spd_coef;
	float Fast2Hall_Iq_coef;
	float Fast2Hall_Ui_coef;

	uint16_t Hall_PwmState;
	uint16_t Hall_GpioData;

	int16_t Hall_State;
	int16_t Hall_PrevState;
	int16_t Hall_LastState;
	int16_t Hall_State_delta;

	uint16_t Hall_dir;
	uint16_t Hall_dir_prev;
	uint16_t Hall_dir_change;

	uint16_t Hall_BLDC_Flag_Is_fdb;
	uint16_t Hall_PwmIndex[8];

	uint32_t Hall_timer_now;
	uint32_t Hall_timer_prev;
	uint32_t Hall_time_delta_now;
	uint32_t Hall_time_delta_prev;
	uint32_t Hall_time_delta;

	uint32_t Hall_speed_scale;
	float Hall_speed_fdb_Hz;
	float Hall_Speed_0p01hz_to_pu_sf;

	uint16_t Hall_PwmCnt;
	uint16_t Hall_PwmCntMax;

	uint32_t Hall_Bldc_Cnt;
	uint32_t Hall_Fast_Cnt;
} HALL_BLDC_Obj;


//! \brief Defines the HALL handle
//!
typedef struct _HALL_BLDC_Obj_ *HALL_BLDC_Handle;

#endif
