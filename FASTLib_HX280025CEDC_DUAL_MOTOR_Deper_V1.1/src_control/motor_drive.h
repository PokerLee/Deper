/*
 * motor1_drive.h
 *
 *  Created on: 2023Äê1ÔÂ31ÈÕ
 *      Author: Kangaihong
 */

#ifndef SRC_CONTROL_MOTOR_DRIVE_H_
#define SRC_CONTROL_MOTOR_DRIVE_H_

#include "userParams.h"
#include "motor_common.h"

// the globals
extern volatile MOTOR_Handle motorHandle[2];
extern volatile MOTOR_Vars_t motorVars[2];
extern MOTOR_SetVars_t motorSetVars[2];
extern USER_Params userParams[2];

extern void initMotorHandles(MOTOR_Handle handle, const MotorNum_e motorNum);
extern void initMotorCtrlParameters(MOTOR_Handle handle, const MotorNum_e motorNum);
extern void runMotorOffsetsCalculation(MOTOR_Handle handle, const MotorNum_e motorNum);
extern void runMotorControl(MOTOR_Handle handle);

#endif /* SRC_CONTROL_MOTOR_DRIVE_H_ */
