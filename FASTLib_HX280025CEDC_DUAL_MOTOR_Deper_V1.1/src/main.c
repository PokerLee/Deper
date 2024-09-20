/*
 * Copyright (c) 2019-2022 Beijing Haawking Technology Co.,Ltd
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Boke Li
 * Email  : 1048402825@qq.com
 * FILE   : main.c
 *****************************************************************/
 
#include <syscalls.h>
#include "IQmathLib.h"
#include "driverlib.h"
#include "device.h"
#include "sys_main.h"

CPU_TIME_Obj     cpuTime;
CPU_TIME_Handle  cpuTimeHandle;
volatile SYSTEM_Vars_t systemVars;

// position 90
float32_t delta_angle = 0.0f;
float32_t delta_angle1 = 0.0f;
float32_t test_div = 10.0f;
uint8_t flag_openDoor = 0;
uint8_t flag_closeDoor = 1;

int main(void)
{
	// 初始化设备时钟和外设
 	Device_init();

    // 禁用引脚锁定
	Device_initGPIO();

    // 初始化中断控制器
    Interrupt_initModule();

    // 初始化中断向量表
    Interrupt_initVectorTable();

    // 配置电机驱动外设地址
	halHandle = HAL_init(&hal);

	// 配置驱动电机外设模块参数
	HAL_setParams(halHandle);

    // 设置电机1的控制参数
    motorHandle[MTR_1] = (MOTOR_Handle)(&motorVars[MTR_1]);

    // 设置电机2的控制参数
    motorHandle[MTR_2] = (MOTOR_Handle)(&motorVars[MTR_2]);

    // 设置电机1启动标志
    motorVars[MTR_1].flagEnableRunAndIdentify = false;

    // 设置电机2启动标志
    motorVars[MTR_2].flagEnableRunAndIdentify = false;

    // 设置电机1参考速度
    motorVars[MTR_1].speedRef_Hz = 20.0f;

    // 设置电机2参考速度
	motorVars[MTR_2].speedRef_Hz = 80.0f;

	// 电机1是否绕过电机参数辨识
    userParams[MTR_1].flag_bypassMotorId = true;// false;//

    // 电机2是否绕过电机参数辨识
    userParams[MTR_2].flag_bypassMotorId = true;// false;//

    // 初始化电机1结构体
    initMotorHandles(motorHandle[MTR_1], MTR_1);

    // 初始化电机2结构体
    initMotorHandles(motorHandle[MTR_2], MTR_2);

    // 初始化电机1控制参数
    initMotorCtrlParameters(motorHandle[MTR_1], MTR_1);

    // 初始化电机2控制参数
    initMotorCtrlParameters(motorHandle[MTR_2], MTR_2);

    // 初始化CPU模块（计数器）
    cpuTimeHandle = &cpuTime;
    CPU_TIME_reset(cpuTimeHandle);
    systemVars.flagEnableSystem = true;

    // 初始化中断向量表
    HAL_initIntVectorTable();

    // 启用ADC/PWM中断进行控制
    HAL_enableCtrlInts(halHandle);

    // 启用电机1零点偏移校准
    motorVars[MTR_1].flagEnableOffsetCalc = true;

    // 启用电机2零点偏移校准
    motorVars[MTR_2].flagEnableOffsetCalc = true;

    // 运行电机1零点偏移校准
    runMotorOffsetsCalculation(motorHandle[MTR_1], MTR_1);

    // 运行电机2零点偏移校准
    runMotorOffsetsCalculation(motorHandle[MTR_2], MTR_2);

    // 启用全局中断
    HAL_enableGlobalInts(halHandle);

    // 启用调试中断
    HAL_enableDebugInt(halHandle);

    systemVars.powerRelayWaitTime_ms = POWER_RELAY_WAIT_TIME_ms;

	while(systemVars.flagEnableSystem == false)
	{
	   if(HAL_getCPUTimerStatus(halHandle, HAL_CPU_TIMER0))
	   {
		   HAL_clearCPUTimerFlag(halHandle, HAL_CPU_TIMER0);

		   systemVars.timerBase_1ms++;

		   if(systemVars.timerBase_1ms > systemVars.powerRelayWaitTime_ms)
		   {
			   systemVars.flagEnableSystem = true;
			   systemVars.timerBase_1ms = 0;
		   }
	   }
	}
	while(systemVars.flagEnableSystem == true)
	{
	   // 主循环计数器
	   systemVars.mainLoopCnt++;

	   // 1ms 时间基数
	   if(HAL_getCPUTimerStatus(halHandle, HAL_CPU_TIMER0))
	   {
		   HAL_clearCPUTimerFlag(halHandle, HAL_CPU_TIMER0);

		   // led闪灯
		   systemVars.counterLED++;
		   if(systemVars.counterLED > (uint16_t)(LED_BLINK_FREQ_Hz * 1000))
		   {
			   // 反转led
			   HAL_toggleLED( HAL_GPIO_LED2C);

			   systemVars.counterLED = 0;
		   }
		   systemVars.timerBase_1ms++;

		   switch(systemVars.timerBase_1ms)
		   {
			   case 1:
				  // 电机1保护处理
				  runMotorMonitor(motorHandle[MTR_1]);

				  // 电机2保护处理
				  runMotorMonitor(motorHandle[MTR_2]);
				   break;
			   case 2:
				   // 电机1有效值计算
				   calculateRMSData(motorHandle[MTR_1]);

				   // 电机2有效值计算
				   calculateRMSData(motorHandle[MTR_2]);
				   break;
			   case 3:
				   break;
			   case 4:
				   // 电机1过流保护
				   calcMotorOverCurrentThreshold(motorHandle[MTR_1]);

				   // 电机2过流保护
				   calcMotorOverCurrentThreshold(motorHandle[MTR_2]);
				   break;
			   case 5:
				   systemVars.timerBase_1ms = 0;
				   systemVars.timerCnt_5ms++;

				   delta_angle += (( 6.283185307179586f * motorVars[0].speed_Hz)/test_div) *0.005f;
				   delta_angle1 += fabsf((( 6.283185307179586f * motorVars[1].speed_Hz)/test_div) *0.005f);
				   break;
		   }

//		   if(delta_angle>=70.0f)//slow
//		   {
//			   motorVars[0].speedRef_Hz = 0.0f;
//			   motorVars[0].accelerationStart_Hzps = 40.0f;
//			   motorVars[0].accelerationMax_Hzps = 40.0f;
//		   }
//
//		   if(delta_angle>=90.0f)//stop
//		   {
//			   delta_angle = 0.0f;
//			   motorVars[0].flagEnableRunAndIdentify = false;
//			   motorVars[0].speedRef_Hz = 80.0f;
//		   }

		   if((flag_openDoor == 0)&&(flag_closeDoor == 1))
		   {
			   motorVars[1].speedRef_Hz = 80.0f;
		   }
		   else if((flag_openDoor == 1)&&(flag_closeDoor == 0))
		   {
			   motorVars[1].speedRef_Hz = -80.0f;
		   }

		   if(delta_angle>=70.0f)//slow
		   {
//			   motorVars[1].speedRef_Hz = 0.0f;
			   motorVars[0].accelerationStart_Hzps = 40.0f;
			   motorVars[0].accelerationMax_Hzps = 40.0f;
		   }
		   else if(delta_angle>=90.0f)//stop
		   {
			   delta_angle = 0.0f;
			   motorVars[0].flagEnableRunAndIdentify = false;
			   motorVars[0].speedRef_Hz = 80.0f;

			   if((flag_openDoor == 0)&&(flag_closeDoor == 1))
			   {
				   flag_openDoor = 1;
				   flag_closeDoor = 0;
			   }
			   else if((flag_openDoor == 1)&&(flag_closeDoor == 0))
			   {
				   flag_openDoor = 0;
				   flag_closeDoor = 1;
			   }
		   }

		   if(delta_angle1>=70.0f)//slow
		   {
//			   motorVars[1].speedRef_Hz = 0.0f;
			   motorVars[1].accelerationStart_Hzps = 40.0f;
			   motorVars[1].accelerationMax_Hzps = 40.0f;
		   }
		   else if(delta_angle1>=90.0f)//stop
		   {
			   delta_angle1 = 0.0f;
			   motorVars[1].flagEnableRunAndIdentify = false;
			   motorVars[1].speedRef_Hz = 80.0f;

			   if((flag_openDoor == 0)&&(flag_closeDoor == 1))
			   {
				   flag_openDoor = 1;
				   flag_closeDoor = 0;
			   }
			   else if((flag_openDoor == 1)&&(flag_closeDoor == 0))
			   {
				   flag_openDoor = 0;
				   flag_closeDoor = 1;
			   }
		   }
	   }
	   // 电机1主函数轮询处理
	   runMotorControl(motorHandle[MTR_1]);

	   // 电机2主函数轮询处理
	   runMotorControl(motorHandle[MTR_2]);
	}

	// 停止PWM发波
	HAL_disablePWM(motorHandle[MTR_1]->halMtrHandle);
	HAL_disablePWM(motorHandle[MTR_2]->halMtrHandle);
}

//
// End of File
//

