################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../device/device.c 

OBJS += \
./device/device.o 

C_DEPS += \
./device/device.d 


# Each subdirectory must supply rules for building sources it contributes
device/%.o: ../device/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU RISC-V Cross C Compiler'
	riscv32-haawking-elf-gcc -march=rv32imfc_xhaawkaa1p0_xhaawkab1p0_xhaawkac1p0 -D__RUNNING_IN_FLASH_ -T "D:/workspace_haawking/haawking-workspace-V2.2.11Pre/FASTLib_HX280025CEDC_DUAL_MOTOR_Deper_V1.1/haawking-drivers/ldscripts/HXS320F280025C_EDC_link_FLASH.ld" -mabi=ilp32f -mcmodel=medlow -mno-save-restore -menable-experimental-extensions --target=riscv32-unknown-elf --sysroot="D:/Haawking-IDE_v2.2.11Pre\haawking-tools/compiler/haawking-elf-gcc/riscv32-unknown-elf" --gcc-toolchain="D:/Haawking-IDE_v2.2.11Pre\haawking-tools/compiler/haawking-elf-gcc" -Odefault -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-inline-functions -cl-single-precision-constant -Wall -Wextra -Wl,--no-warn-rwx-segments  -g3 -gdwarf-3 -Wl,--defsym,IDE_VERSION_2_2_11Pre=0 -DDSC280025C_EDC -DDEBUG -DHALL_BLDC=1 -I../device -I../haawking-drivers/haawking-dsc280025_edc-board/common -I../haawking-drivers/haawking-dsc280025_edc-board -I../haawking-drivers/haawking-dsc280025_edc-board/inc -I../src -I"D:\workspace_haawking\haawking-workspace-V2.2.11Pre\FASTLib_HX280025CEDC_DUAL_MOTOR_Deper_V1.1\src_board" -I"D:\workspace_haawking\haawking-workspace-V2.2.11Pre\FASTLib_HX280025CEDC_DUAL_MOTOR_Deper_V1.1\src_control" -I"D:\workspace_haawking\haawking-workspace-V2.2.11Pre\FASTLib_HX280025CEDC_DUAL_MOTOR_Deper_V1.1\src_est" -std=gnu99 -x c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


