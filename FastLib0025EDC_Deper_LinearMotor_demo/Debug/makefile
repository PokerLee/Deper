################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf *.s *.map *.asm

# All of the sources participating in the build are defined here
-include sources.mk
-include src_control/hall_fast/subdir.mk
-include src_control/hall_bldc/subdir.mk
-include src_control/subdir.mk
-include src_board/subdir.mk
-include src/subdir.mk
-include haawking-drivers/haawking-dsc280025_edc-board/common/subdir.mk
-include haawking-drivers/haawking-dsc280025_edc-board/subdir.mk
-include device/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_GCCASM += \
FastLib0025EDC_Deper_LinearMotor_demo.s \

SECONDARY_LLVMASM += \
FastLib0025EDC_Deper_LinearMotor_demo.asm \

SECONDARY_FLASH += \
FastLib0025EDC_Deper_LinearMotor_demo.hex \

SECONDARY_SIZE += \
FastLib0025EDC_Deper_LinearMotor_demo.siz \


# All Target
all: FastLib0025EDC_Deper_LinearMotor_demo.elf secondary-outputs

# print-size
print-size:
	llvm-nm -n --print-size "FastLib0025EDC_Deper_LinearMotor_demo.elf">>"FastLib0025EDC_Deper_LinearMotor_demo.map"

# Tool invocations
FastLib0025EDC_Deper_LinearMotor_demo.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	riscv32-haawking-elf-gcc -march=rv32imfc_xhaawkaa1p0_xhaawkab1p0_xhaawkac1p0 -D__RUNNING_IN_FLASH_ -T "D:/workspace_haawking/haawking-workspace-V2.2.11Pre/FastLib0025EDC_Deper_LinearMotor_demo/haawking-drivers/ldscripts/HXS320F280025C_EDC_link_FLASH.ld" -mabi=ilp32f -mcmodel=medlow -mno-save-restore -menable-experimental-extensions --target=riscv32-unknown-elf --sysroot="D:/Haawking-IDE_v2.2.11Pre\haawking-tools/compiler/haawking-elf-gcc/riscv32-unknown-elf" --gcc-toolchain="D:/Haawking-IDE_v2.2.11Pre\haawking-tools/compiler/haawking-elf-gcc" -Odefault -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-inline-functions -cl-single-precision-constant -Wall -Wextra -Wl,--no-warn-rwx-segments  -g3 -gdwarf-3 -Wl,--defsym,IDE_VERSION_2_2_11Pre=0 -nostartfiles -nostdlib -Xlinker --gc-sections -L../lib -L../haawking-drivers/ldscripts -L"D:/Haawking-IDE_v2.2.11Pre/haawking-tools/segger_lib" -Wl,-Map,"FastLib0025EDC_Deper_LinearMotor_demo.map" -o "FastLib0025EDC_Deper_LinearMotor_demo.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	$(MAKE) --no-print-directory print-size

FastLib0025EDC_Deper_LinearMotor_demo.s: FastLib0025EDC_Deper_LinearMotor_demo.elf
	@riscv64-unknown-elf-objdump -D "FastLib0025EDC_Deper_LinearMotor_demo.elf" > "FastLib0025EDC_Deper_LinearMotor_demo.s"

FastLib0025EDC_Deper_LinearMotor_demo.asm: FastLib0025EDC_Deper_LinearMotor_demo.elf
	@riscv32-haawking-elf-objdump -D "FastLib0025EDC_Deper_LinearMotor_demo.elf" > "FastLib0025EDC_Deper_LinearMotor_demo.asm"

FastLib0025EDC_Deper_LinearMotor_demo.hex: FastLib0025EDC_Deper_LinearMotor_demo.elf
	@riscv32-haawking-elf-objcopy -O ihex "FastLib0025EDC_Deper_LinearMotor_demo.elf"  "FastLib0025EDC_Deper_LinearMotor_demo.hex"

FastLib0025EDC_Deper_LinearMotor_demo.siz: FastLib0025EDC_Deper_LinearMotor_demo.elf
	riscv32-haawking-elf-size --format=berkeley "FastLib0025EDC_Deper_LinearMotor_demo.elf"

# Other Targets
clean:
	-$(RM) $(SECONDARY_LLVMASM)$(CLA_DEPS)$(SECONDARY_GCCASM)$(OBJS)$(SECONDARY_FLASH)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) FastLib0025EDC_Deper_LinearMotor_demo.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_GCCASM) $(SECONDARY_LLVMASM) $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
