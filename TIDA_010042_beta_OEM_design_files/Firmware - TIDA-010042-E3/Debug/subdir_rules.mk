################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1230/ccs/tools/compiler/ti-cgt-armllvm_2.1.3.LTS/bin/tiarmclang.exe" -c -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/A0501766/Desktop/TIDA-010042-E3_SW" -I"C:/Users/A0501766/Desktop/TIDA-010042-E3_SW/Debug" -I"C:/ti/mspm0_sdk_1_00_01_03/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_1_00_01_03/source" -D__MSPM0G3507__ -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/A0501766/Desktop/TIDA-010042-E3_SW/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-2110667615: ../TIDA-010042E3.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/ccs1230/ccs/utils/sysconfig_1.16.1/sysconfig_cli.bat" -s "C:/ti/mspm0_sdk_1_00_01_03/.metadata/product.json" --script "C:/Users/A0501766/Desktop/TIDA-010042-E3_SW/TIDA-010042E3.syscfg" -o "syscfg" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/ti_msp_dl_config.c: build-2110667615 ../TIDA-010042E3.syscfg
syscfg/ti_msp_dl_config.h: build-2110667615
syscfg/Event.dot: build-2110667615
syscfg/: build-2110667615

syscfg/%.o: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1230/ccs/tools/compiler/ti-cgt-armllvm_2.1.3.LTS/bin/tiarmclang.exe" -c -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/A0501766/Desktop/TIDA-010042-E3_SW" -I"C:/Users/A0501766/Desktop/TIDA-010042-E3_SW/Debug" -I"C:/ti/mspm0_sdk_1_00_01_03/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_1_00_01_03/source" -D__MSPM0G3507__ -gdwarf-3 -MMD -MP -MF"syscfg/$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/A0501766/Desktop/TIDA-010042-E3_SW/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


