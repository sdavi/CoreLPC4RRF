PROCESSOR = LPC17xx


#Enable when debugging on MBED to swap serial and USB 
#and select direct ld script
MBED = true



BUILD_DIR = ./build
FREERTOS_DIR = ./FreeRTOS
REPRAPFIRMWARE_DIR = ./RepRapFirmware
RRFLIBRARIES_DIR = ./RRFLibraries
CORELPC_DIR = ./CoreLPC2

BUILD = Debug
#BUILD = Release

#Enable only one
NETWORKING = true
#ESP8266WIFI = true

#Comment out to show compilation commands (verbose)
V=@


OUTPUT_NAME=firmware
$(info  - Firmware Filename:  $(OUTPUT_NAME).bin)


## Cross-compilation commands 
CC      = arm-none-eabi-gcc
CXX     = arm-none-eabi-g++
LD      = arm-none-eabi-gcc
AR      = arm-none-eabi-ar
AS      = arm-none-eabi-as
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE    = arm-none-eabi-size

MKDIR = mkdir -p


include LPCCore.mk
include FreeRTOS.mk
include RepRapFirmware.mk



ifeq ($(BUILD),Debug)
	DEBUG_FLAGS = -Og -g
else
	DEBUG_FLAGS = -Os
endif
	

#select correct linker script
ifeq ($(MBED), true)
	#No bootloader for MBED
	LINKER_SCRIPT_BASE = $(CORE)/variants/LPC/linker_scripts/gcc/LPC17xx_direct
else 
	#Linker script to avoid Smoothieware Bootloader
 	LINKER_SCRIPT_BASE = $(CORE)/variants/LPC/linker_scripts/gcc/LPC17xx_smoothie
endif


#Path to the linker Script
LINKER_SCRIPT  = $(LINKER_SCRIPT_BASE)_combined.ld
$(info  - Linker Script used: $(LINKER_SCRIPT))


#Flags common for Core in c and c++
FLAGS  = -D__$(PROCESSOR)__ -D_XOPEN_SOURCE

ifeq ($(MBED), true)
        $(info  - Building for MBED)
	    FLAGS += -D__MBED__
        ifeq ($(ESP8266WIFI), true)
		       FLAGS += -DENABLE_UART3
        endif
endif

#lpcopen Defines
FLAGS += -DCORE_M3
#RTOS + enable mods to RTOS+TCP for RRF
FLAGS += -DRTOS -DFREERTOS_USED -DRRF_RTOSPLUS_MOD
FLAGS += -DDEVICE_USBDEVICE=1 -DTARGET_LPC1768
FLAGS +=  -Wall -c -mcpu=cortex-m3 -mthumb -ffunction-sections -fdata-sections -march=armv7-m 
FLAGS += -nostdlib -Wdouble-promotion -fsingle-precision-constant
#FLAGS += -Wundef
FLAGS += $(DEBUG_FLAGS)
FLAGS += -MMD -MP 

ifeq ($(NETWORKING), true)
        $(info  - Building LPC Ethernet Support)
        FLAGS += -DLPC_NETWORKING
else ifeq ($(ESP8266WIFI), true)
        $(info  - Building ESP8266 WIFI Support) 
        FLAGS += -DESP8266WIFI
else
        $(info  - No Networking Support)
endif


CFLAGS   = $(FLAGS) -std=gnu11 -fgnu89-inline
CXXFLAGS = $(FLAGS) -std=gnu++17 -fno-threadsafe-statics -fno-exceptions -fno-rtti -Wno-register



#all Includes (RRF + Core)
INCLUDES = $(CORE_INCLUDES) $(RRF_INCLUDES)


DEPS = $(CORE_OBJS:.o=.d)
DEPS += $(RRF_OBJS:.o=.d)


default: all

all: firmware

-include $(DEPS)

firmware:  $(BUILD_DIR)/$(OUTPUT_NAME).elf

coreLPC: $(BUILD_DIR)/core.a

$(BUILD_DIR)/core.a: $(CORE_OBJS)

	@echo "\nBuilt LPCCore"
	$(V)$(AR) rcs $@ $(CORE_OBJS)

$(BUILD_DIR)/$(OUTPUT_NAME).elf: $(BUILD_DIR)/core.a $(RRF_OBJS) 
	@echo "\nCreating $(OUTPUT_NAME).bin"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(LD) -L$(BUILD_DIR)/ -L$(CORE)/variants/LPC/linker_scripts/gcc/ -Os --specs=nano.specs -u _printf_float -u _scanf_float -Wl,--warn-section-align -Wl,--gc-sections -Wl,--fatal-warnings -march=armv7-m -mcpu=cortex-m3 -T$(LINKER_SCRIPT) -Wl,-Map,$(OUTPUT_NAME).map -o $(OUTPUT_NAME).elf  -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--entry=Reset_Handler -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-unresolved-symbols -Wl,--start-group $(BUILD_DIR)/$(CORE)/cores/arduino/syscalls.o $(BUILD_DIR)/core.a $(RRF_OBJS) -Wl,--end-group -lm
	$(V)$(OBJCOPY) --strip-unneeded -O binary $(OUTPUT_NAME).elf $(OUTPUT_NAME).bin
	$(V)$(SIZE) $(OUTPUT_NAME).elf

	-@./staticMemStats.sh 

$(BUILD_DIR)/%.o: %.c
	@echo "[$(CC): Compiling $<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CC)  $(CFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CC)  $(CFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<
	
$(BUILD_DIR)/%.o: %.cpp
	@echo "[$(CXX): Compiling $<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CXX) $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CXX) $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

cleanrrf:
	-rm -f $(RRF_OBJS)
	
cleancore:
	-rm -f $(BUILD_DIR)/core.a $(CORE_OBJS)

clean: distclean

distclean:
	-rm -rf $(BUILD_DIR)/ 
	-rm -f firmware.elf firmware.bin firmware.map

.PHONY: all  clean distclean
