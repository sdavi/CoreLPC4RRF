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
#NETWORKING = true
ESP8266WIFI = true
#SBC = true

TMC22XX = false

#Comment out to show compilation commands (verbose)
V=@

$(info Building RepRapFirmware for LPC1768/1769 based boards:)

OUTPUT_NAME=firmware

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
include RRFLibraries.mk
include RepRapFirmware.mk

ifeq ($(BUILD),Debug)
	DEBUG_FLAGS = -Og -g -DLPC_DEBUG
        $(info - Build: Debug) 
else
	DEBUG_FLAGS = -Os
        $(info - Build: Release)
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
            FLAGS += -DENABLE_UART3 -DENABLE_UART2 -DENABLE_UART1
        endif
endif

#lpcopen Defines
FLAGS += -DCORE_M3
#RTOS + enable mods to RTOS+TCP for RRF
FLAGS += -DRTOS -DFREERTOS_USED -DRRF_RTOSPLUS_MOD
FLAGS += -DDEVICE_USBDEVICE=1 -DTARGET_LPC1768
FLAGS +=  -Wall -c -mcpu=cortex-m3 -mthumb -ffunction-sections -fdata-sections -march=armv7-m 
FLAGS += -nostdlib -Wdouble-promotion -fsingle-precision-constant -fstack-usage
#FLAGS += -Wfloat-equal
#FLAGS += -Wundef
FLAGS += $(DEBUG_FLAGS)
FLAGS += -MMD -MP 

ifeq ($(NETWORKING), true)
        $(info  - Networking: Ethernet)
        FLAGS += -DLPC_NETWORKING
else ifeq ($(ESP8266WIFI), true)
        $(info  - Networking: ESP8266 WIFI) 
        FLAGS += -DESP8266WIFI
else ifeq ($(SBC), true)
        $(info  - SBC Interface Enabled)
        FLAGS += -DLPC_SBC
else
        $(info  - Networking: None)
endif

ifeq ($(TMC22XX), true)
        $(info  - Smart Drivers: TMC22XX)
        FLAGS += -DSUPPORT_TMC22xx
else
        $(info  - Smart Drivers: None)
endif

CFLAGS   = $(FLAGS) -std=gnu11 -fgnu89-inline
CXXFLAGS = $(FLAGS) -std=gnu++17 -fno-threadsafe-statics -fexceptions -fno-rtti -Wno-register


#all Includes (RRF + Core)
INCLUDES = $(CORE_INCLUDES) $(RRFLIBRARIES_INCLUDES) $(RRF_INCLUDES) $(RRFLIBC_INCLUDES)


DEPS = $(CORE_OBJS:.o=.d)
DEPS += $(RRF_OBJS:.o=.d)
DEPS += $(RRFLIBC_OBJS:.o=.d)
DEPS += $(RRFLIBRARIES_OBJS:.o=.d)

default: all

all: firmware

-include $(DEPS)

firmware:  $(BUILD_DIR)/$(OUTPUT_NAME).elf

coreLPC: $(BUILD_DIR)/core.a

$(BUILD_DIR)/libLPCCore.a: $(CORE_OBJS)
	$(V)$(AR) rcs $@ $(CORE_OBJS)
	@echo "\nBuilt LPCCore\n"
	
$(BUILD_DIR)/libRRFLibraries.a: $(RRFLIBRARIES_OBJS)
	$(V)$(AR) rcs $@ $(RRFLIBRARIES_OBJS)
	@echo "\nBuilt RRF Libraries\n"
	
$(BUILD_DIR)/$(OUTPUT_NAME).elf: $(BUILD_DIR)/libLPCCore.a $(BUILD_DIR)/libRRFLibraries.a $(RRFLIBC_OBJS) $(RRF_OBJS)
	@echo "\nCreating $(OUTPUT_NAME).bin"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(LD) -L$(BUILD_DIR)/ -L$(CORE)/variants/LPC/linker_scripts/gcc/ --specs=nosys.specs -Os -Wl,--warn-section-align -Wl,--fatal-warnings -march=armv7-m -mcpu=cortex-m3 -T$(LINKER_SCRIPT) -Wl,-Map,$(BUILD_DIR)/$(OUTPUT_NAME).map -o $(BUILD_DIR)/$(OUTPUT_NAME).elf  -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--entry=Reset_Handler -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,--warn-unresolved-symbols -Wl,--start-group  $(BUILD_DIR)/$(CORE)/cores/arduino/syscalls.o $(RRFLIBC_OBJS) -lLPCCore $(RRF_OBJS) -lsupc++ -lRRFLibraries  -Wl,--end-group -lm 
	$(V)$(OBJCOPY) --strip-unneeded -O binary $(BUILD_DIR)/$(OUTPUT_NAME).elf $(BUILD_DIR)/$(OUTPUT_NAME).bin
	$(V)$(SIZE) $(BUILD_DIR)/$(OUTPUT_NAME).elf
	-@./staticMemStats.sh $(BUILD_DIR)/$(OUTPUT_NAME).elf
	
$(BUILD_DIR)/%.o: %.c
	@echo "[$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CC)  $(CFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CC)  $(CFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<
	
$(BUILD_DIR)/%.o: %.cpp
	@echo "[$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CXX) $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CXX) $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

$(BUILD_DIR)/%.o: %.cc
	@echo "[$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CXX) $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CXX) $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<


cleanrrf:
	-rm -f $(RRF_OBJS)  $(BUILD_DIR)/libRRFLibraries.a
	
cleancore:
	-rm -f $(CORE_OBJS) $(BUILD_DIR)/libLPCCore.a

cleanrrflibraries:
	-rm -f $(RRFLIBRARIES_OBJS) $(BUILD_DIR)/libRRFLibraries.a

clean: distclean

distclean:
	-rm -rf $(BUILD_DIR)/ 

.PHONY: all firmware clean distclean $(BUILD_DIR)/$(OUTPUT_NAME).elf
