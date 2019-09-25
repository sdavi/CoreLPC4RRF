PROCESSOR = LPC17xx


#Enable when debugging on MBED to swap serial and USB 
#and select direct ld script
MBED = true

BUILD_DIR = ./build

BUILD = Debug
#BUILD = Release

#compile in Ethernet Networking?
NETWORKING = true

#enable DFU
USE_DFU = true
#USE_DFU = false

#Comment out to show compilation commands (verbose)
V=@


## Cross-compilation commands 
CC      = arm-none-eabi-gcc
CXX     = arm-none-eabi-g++
LD      = arm-none-eabi-gcc
AR      = arm-none-eabi-ar
AS      = arm-none-eabi-as
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE    = arm-none-eabi-size



#Sources Locations
RRF_SRC_BASE  = RepRapFirmware/src
CORE = CoreLPC2

#RTOS location
RTOS_SRC = FreeRTOS/src
RTOS_INCLUDE = $(RTOS_SRC)/include/
RTOS_SRC_PORTABLE = $(RTOS_SRC)/portable/GCC/ARM_CM3

RTOSPLUS_SRC = FreeRTOS
RTOS_CONFIG_INCLUDE = FreeRTOSConfig


OUTPUT_NAME=firmware

$(info  - Firmware Filename:  $(OUTPUT_NAME).bin)
$(info  - Building Network Support: $(NETWORKING))



ifeq ($(BUILD),Debug)
	DEBUG_FLAGS = -Og -g
else
	DEBUG_FLAGS = -Os#-O2
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


MKDIR = mkdir -p


#Flags common for Core in c and c++
FLAGS  = -D__$(PROCESSOR)__ -D_XOPEN_SOURCE

ifeq ($(MBED), true)
$(info Building for MBED)
	FLAGS += -D__MBED__
endif

#lpcopen Defines
FLAGS += -DCORE_M3
#RTOS + enable mods to RTOS+TCP for RRF
FLAGS += -DRTOS -DFREERTOS_USED -DRRF_RTOSPLUS_MOD
#
FLAGS += -DDEVICE_USBDEVICE=1 -DTARGET_LPC1768

FLAGS +=  -Wall -c -mcpu=cortex-m3 -mthumb -ffunction-sections -fdata-sections -march=armv7-m 
FLAGS += -nostdlib -Wdouble-promotion -fsingle-precision-constant
#FLAGS += -Wundef
FLAGS += $(DEBUG_FLAGS)
FLAGS += -MMD -MP 

ifeq ($(NETWORKING), true)
        FLAGS += -DLPC_NETWORKING
endif


CFLAGS   = $(FLAGS) -std=gnu11 -fgnu89-inline
CXXFLAGS = $(FLAGS) -std=gnu++17  -fno-threadsafe-statics -fno-exceptions -fno-rtti -Wno-register



#Core
CORE_SRC_DIRS  = cores cores/arduino system variants/LPC
CORE_SRC_DIRS += cores/ExploreM3 cores/ExploreM3/ExploreM3_lib
CORE_SRC_DIRS += cores/lpcopen/src cores/mbed
#Core libraries
CORE_SRC_DIRS += libraries/WIRE libraries/SDCard libraries/SharedSPI libraries/SoftwarePWM
#mbed
CORE_SRC_DIRS += cores/mbed/usb/hal cores/mbed/usb/TARGET_NXP cores/mbed/usb/USBDevice cores/mbed/usb/USBPHY
CORE_SRC_DIRS += cores/mbed/usb/USBSerial cores/mbed/usb/utilities cores/mbed/platform

CORE_SRC = $(CORE) $(addprefix $(CORE)/, $(CORE_SRC_DIRS))
CORE_INCLUDES = $(addprefix -I, $(CORE_SRC))


ifeq ($(USE_DFU), true)
        #enable DFU
        $(info Enabling DFU)
        CXXFLAGS += -DENABLE_DFU
endif

#Additional Core Includes
CORE_INCLUDES	+= -I$(CORE)/system/ExploreM3_lib/
CORE_INCLUDES   += -I$(CORE)/system/CMSIS/CMSIS/Include/

#openlpc 
CORE_INCLUDES  += -I$(CORE)/cores/lpcopen/inc

#Find all c and c++ files for Core
CORE_OBJ_SRC_C    += $(foreach src, $(CORE_SRC), $(wildcard $(src)/*.c))
CORE_OBJ_SRC_CXX   += $(foreach src, $(CORE_SRC), $(wildcard $(src)/*.cpp))
CORE_OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(CORE_OBJ_SRC_C)) $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(CORE_OBJ_SRC_CXX))



#RTOS Sources (selected dirs only)
RTOS_CORE_SRC    += $(RTOS_SRC) $(RTOS_SRC_PORTABLE)
RTOS_CORE_OBJ_SRC_C  += $(foreach src, $(RTOS_CORE_SRC), $(wildcard $(src)/*.c) )
#RTOS Dynamic Memory Management
RTOS_CORE_OBJ_SRC_C  += $(RTOS_SRC)/portable/MemMang/heap_5.c

CORE_OBJS += $(patsubst %.c,$(BUILD_DIR)/%.o,$(RTOS_CORE_OBJ_SRC_C))

#RTOS Includes
CORE_INCLUDES   += -I$(RTOS_INCLUDE) -I$(RTOS_SRC_PORTABLE)
CORE_INCLUDES   += -I$(RTOS_CONFIG_INCLUDE)


#RTOS-Plus (selected dirs only)
#RTOS +TCP
RTOSPLUS_TCP_SRC = $(RTOSPLUS_SRC)/FreeRTOS-Plus-TCP
RTOSPLUS_TCP_INCLUDE = $(RTOSPLUS_TCP_SRC)/include/

RTOSPLUS_CORE_SRC    += $(RTOSPLUS_TCP_SRC) $(RTOS_SRC_PORTABLE)
RTOSPLUS_CORE_OBJ_SRC_C  += $(foreach src, $(RTOSPLUS_CORE_SRC), $(wildcard $(src)/*.c) )
#** RTOS+TCP Portable**

#(portable) Select Buffer Management (only enable one)
RTOSPLUS_CORE_OBJ_SRC_C  += $(RTOSPLUS_TCP_SRC)/portable/BufferManagement/BufferAllocation_1.c #Static Ethernet Buffers
#RTOSPLUS_CORE_OBJ_SRC_C  += $(RTOSPLUS_TCP_SRC)/portable/BufferManagement/BufferAllocation_2.c #Dynamic Ethernet Buffers

#(portable) Compiler Includes (GCC)
RTOSPLUS_TCP_INCLUDE += -I$(RTOSPLUS_TCP_SRC)/portable/Compiler/GCC

#(portable) NetworkInterface
RTOSPLUS_TCP_INCLUDE += -I$(RTOSPLUS_TCP_SRC)/portable/NetworkInterface/include
RTOSPLUS_TCP_NI_SRC    += $(RTOSPLUS_TCP_SRC)/portable/NetworkInterface/Common
RTOSPLUS_TCP_NI_SRC    += $(RTOSPLUS_TCP_SRC)/portable/NetworkInterface/LPC17xx 
RTOSPLUS_CORE_OBJ_SRC_C  += $(foreach src, $(RTOSPLUS_TCP_NI_SRC), $(wildcard $(src)/*.c) )


ifeq ($(NETWORKING), true)
	CORE_OBJS += $(patsubst %.c,$(BUILD_DIR)/%.o,$(RTOSPLUS_CORE_OBJ_SRC_C))
	#RTOS+TCP Includes
	CORE_INCLUDES   += -I$(RTOSPLUS_TCP_INCLUDE) -I$(RTOSPLUS_TCP_INCLUDE)	
endif



#---RepRapFirmware---
RRF_SRC_DIRS  = FilamentMonitors GCodes GCodes/GCodeBuffer Heating Movement Movement/BedProbing Movement/Kinematics 
RRF_SRC_DIRS += Storage Tools Libraries/Fatfs Libraries/Fatfs/port/lpc Libraries/sha1
RRF_SRC_DIRS += Heating/Sensors Fans ObjectModel Endstops Hardware Linux
RRF_SRC_DIRS += LPC LPC/MCP4461

RRF_SRC_DIRS += Display Display/ST7920

#build in LCD Support? only when networking is false
#networking support?
ifeq ($(NETWORKING), true)
	RRF_SRC_DIRS += Networking Networking/RTOSPlusTCPEthernet
else
	RRF_SRC_DIRS += LPC/NoNetwork
endif

#Find the c and cpp source files
RRF_SRC = $(RRF_SRC_BASE) $(addprefix $(RRF_SRC_BASE)/, $(RRF_SRC_DIRS))
RRF_OBJ_SRC_C	   += $(foreach src, $(RRF_SRC), $(wildcard $(src)/*.c) ) 
RRF_OBJ_SRC_CXX   += $(foreach src, $(RRF_SRC), $(wildcard $(src)/*.cpp) )


RRF_OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(RRF_OBJ_SRC_C)) $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(RRF_OBJ_SRC_CXX))

RRF_INCLUDES = $(addprefix -I, $(RRF_SRC))
RRF_INCLUDES += -I$(RRF_SRC_BASE)/Libraries/

#end RRF


#---RRF Libraries----
RRF_LIBRARY_SRC_BASE = RRFLibraries/src
RRF_LIBRARY_SRC_DIRS = General Math RTOSIface

#  Find the c and cpp source files
RRF_LIBRARY_SRC = $(RRF_LIBRARY_SRC_BASE) $(addprefix $(RRF_LIBRARY_SRC_BASE)/, $(RRF_LIBRARY_SRC_DIRS))
RRF_OBJ_SRC_C      += $(foreach src, $(RRF_LIBRARY_SRC), $(wildcard $(src)/*.c) ) 
RRF_OBJ_SRC_CXX   += $(foreach src, $(RRF_LIBRARY_SRC), $(wildcard $(src)/*.cpp) )
RRF_INCLUDES += $(addprefix -I, $(RRF_LIBRARY_SRC))
#end RRF Libraries



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

	@./staticMemStats.sh 

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

upload: $(BUILD_DIR)/$(OUTPUT_NAME).elf
	@echo "Flashing firmware via DFU"
	dfu-util -d 1d50:6015 -D $(OUTPUT_NAME).bin -R

cleanrrf:
	-rm -f $(RRF_OBJS)
	
cleancore:
	-rm -f $(BUILD_DIR)/core.a $(CORE_OBJS)

clean: distclean

distclean:
	-rm -rf $(BUILD_DIR)/ 
	-rm -f firmware.elf firmware.bin firmware.map

.PHONY: all  clean distclean
