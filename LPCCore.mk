CORE = $(CORELPC_DIR)


#Core
CORE_SRC_DIRS  = cores cores/arduino system variants/LPC
CORE_SRC_DIRS += cores/ExploreM3 cores/ExploreM3/ExploreM3_lib
CORE_SRC_DIRS += cores/lpcopen/src cores/mbed
#Core libraries
CORE_SRC_DIRS += libraries/Wire libraries/SDCard libraries/SharedSPI libraries/SoftwarePWM libraries/ConfigurableUART
#mbed
CORE_SRC_DIRS += cores/mbed/usb/hal cores/mbed/usb/TARGET_NXP cores/mbed/usb/USBDevice cores/mbed/usb/USBPhy
CORE_SRC_DIRS += cores/mbed/usb/USBSerial cores/mbed/usb/utilities cores/mbed/platform

CORE_SRC_DIRS += cores/smoothie

CORE_SRC = $(CORE) $(addprefix $(CORE)/, $(CORE_SRC_DIRS))
CORE_INCLUDES = $(addprefix -I, $(CORE_SRC))


#Additional Core Includes
CORE_INCLUDES	+= -I$(CORE)/system/ExploreM3_lib/
CORE_INCLUDES   += -I$(CORE)/system/CMSIS/CMSIS/Include/

#openlpc 
CORE_INCLUDES  += -I$(CORE)/cores/lpcopen/inc

#Find all c and c++ files for Core
CORE_OBJ_SRC_C    += $(foreach src, $(CORE_SRC), $(wildcard $(src)/*.c))
CORE_OBJ_SRC_CXX   += $(foreach src, $(CORE_SRC), $(wildcard $(src)/*.cpp))
CORE_OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(CORE_OBJ_SRC_C)) $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(CORE_OBJ_SRC_CXX))
