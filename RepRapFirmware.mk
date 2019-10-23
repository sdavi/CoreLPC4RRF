#---RepRapFirmware---
RRF_SRC_BASE  = $(REPRAPFIRMWARE_DIR)/src

RRF_SRC_DIRS  = FilamentMonitors GCodes GCodes/GCodeBuffer Heating 
RRF_SRC_DIRS += Movement Movement/BedProbing Movement/Kinematics 
RRF_SRC_DIRS += Storage Libraries/Fatfs Libraries/Fatfs/port/lpc Libraries/sha1
RRF_SRC_DIRS += Heating/Sensors Fans ObjectModel Endstops Hardware Tools
RRF_SRC_DIRS += LPC LPC/MCP4461
RRF_SRC_DIRS += Display Display/ST7920
#RRF_SRC_DIRS += Linux


#networking support?
ifeq ($(NETWORKING), true)
	RRF_SRC_DIRS += Networking Networking/RTOSPlusTCPEthernet
else ifeq ($(ESP8266WIFI), true) 
	RRF_SRC_DIRS += Networking Networking/ESP8266WIFI
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
ifeq ($(ESP8266WIFI), true)
	RRF_INCLUDES += -IDuetWiFiSocketServer/src/include
endif

#end RRF



#---RRF Libraries----
RRF_LIBRARY_SRC_BASE = $(RRFLIBRARIES_DIR)/src
RRF_LIBRARY_SRC_DIRS = General Math RTOSIface

#  Find the c and cpp source files
RRF_LIBRARY_SRC = $(RRF_LIBRARY_SRC_BASE) $(addprefix $(RRF_LIBRARY_SRC_BASE)/, $(RRF_LIBRARY_SRC_DIRS))
RRF_OBJ_SRC_C      += $(foreach src, $(RRF_LIBRARY_SRC), $(wildcard $(src)/*.c) ) 
RRF_OBJ_SRC_CXX   += $(foreach src, $(RRF_LIBRARY_SRC), $(wildcard $(src)/*.cpp) )
RRF_INCLUDES += $(addprefix -I, $(RRF_LIBRARY_SRC))
#end RRF Libraries



