PROJECT_NAME := dfu_dual_bank_serial_s130_pca10028
TARGETS          := nrf51422_xxac
OUTPUT_DIRECTORY := _build

SDK_ROOT := $(HOME)/nRF5_SDK_11.0.0_89a8197
PROJ_DIR := $(HOME)/workspace/ble_bootloader_sdk11


export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

TEMPLATE_PATH = $(SDK_ROOT)/components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#	$(SDK_ROOT)/components/libraries/uart/app_uart.c \
#	$(SDK_ROOT)/components/drivers_nrf/uart/nrf_drv_uart.c \


#source common to all targets
C_SOURCE_FILES += \
	$(SDK_ROOT)/components/libraries/util/app_error.c \
	$(SDK_ROOT)/components/libraries/util/app_error_weak.c \
	$(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
	$(SDK_ROOT)/components/libraries/timer/app_timer.c \
	$(SDK_ROOT)/components/libraries/timer/app_timer_appsh.c \
	$(SDK_ROOT)/components/libraries/util/app_util_platform.c \
	$(SDK_ROOT)/components/libraries/bootloader_dfu/bootloader.c \
	$(SDK_ROOT)/components/libraries/bootloader_dfu/bootloader_settings.c \
	$(SDK_ROOT)/components/libraries/bootloader_dfu/bootloader_util.c \
	$(SDK_ROOT)/components/libraries/crc16/crc16.c \
	$(SDK_ROOT)/components/libraries/bootloader_dfu/dfu_dual_bank.c \
	$(SDK_ROOT)/components/libraries/bootloader_dfu/dfu_init_template.c \
	$(SDK_ROOT)/components/libraries/hci/hci_mem_pool.c \
	$(SDK_ROOT)/components/libraries/hci/hci_slip.c \
	$(SDK_ROOT)/components/libraries/hci/hci_transport.c \
	$(SDK_ROOT)/components/libraries/util/nrf_assert.c \
	$(SDK_ROOT)/components/libraries/bootloader_dfu/dfu_transport_ble.c \
	$(SDK_ROOT)/components/drivers_nrf/delay/nrf_delay.c \
	$(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
	$(SDK_ROOT)/components/drivers_nrf/pstorage/pstorage_raw.c \
	$(PROJ_DIR)/Application/dfu_ble_svc.c \
	$(PROJ_DIR)/Application/main.c \
	$(SDK_ROOT)/components/ble/common/ble_advdata.c \
	$(SDK_ROOT)/components/ble/common/ble_conn_params.c \
	$(SDK_ROOT)/components/ble/ble_services/ble_dfu/ble_dfu.c \
	$(SDK_ROOT)/components/ble/common/ble_srv_common.c \
	$(SDK_ROOT)/components/toolchain/system_nrf51.c \
	$(SDK_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
	$(SDK_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler_appsh.c \


#assembly files common to all targets
ASM_SOURCE_FILES = $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf51.s \

#includes common to all targets
INC_PATHS = -I$(SDK_ROOT)/components/device
INC_PATHS += -I$(PROJ_DIR)/config 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/bootloader_dfu/hci_transport
INC_PATHS += -I$(SDK_ROOT)/components/libraries/bootloader_dfu
INC_PATHS += -I$(SDK_ROOT)/components/libraries/scheduler
INC_PATHS += -I$(SDK_ROOT)/components/libraries/fifo
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/config
INC_PATHS += -I$(SDK_ROOT)/examples/bsp
INC_PATHS += -I$(SDK_ROOT)/components/softdevice/s130/headers
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/delay
INC_PATHS += -I$(SDK_ROOT)/components/libraries/crc16
INC_PATHS += -I$(SDK_ROOT)/components/libraries/util
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/pstorage/config
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/pstorage
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/uart
INC_PATHS += -I$(SDK_ROOT)/components/ble/common
INC_PATHS += -I$(SDK_ROOT)/components/libraries/hci/config
INC_PATHS += -I$(SDK_ROOT)/components/libraries/uart
INC_PATHS += -I$(SDK_ROOT)/components/libraries/hci
INC_PATHS += -I$(SDK_ROOT)/components/libraries/timer
INC_PATHS += -I$(SDK_ROOT)/components/toolchain/CMSIS/Include
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/hal
INC_PATHS += -I$(SDK_ROOT)/components/toolchain/gcc
INC_PATHS += -I$(SDK_ROOT)/components/toolchain
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/common
INC_PATHS += -I$(SDK_ROOT)/components/softdevice/s130/headers/nrf51
INC_PATHS += -I$(SDK_ROOT)/components/softdevice/common/softdevice_handler
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_dfu

OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -DSWI_DISABLE0
CFLAGS += -DBOARD_ITC_POLYU
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -D__HEAP_SIZE=0
CFLAGS += -DS130
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -Os -g3
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums 
# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DBOARD_ITC_POLYU
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -D__HEAP_SIZE=0
ASMFLAGS += -DS130
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DBSP_DEFINES_ONLY

#default target - first one defined
default: clean nrf51422_xxac

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51422_xxac

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf51422_xxac

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrf51422_xxac: OUTPUT_FILENAME := nrf51422_xxac
nrf51422_xxac: LINKER_SCRIPT=$(PROJ_DIR)/dfu_gcc_nrf51.ld

nrf51422_xxac: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Assembly file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<
# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o
#flash: nrf51422_xxac
#	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.hex
#	nrfjprog --program $(OUTPUT_BINARY_DIRECTORY)/$<.hex -f nrf51  --chiperase
#	nrfjprog --reset -f nrf51
#
## Flash softdevice
JLINK_OPTS = -device nrf51422_xxaa -if swd -speed 4000
JLINK_ROOT := /opt/SEGGER/JLink
SOFTDEVICE_PATH := $(SDK_ROOT)/components/softdevice/s130/hex
SOFTDEVICE_FILE := s130_nrf51_2.0.0_softdevice
GDB_PORT_NUMBER = 2331

ifeq ($(USE_SOFTDEVICE),)
	FLASH_START_ADDRESS = 0
else
	FLASH_START_ADDRESS = 0x03C000
endif

ifneq ($(OS),Windows_NT)
	UNAME_S := $(shell uname -s)
	ifeq ($(UNAME_S),Linux)
		export LD_LIBRARY_PATH := $(JLINK_ROOT):$(LD_LIBRARY_PATH)
	endif
	ifeq ($(UNAME_S),Darwin)
		export DYLD_LIBRARY_PATH := $(JLINK_ROOT):$(DYLD_LIBRARY_PATH)
	endif
endif

JLINK := $(JLINK_ROOT)/JLinkExe $(JLINK_OPTS)
JLINKGDBSERVER := $(JLINK_ROOT)/JLinkGDBServer


#	nrfutil settings generate --family NRF51 --application $(PROJ_DIR)/$(OUTPUT_DIRECTORY)/$(TARGETS).hex --application-version 1 --bootloader-version 1 --bl-settings-version 1 $(PROJ_DIR)/$(OUTPUT_DIRECTORY)/settings.hex
#	srec_cat $(PROJ_DIR)/$(OUTPUT_BINARY_DIRECTORY)/comb_bl_sd.hex --intel $(PROJ_DIR)/$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex --intel -o $(PROJ_DIR)/$(OUTPUT_BINARY_DIRECTORY)/comb_bl_sd_$(OUTPUT_FILENAME).hex --intel

genpack: combine genzip
	@echo GEN Package
	rm $(BUILD_DIRECTORIES)/*.o
	rm $(BUILD_DIRECTORIES)/*.map
	rm $(BUILD_DIRECTORIES)/*.out
	#rm $(BUILD_DIRECTORIES)/*.bin

combine:
	@echo Merging app and SoftDevice and App and Valid params
	srec_cat $(PROJ_DIR)/$(OUTPUT_DIRECTORY)/$(TARGETS).hex  --intel $(SOFTDEVICE_PATH)/$(SOFTDEVICE_FILE).hex --intel -o $(PROJ_DIR)/$(OUTPUT_BINARY_DIRECTORY)/comb_bl_sd.hex --intel
	srec_cat $(PROJ_DIR)/$(OUTPUT_BINARY_DIRECTORY)/comb_bl_sd.hex --intel $(PROJ_DIR)/app_valid_setting_apply.hex --intel -o $(PROJ_DIR)/$(OUTPUT_BINARY_DIRECTORY)/file_combined.hex --intel
	#rm $(PROJ_DIR)/$(BUILD_DIRECTORIES)/comb_bl*.hex	



genzip:
	@echo Package zip
	~/.wine/drive_c/Program\ Files\ \(x86\)/Nordic\ Semiconductor/Master\ Control\ Panel/3.10.0.14/nrf/nrfutil.exe dfu genpkg $(OUTPUT_BINARY_DIRECTORY)/$(TARGETS).zip --softdevice $(SOFTDEVICE_PATH)/$(SOFTDEVICE_FILE).hex --bootloader $(OUTPUT_BINARY_DIRECTORY)/$(TARGETS).hex --application-version 0xffff --dev-revision 0xffff --dev-type 0xffff --sd-req 0xfffe


# Flash the program
flash: erase-all flash-softdevice flash.jlink
	$(JLINK) flash.jlink
flash.jlink:
	printf "loadbin $(OUTPUT_DIRECTORY)/$(TARGETS).hex 0x03C000\nr\ng\nexit\n" > flash.jlink
#	printf "loadbin $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin $(FLASH_START_ADDR)\nr\ng\nexit\n" > flash.jlink
	
flash-softdevice: flash-softdevice.jlink
	$(JLINK) flash-softdevice.jlink
	
flash-softdevice.jlink:
	printf "loadbin $(SOFTDEVICE_PATH)/$(SOFTDEVICE_FILE).hex 0x001000\nr\ng\nexit\n" > flash-softdevice.jlink
erase-all: erase-all.jlink
	$(JLINK) erase-all.jlink

erase-all.jlink:
	echo "device nrf51822\nw4 4001e504 2\nw4 4001e50c 1\nw4 4001e514 1\nsleep 100\nr\nexit\n" >  erase-all.jlink


