# This file was automagically generated by mbed.org. For more information, 
# see http://mbed.org/handbook/Exporting-to-GCC-ARM-Embedded

# cross-platform directory manipulation
ifeq ($(shell echo $$OS),$$OS)
    MAKEDIR = if not exist "$(1)" mkdir "$(1)"
    RM = rmdir /S /Q "$(1)"
else
    MAKEDIR = $(SHELL) -c "mkdir -p \"$(1)\""
    RM = $(SHELL) -c "rm -rf \"$(1)\""
endif

ifeq (,$(filter .build,$(notdir $(CURDIR))))
.SUFFIXES:
OBJDIR := .build
mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
MAKETARGET = $(MAKE) --no-print-directory -C $(OBJDIR) -f $(mkfile_path) \
		SRCDIR=$(CURDIR) $(MAKECMDGOALS)
.PHONY: $(OBJDIR) clean
all:
	+@$(call MAKEDIR,$(OBJDIR))
	+@$(MAKETARGET)
$(OBJDIR): all
Makefile : ;
% :: $(OBJDIR) ; :
clean :
	$(call RM,$(OBJDIR))

else

VPATH = .. 

GCC_BIN = 
PROJECT = BLE_Keyboard
OBJECTS = nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/ble_radio_notification/ble_radio_notification.o nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/ble_services/ble_dfu/ble_dfu.o nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/common/ble_advdata.o nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/common/ble_conn_state.o nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/common/ble_srv_common.o nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/device_manager/device_manager_peripheral.o nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/peer_manager/id_manager.o nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/peer_manager/peer_data.o nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/peer_manager/peer_data_storage.o nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/peer_manager/peer_database.o nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/peer_manager/peer_id.o nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/peer_manager/pm_buffer.o nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/peer_manager/pm_mutex.o nRF51822/TARGET_MCU_NRF51822/sdk/source/drivers_nrf/ble_flash/ble_flash.o nRF51822/TARGET_MCU_NRF51822/sdk/source/drivers_nrf/delay/nrf_delay.o nRF51822/TARGET_MCU_NRF51822/sdk/source/drivers_nrf/hal/nrf_ecb.o nRF51822/TARGET_MCU_NRF51822/sdk/source/drivers_nrf/hal/nrf_nvmc.o nRF51822/TARGET_MCU_NRF51822/sdk/source/drivers_nrf/pstorage/pstorage.o nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/bootloader_dfu/bootloader_util.o nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/bootloader_dfu/dfu_app_handler.o nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/bootloader_dfu/dfu_init_template.o nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/crc16/crc16.o nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/fds/fds.o nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/fstorage/fstorage.o nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/fstorage/fstorage_nosd.o nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/hci/hci_mem_pool.o nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/scheduler/app_scheduler.o nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/util/app_error.o nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/util/app_util_platform.o nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/util/nrf_assert.o nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/util/sdk_mapped_flags.o nRF51822/TARGET_MCU_NRF51822/sdk/source/softdevice/common/softdevice_handler/softdevice_handler.o nRF51822/TARGET_MCU_NRF51822/sdk/source/softdevice/common/softdevice_handler/softdevice_handler_appsh.o main.o BLE_API/source/BLE.o BLE_API/source/BLEInstanceBase.o BLE_API/source/DiscoveredCharacteristic.o BLE_API/source/GapScanningParams.o BLE_API/source/services/DFUService.o BLE_API/source/services/UARTService.o BLE_API/source/services/URIBeaconConfigService.o nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/common/ble_conn_params.o nRF51822/TARGET_MCU_NRF51822/source/nRF5xCharacteristicDescriptorDiscoverer.o nRF51822/TARGET_MCU_NRF51822/source/nRF5xDiscoveredCharacteristic.o nRF51822/TARGET_MCU_NRF51822/source/nRF5xGap.o nRF51822/TARGET_MCU_NRF51822/source/nRF5xGattClient.o nRF51822/TARGET_MCU_NRF51822/source/nRF5xGattServer.o nRF51822/TARGET_MCU_NRF51822/source/nRF5xServiceDiscovery.o nRF51822/TARGET_MCU_NRF51822/source/nRF5xn.o nRF51822/TARGET_MCU_NRF51822/source/btle/btle.o nRF51822/TARGET_MCU_NRF51822/source/btle/btle_advertising.o nRF51822/TARGET_MCU_NRF51822/source/btle/btle_discovery.o nRF51822/TARGET_MCU_NRF51822/source/btle/btle_gap.o nRF51822/TARGET_MCU_NRF51822/source/btle/btle_security.o nRF51822/TARGET_MCU_NRF51822/source/btle/custom/custom_helper.o BLE_HID/HIDServiceBase.o 
SYS_OBJECTS = mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM/analogin_api.o mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM/cmsis_nvic.o mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM/gpio_api.o mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM/gpio_irq_api.o mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM/i2c_api.o mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM/mbed_board.o mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM/pinmap.o mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM/port_api.o mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM/pwmout_api.o mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM/retarget.o mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM/serial_api.o mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM/sleep.o mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM/spi_api.o mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM/startup_NRF51822.o mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM/system_nrf51.o mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM/twi_master.o mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM/us_ticker.o 
INCLUDE_PATHS = -I../. -I../shields -I../BLE_API -I../BLE_API/ble -I../BLE_API/ble/services -I../BLE_API/source -I../BLE_API/source/services -I../nRF51822 -I../nRF51822/TARGET_MCU_NRF51822 -I../nRF51822/TARGET_MCU_NRF51822/bootloader -I../nRF51822/TARGET_MCU_NRF51822/sdk -I../nRF51822/TARGET_MCU_NRF51822/sdk/script -I../nRF51822/TARGET_MCU_NRF51822/sdk/source -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/ble -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/ble_radio_notification -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/ble_services -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/ble_services/ble_dfu -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/common -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/device_manager -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/device_manager/config -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/ble/peer_manager -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/device -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/drivers_nrf -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/drivers_nrf/ble_flash -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/drivers_nrf/delay -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/drivers_nrf/hal -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/drivers_nrf/pstorage -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/drivers_nrf/pstorage/config -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/bootloader_dfu -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/bootloader_dfu/hci_transport -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/crc16 -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/experimental_section_vars -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/fds -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/fstorage -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/hci -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/scheduler -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/timer -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/libraries/util -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/softdevice -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/softdevice/common -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/softdevice/common/softdevice_handler -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/softdevice/s130 -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/softdevice/s130/headers -I../nRF51822/TARGET_MCU_NRF51822/sdk/source/toolchain -I../nRF51822/TARGET_MCU_NRF51822/source -I../nRF51822/TARGET_MCU_NRF51822/source/btle -I../nRF51822/TARGET_MCU_NRF51822/source/btle/custom -I../nRF51822/TARGET_MCU_NRF51822/source/common -I../BLE_HID -I../mbed/. -I../mbed/TARGET_RBLAB_BLENANO -I../mbed/TARGET_RBLAB_BLENANO/TARGET_NORDIC -I../mbed/TARGET_RBLAB_BLENANO/TARGET_NORDIC/TARGET_MCU_NRF51822 -I../mbed/TARGET_RBLAB_BLENANO/TARGET_NORDIC/TARGET_MCU_NRF51822/Lib -I../mbed/TARGET_RBLAB_BLENANO/TARGET_NORDIC/TARGET_MCU_NRF51822/Lib/nordic_sdk -I../mbed/TARGET_RBLAB_BLENANO/TARGET_NORDIC/TARGET_MCU_NRF51822/Lib/nordic_sdk/components -I../mbed/TARGET_RBLAB_BLENANO/TARGET_NORDIC/TARGET_MCU_NRF51822/Lib/nordic_sdk/components/libraries -I../mbed/TARGET_RBLAB_BLENANO/TARGET_NORDIC/TARGET_MCU_NRF51822/Lib/nordic_sdk/components/libraries/crc16 -I../mbed/TARGET_RBLAB_BLENANO/TARGET_NORDIC/TARGET_MCU_NRF51822/Lib/nordic_sdk/components/libraries/scheduler -I../mbed/TARGET_RBLAB_BLENANO/TARGET_NORDIC/TARGET_MCU_NRF51822/Lib/nordic_sdk/components/libraries/util -I../mbed/TARGET_RBLAB_BLENANO/TARGET_NORDIC/TARGET_MCU_NRF51822/Lib/s110_nrf51822_8_0_0 -I../mbed/TARGET_RBLAB_BLENANO/TARGET_NORDIC/TARGET_MCU_NRF51822/Lib/s130_nrf51822_1_0_0 -I../mbed/TARGET_RBLAB_BLENANO/TARGET_NORDIC/TARGET_MCU_NRF51822/TARGET_RBLAB_BLENANO -I../mbed/TARGET_RBLAB_BLENANO/TARGET_NORDIC/TARGET_MCU_NRF51822/device -I../mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM -I../mbed/drivers -I../mbed/hal -I../mbed/platform 
LIBRARY_PATHS = -L../mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM 
LIBRARIES = -lmbed 
LINKER_SCRIPT = ../mbed/TARGET_RBLAB_BLENANO/TOOLCHAIN_GCC_ARM/NRF51822.ld
SOFTDEVICE = ../mbed/TARGET_RBLAB_BLENANO/TARGET_NORDIC/TARGET_MCU_NRF51822/Lib/s130_nrf51822_1_0_0/s130_nrf51_1.0.0_softdevice.hex


############################################################################### 
AS      = $(GCC_BIN)arm-none-eabi-as
CC      = $(GCC_BIN)arm-none-eabi-gcc
CPP     = $(GCC_BIN)arm-none-eabi-g++
LD      = $(GCC_BIN)arm-none-eabi-gcc
OBJCOPY = $(GCC_BIN)arm-none-eabi-objcopy
OBJDUMP = $(GCC_BIN)arm-none-eabi-objdump
SIZE    = $(GCC_BIN)arm-none-eabi-size
SREC_CAT = srec_cat
 


CPU = -mcpu=cortex-m0 -mthumb 
CC_FLAGS = -c -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fmessage-length=0 -fno-exceptions -fno-builtin -ffunction-sections -fdata-sections -funsigned-char -MMD -fno-delete-null-pointer-checks -fomit-frame-pointer -mcpu=cortex-m0 -mthumb -Os -std=gnu99 -DDEVICE_ERROR_PATTERN=1 -DNRF51 -DTARGET_MCU_NRF51_16K_S130 -DTARGET_LIKE_MBED -DTARGET_NRF51822 -DDEVICE_PORTINOUT=1 -D__MBED_CMSIS_RTOS_CM -DTOOLCHAIN_object -D__CMSIS_RTOS -DTARGET_MCU_NRF51_16K -DTOOLCHAIN_GCC -DMBED_BUILD_TIMESTAMP=1481606839.89 -DTARGET_CORTEX_M -DARM_MATH_CM0 -DTARGET_UVISOR_UNSUPPORTED -DDEVICE_ANALOGIN=1 -DTARGET_M0 -DTARGET_MCU_NRF51 -DDEVICE_SERIAL=1 -D__MBED__=1 -D__CORTEX_M0 -DDEVICE_I2C=1 -DDEVICE_PORTOUT=1 -DTARGET_RELEASE -DTARGET_NORDIC -DFEATURE_BLE=1 -DTARGET_MCU_NORDIC_16K -DDEVICE_PORTIN=1 -DDEVICE_SLEEP=1 -DTOOLCHAIN_GCC_ARM -DTARGET_MCU_NRF51822 -DDEVICE_SPI=1 -DDEVICE_INTERRUPTIN=1 -DMBED_RTOS_SINGLE_THREAD -DDEVICE_SPISLAVE=1 -DTARGET_RBLAB_BLENANO -DDEVICE_PWMOUT=1 -DTARGET_LIKE_CORTEX_M0 -include mbed_config.h -MMD -MP
CPPC_FLAGS = -c -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fmessage-length=0 -fno-exceptions -fno-builtin -ffunction-sections -fdata-sections -funsigned-char -MMD -fno-delete-null-pointer-checks -fomit-frame-pointer -mcpu=cortex-m0 -mthumb -Os -std=gnu++98 -fno-rtti -Wvla -DDEVICE_ERROR_PATTERN=1 -DNRF51 -DTARGET_MCU_NRF51_16K_S130 -DTARGET_LIKE_MBED -DTARGET_NRF51822 -DDEVICE_PORTINOUT=1 -D__MBED_CMSIS_RTOS_CM -DTOOLCHAIN_object -D__CMSIS_RTOS -DTARGET_MCU_NRF51_16K -DTOOLCHAIN_GCC -DMBED_BUILD_TIMESTAMP=1481606839.89 -DTARGET_CORTEX_M -DARM_MATH_CM0 -DTARGET_UVISOR_UNSUPPORTED -DDEVICE_ANALOGIN=1 -DTARGET_M0 -DTARGET_MCU_NRF51 -DDEVICE_SERIAL=1 -D__MBED__=1 -D__CORTEX_M0 -DDEVICE_I2C=1 -DDEVICE_PORTOUT=1 -DTARGET_RELEASE -DTARGET_NORDIC -DFEATURE_BLE=1 -DTARGET_MCU_NORDIC_16K -DDEVICE_PORTIN=1 -DDEVICE_SLEEP=1 -DTOOLCHAIN_GCC_ARM -DTARGET_MCU_NRF51822 -DDEVICE_SPI=1 -DDEVICE_INTERRUPTIN=1 -DMBED_RTOS_SINGLE_THREAD -DDEVICE_SPISLAVE=1 -DTARGET_RBLAB_BLENANO -DDEVICE_PWMOUT=1 -DTARGET_LIKE_CORTEX_M0 -include mbed_config.h -MMD -MP
ASM_FLAGS = -x assembler-with-cpp -D__CORTEX_M0 -DMBED_RTOS_SINGLE_THREAD -DTARGET_MCU_NRF51_16K -DNRF51 -DTARGET_MCU_NRF51_16K_S130 -DTARGET_NRF51822 -DARM_MATH_CM0 -D__MBED_CMSIS_RTOS_CM -D__CMSIS_RTOS -DTARGET_MCU_NORDIC_16K -c -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fmessage-length=0 -fno-exceptions -fno-builtin -ffunction-sections -fdata-sections -funsigned-char -MMD -fno-delete-null-pointer-checks -fomit-frame-pointer -mcpu=cortex-m0 -mthumb -Os
CC_SYMBOLS = -DDEVICE_ERROR_PATTERN=1 -DNRF51 -DTARGET_MCU_NRF51_16K_S130 -DTARGET_LIKE_MBED -DTARGET_NRF51822 -DDEVICE_PORTINOUT=1 -D__MBED_CMSIS_RTOS_CM -DTOOLCHAIN_object -D__CMSIS_RTOS -DTARGET_MCU_NRF51_16K -DTOOLCHAIN_GCC -DMBED_BUILD_TIMESTAMP=1481606839.89 -DTARGET_CORTEX_M -DARM_MATH_CM0 -DTARGET_UVISOR_UNSUPPORTED -DDEVICE_ANALOGIN=1 -DTARGET_M0 -DTARGET_MCU_NRF51 -DDEVICE_SERIAL=1 -D__MBED__=1 -D__CORTEX_M0 -DDEVICE_I2C=1 -DDEVICE_PORTOUT=1 -DTARGET_RELEASE -DTARGET_NORDIC -DFEATURE_BLE=1 -DTARGET_MCU_NORDIC_16K -DDEVICE_PORTIN=1 -DDEVICE_SLEEP=1 -DTOOLCHAIN_GCC_ARM -DTARGET_MCU_NRF51822 -DDEVICE_SPI=1 -DDEVICE_INTERRUPTIN=1 -DMBED_RTOS_SINGLE_THREAD -DDEVICE_SPISLAVE=1 -DTARGET_RBLAB_BLENANO -DDEVICE_PWMOUT=1 -DTARGET_LIKE_CORTEX_M0 

LD_FLAGS =-Wl,--gc-sections -Wl,--wrap,main -Wl,--wrap,_malloc_r -Wl,--wrap,_free_r -Wl,--wrap,_realloc_r -Wl,--wrap,_calloc_r -mcpu=cortex-m0 -mthumb --specs=nano.specs 
LD_SYS_LIBS = -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys


ifeq ($(DEBUG), 1)
  CC_FLAGS += -DDEBUG -O0
else
  CC_FLAGS += -DNDEBUG -Os
endif


.PHONY: all lst size

all: $(PROJECT).bin $(PROJECT).hex size



.asm.o:
	+@$(call MAKEDIR,$(dir $@))
	$(CC) $(CPU) -c $(ASM_FLAGS) $(CC_SYMBOLS) $(INCLUDE_PATHS) -o $@ $<
.s.o:
	+@$(call MAKEDIR,$(dir $@))
	$(CC) $(CPU) -c $(ASM_FLAGS) $(CC_SYMBOLS) $(INCLUDE_PATHS) -o $@ $<
.S.o:
	+@$(call MAKEDIR,$(dir $@))
	$(CC) $(CPU) -c $(ASM_FLAGS) $(CC_SYMBOLS) $(INCLUDE_PATHS) -o $@ $<

.c.o:
	+@$(call MAKEDIR,$(dir $@))
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) $(INCLUDE_PATHS) -o $@ $<

.cpp.o:
	+@$(call MAKEDIR,$(dir $@))
	$(CPP) $(CPPC_FLAGS) $(CC_SYMBOLS) $(INCLUDE_PATHS) -o $@ $<



$(PROJECT).elf: $(OBJECTS) $(SYS_OBJECTS) $(LINKER_SCRIPT)
	$(LD) $(LD_FLAGS) -T$(filter %.ld, $^) $(LIBRARY_PATHS) -o $@ $(filter %.o, $^) -Wl,--start-group $(LIBRARIES) $(LD_SYS_LIBS) -Wl,--end-group


$(PROJECT).bin: $(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

$(PROJECT).hex: $(PROJECT).elf
	@$(OBJCOPY) -O ihex $< $@

$(PROJECT).lst: $(PROJECT).elf
	@$(OBJDUMP) -Sdh $< > $@

lst: $(PROJECT).lst

size: $(PROJECT).elf
	$(SIZE) $(PROJECT).elf

DEPS = $(OBJECTS:.o=.d) $(SYS_OBJECTS:.o=.d)
-include $(DEPS)


merge:
	$(SREC_CAT) $(SOFTDEVICE) -intel $(PROJECT).hex -intel -o combined.hex -intel --line-length=44

endif
