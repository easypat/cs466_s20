PART=TM4C123GH6PM

TARGET=spi_master

TIVA=../../TivaDriver
FREERTOS=../../FreeRTOSv10.2.1_191129/FreeRTOS/Source

SIZE=arm-none-eabi-size

FLASHOPTS=-v
FLASH=lm4flash
CP=/bin/cp

#
# Echo full command lines.. Comment out for terse
VERBOSE=1
DEBUG=1

#
# Include the common make definitions.
#
include makedefs

CFLAGS += -Wno-unused-value  
#CFLAGS += -save-temps  

CFLAGS += -DUSB_SERIAL_OUTPUT

LDFLAGS += --stats

ODIR = ${COMPILER}
dummy_build_folder := $(shell mkdir -p ${COMPILER})

#
# Where to find source files that do not live in this directory.
#
VPATH+=${FREERTOS}
VPATH+=${FREERTOS}/portable/GCC/ARM_CM4F
VPATH+=${FREERTOS}/portable/MemMang

VPATH+=${TIVA}/driverlib
VPATH+=${TIVA}/utils

#
# Where to find header files that do not live in the source directory.
#
IPATH=.
IPATH+=${FREERTOS}/portable/GCC/ARM_CM4F
IPATH+=${FREERTOS}/include
IPATH+=${TIVA}

#
# The default rule, which causes the FreeRTOS example to be built.
#
all: ${COMPILER}
all: ${COMPILER}/${TARGET}.axf

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}

#
# The rule to create the target directory.
#
#${COMPILER}:
#	mkdir -p ${COMPILER}

#
# Rules for building the Tiva Driver UARTPrintf utility
#
${COMPILER}/${TARGET}.axf: ${COMPILER}/uartstdio.o

#
# Rules for building the Tiva Driver Library core.
#
${COMPILER}/${TARGET}.axf: ${COMPILER}/adc.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/aes.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/can.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/comp.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/cpu.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/crc.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/des.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/eeprom.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/emac.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/epi.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/flash.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/fpu.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/gpio.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/hibernate.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/i2c.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/interrupt.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/lcd.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/mpu.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/pwm.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/qei.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/shamd5.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/ssi.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/sw_crc.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/sysctl.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/sysexc.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/systick.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/timer.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/uart.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/udma.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/usb.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/watchdog.o

#
# Rules for building the FreeRTOS core.
#
${COMPILER}/${TARGET}.axf: ${COMPILER}/heap_2.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/port.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/list.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/queue.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/tasks.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/timers.o

#
# Rules for building the application
#
${COMPILER}/${TARGET}.axf: ${COMPILER}/spi_master.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/assert.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/startup_${COMPILER}.o

#
# load map and device memory description
#
${COMPILER}/${TARGET}.axf: ti_tm4c123g.ld

SCATTERgcc_${TARGET}=ti_tm4c123g.ld
ENTRY_${TARGET}=ResetISR
CFLAGSgcc=-DTARGET_IS_BLIZZARD_RB1

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif

#
# A rule to flash and restarrt the program
#
flash: ${COMPILER}/${TARGET}.axf
	${SIZE} ${COMPILER}/${TARGET}.axf
	${FLASH} ${FLASHOPTS} ${COMPILER}/${TARGET}.bin
