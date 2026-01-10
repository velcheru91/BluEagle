#******************************************************************************
#
# Makefile - Rules for building the libraries and examples.
#
# Copyright (c) 2005-2020 Texas Instruments Incorporated.  All rights reserved.
# Software License Agreement
# 
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#   Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# 
#   Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the  
#   distribution.
# 
#   Neither the name of Texas Instruments Incorporated nor the names of
#   its contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# This is part of revision 2.2.0.295 of the Tiva Firmware Development Package.
#
#******************************************************************************

PROJECT  = BluEagle
TOOLCHAIN = arm-none-eabi-
GCC = $(TOOLCHAIN)gcc
CC = $(GCC)
CXX = $(TOOLCHAIN)g++
AS = $(TOOLCHAIN)as
LD = $(TOOLCHAIN)ld
OBJCOPY = $(TOOLCHAIN)objcopy
AR = $(TOOLCHAIN)ar

# GCC flags
CFLAG = -c -O0 -g3
OFLAG = -o
INCLUDEFLAG = -I
CPUFLAG = -mthumb -mcpu=cortex-m4
WFLAG = -Wall -Wextra -Werror -Wno-unused-parameter
FPUFLAG=-mfpu=fpv4-sp-d16 -mfloat-abi=softfp
#-specs=rdimon.specs
#CFLAGS =-ffunction-sections       \
    -fdata-sections           \
	-specs=nosys.specs        \
#CXXFLAGS += $(CFLAGS) 

CFLAGS = $(CPUFLAG) $(WFLAG) -Dgcc
# Uncomment this if the application performs floating point operations
#CFLAGS += $(FPUFLAG)

# Additional C compiler flags to produce debugging symbols
DEB_FLAG = -g -DDEBUG

# Intermediate directory for all *.o and other files:
OBJDIR = obj/
BINDIR = bin/

# FreeRTOS source base directory
FREERTOS_SRC = FreeRTOS/Source/

# Directory with memory management source files
FREERTOS_MEMMANG_SRC = $(FREERTOS_SRC)portable/MemMang/

# Directory with platform specific source files
FREERTOS_PORT_SRC = $(FREERTOS_SRC)portable/GCC/tm4c123g/

# Directory with HW drivers' source files
DRIVERS_SRC = drivers/
DRIVERLIB_SRC = driverlib/

# Directory with demo specific source (and header) files
APP_DIR = app/
APP_SRC = app/source/
APP_INC = app/inc/

vpath %.c $(FREERTOS_SRC)
vpath %.c $(DRIVERS_SRC)
vpath %.c $(DRIVERLIB_SRC)
vpath %.c $(APP_SRC)

# Object files to be linked into an application
# Due to a large number, the .o files are arranged into logical groups:
# The following o. files are only necessary if
# certain options are enabled in FreeRTOSConfig.h
FREERTOS_OBJS = croutine.o event_groups.o list.o queue.o stream_buffer.o tasks.o timers.o


# Only one memory management .o file must be uncommented!
#FREERTOS_MEMMANG_OBJS = heap_1.o
#FREERTOS_MEMMANG_OBJS = heap_2.o
#FREERTOS_MEMMANG_OBJS = heap_3.o
FREERTOS_MEMMANG_OBJS = heap_4.o
#FREERTOS_MEMMANG_OBJS = heap_5.o

FREERTOS_PORT_OBJS = port.o

DRIVERS_OBJS = fpu.o scb.o
#Unnnecessary driver object files may be commented out
DRIVERLIB_OBJS = cpu.o gpio.o i2c.o interrupt.o pwm.o sysctl.o systick.o

APP_OBJS = main.o BSP_delay.o BSP_LCD.o nostdlib.o startup_gcc.o
# nostdlib.o must be commented out if standard lib is going to be linked!
#APP_OBJS += nostdlib.o

# All object files specified above are prefixed the intermediate directory
OBJS = $(addprefix $(OBJDIR), $(FREERTOS_OBJS) $(FREERTOS_MEMMANG_OBJS) \
		$(FREERTOS_PORT_OBJS) $(DRIVERS_OBJS) $(DRIVERLIB_OBJS) $(APP_OBJS))

# Definition of the linker script and final targets
LINKER_SCRIPT = $(addprefix $(APP_DIR), BluEagle.ld)
ELF_IMAGE = $(PROJECT).elf
TARGET = $(PROJECT).bin

# Include paths to be passed to $(CC) where necessary
INC_FREERTOS = $(FREERTOS_SRC)include/
INC_DRIVERS = $(DRIVERS_SRC)include/
INC_DRIVERLIB = $(DRIVERLIB_SRC)inc/

# Complete include flags to be passed to $(CC) where necessary
INC_FLAGS = $(INCLUDEFLAG) $(INC_FREERTOS) $(INCLUDEFLAG) $(APP_INC) \
            $(INCLUDEFLAG) $(FREERTOS_PORT_SRC) $(INCLUDEFLAG) $(INC_DRIVERS) \
			$(INCLUDEFLAG) $(INC_DRIVERLIB) $(INCLUDEFLAG) $(DRIVERLIB_SRC)

# Dependency on HW specific settings
DEP_BSP = $(INC_DRIVERS)bsp.h
DEP_FRTOS_CONFIG = $(APP_INC)/FreeRTOSConfig.h
DEP_SETTINGS = $(DEP_FRTOS_CONFIG)

#
# Make rules:
#
all : $(TARGET)

rebuild : clean all

$(TARGET) : $(OBJDIR) $(BINDIR) $(ELF_IMAGE)
	$(OBJCOPY) -O binary $(BINDIR)$(word 3,$^) $(BINDIR)$@

$(OBJDIR) :
	mkdir -p $@

$(BINDIR):
	mkdir -p $@

$(ELF_IMAGE) : $(OBJS) $(LINKER_SCRIPT)
	$(LD) -nostdlib -L $(OBJDIR) -T $(LINKER_SCRIPT) $(OBJS) $(OFLAG) $(BINDIR)$@

debug : _debug_flags all

debug_rebuild : _debug_flags rebuild

_debug_flags :
	$(eval CFLAGS += $(DEB_FLAG))
# FreeRTOS core
$(OBJDIR)%.o: %.c $(DEP_FRTOS_CONFIG)
	$(CC) $(CFLAG) $(CFLAGS) $(INC_FLAGS) $< $(OFLAG) $@

# HW specific part, in FreeRTOS/Source/portable/GCC/tm4c123g/

$(OBJDIR)port.o : $(FREERTOS_PORT_SRC)port.c $(DEP_FRTOS_CONFIG)
	$(CC) $(CFLAG) $(CFLAGS) $(INC_FLAGS) $< $(OFLAG) $@

# Rules for all MemMang implementations are provided
# Only one of these object files must be linked to the final target

#$(OBJDIR)heap_1.o : $(FREERTOS_MEMMANG_SRC)heap_1.c $(DEP_FRTOS_CONFIG)
#	$(CC) $(CFLAG) $(CFLAGS) $(INC_FLAGS) $< $(OFLAG) $@

#$(OBJDIR)heap_2.o : $(FREERTOS_MEMMANG_SRC)heap_2.c $(DEP_FRTOS_CONFIG)
#	$(CC) $(CFLAG) $(CFLAGS) $(INC_FLAGS) $< $(OFLAG) $@

#$(OBJDIR)heap_3.o : $(FREERTOS_MEMMANG_SRC)heap_3.c $(DEP_FRTOS_CONFIG)
#	$(CC) $(CFLAG) $(CFLAGS) $(INC_FLAGS) $< $(OFLAG) $@

$(OBJDIR)heap_4.o : $(FREERTOS_MEMMANG_SRC)heap_4.c $(DEP_FRTOS_CONFIG)
	$(CC) $(CFLAG) $(CFLAGS) $(INC_FLAGS) $< $(OFLAG) $@

#$(OBJDIR)heap_5.o : $(FREERTOS_MEMMANG_SRC)heap_5.c $(DEP_FRTOS_CONFIG)
#	$(CC) $(CFLAG) $(CFLAGS) $(INC_FLAGS) $< $(OFLAG) $@

# Drivers
$(OBJDIR)%.o: %.c $(DEP_BSP)
	$(CC) $(CFLAG) $(CFLAGS) $(INC_FLAGS) $< $(OFLAG) $@

#Driverlib 
$(OBJDIR)%.o: %.c
	$(CC) $(CFLAG) $(CFLAGS) $(INC_FLAGS) $< $(OFLAG) $@

# Demo Application 
$(OBJDIR)%.o: %.c $(DEP_SETTINGS)
	$(CC) $(CFLAG) $(CFLAGS) $(INC_FLAGS) $< $(OFLAG) $@

clean: clean-custom
	$(RM) -r $(OBJDIR)
	$(RM) -r $(BINDIR)

.PHONY: all all-before all-after clean clean-custom
