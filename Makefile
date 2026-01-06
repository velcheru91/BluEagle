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
CPP      = arm-none-eabi-g++
CC       = arm-none-eabi-gcc
AR       = arm-none-eabi-ar
LD       = arm-none-eabi-ld
OBJCOPY  = arm-none-eabi-objcopy
SRCDIR   = source
INCDIR   = header
CFILE    = main.c
BINDIR   = bin
OBJ      = $(PROJECT).o
LD_FILE  = $(PROJECT).ld
SRCS= $(wildcard $(SRCDIR)/*.c)
OBJECTS   = $(patsubst $(SRCDIR)/%.c,$(BINDIR)/%.o,$(SRCS))
OBJS     = $(addprefix $(BINDIR)/,$(notdir $(SRCS:.c=.o)))
BIN      = $(PROJECT)
# Set the compiler CPU/FPU options.
CPU=-mcpu=cortex-m4
FPU=-mfpu=fpv4-sp-d16 -mfloat-abi=hard
#INCLUDE FLAGS
CINCS = -I"C:/Program Files (x86)/GNU Arm Embedded Toolchain/10-2021.10/arm-none-eabi/include/" 
INCS = $(CINCS) -I./$(INCDIR) -I./$(SRCDIR) -I./inc
CXXINCS += $(CINCS)
#-specs=rdimon.specs
CFLAGS =-mcpu=cortex-m4       \
    -mthumb                   \
	-mfpu=fpv4-sp-d16         \
    -mfloat-abi=hard          \
    -ffunction-sections       \
    -fdata-sections           \
    -O0                       \
    -g3                       \
	-specs=nosys.specs        \
    -Wall                     \
    -Wextra                   \
    -Wpedantic #-Werror
CXXFLAGS += $(CFLAGS) 
#LINKER FLAGS
LDFLAGS = -T $(LD_FILE) -e ResetISR -Wl,-Map=$(BINDIR)/$(PROJECT).map
RM  = C:/cygwin/bin/rm.exe -f

all: clean all-before $(BINDIR)/$(BIN).bin all-after

$(OBJECTS): $(BINDIR)/%.o: $(SRCDIR)/%.c 
	$(CC) -c $< -o $@ $(INCS) $(CFLAGS)

#$(BINDIR)/$(OBJ): $(OBJECTS)
#	$(CC) -c -o $@ $(INCS) $(CFLAGS) $(LDFLAGS)

#Program .elf
#$(BINDIR)/$(BIN).elf: $(BINDIR)/$(OBJ) # $(LIB) # $(BINDIR)/main.o
$(BINDIR)/$(BIN).elf: $(OBJECTS)
	$(CC) -o $@ $^ $(INCS) $(CFLAGS) $(LDFLAGS) 

#Binary
$(BINDIR)/$(BIN).bin: $(BINDIR)/$(BIN).elf
	$(OBJCOPY) -O binary $< $@

clean: clean-custom
	$(RM) $(BINDIR)/*

.PHONY: all all-before all-after clean clean-custom
