# makefile for building fpga_conf

# Set the defaults

ifndef OPTIMIZE
    OPTIMIZE := 1
endif

ifndef DEBUG
    DEBUG := 0
endif

# General definitions

CC := ${CROSS_COMPILE}gcc
AR := ${CROSS_COMPILE}ar
RM := rm -f

CFLAGS := -Wall -Wextra -Wconversion
LDFLAGS :=

INCLUDES := -I.
LIBS :=

OBJS :=

TARGET := fpga_conf

# Modify the FLAGS based on the options

ifneq ($(OPTIMIZE), 0)
    CFLAGS := $(CFLAGS) -O3
else
    CFLAGS := $(CFLAGS) -O0
endif

ifneq ($(DEBUG), 0)
    CFLAGS := $(CFLAGS) -g -ggdb
    LDFLAGS := $(LDFLAGS) -g
endif

# Main target

all: $(TARGET)

include module.mk

# Pattern rules

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

clean:
	$(RM) $(TARGET) $(OBJS)

.PHONY: all clean
