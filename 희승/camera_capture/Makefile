CP_PATH=$(abspath ../..)/CrossCompiler
CROSS_COMPILE=$(CP_PATH)/gcc-linaro-arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
ROOTFS=$(CP_PATH)/sysroots/cortexa15t2hf-vfp-neon-linux-gnueabi

ARCH=arm
CC=$(CROSS_COMPILE)gcc
CXX=$(CROSS_COMPILE)g++

INC += -I protocol
INC += -I$(ROOTFS)/include
INC += -I$(ROOTFS)/usr/include
INC += -I$(ROOTFS)/usr/include/omap
INC += -I$(ROOTFS)/usr/include/libdrm
INC += -I$(ROOTFS)/usr/include/gbm
LIBDIR := $(ROOTFS)/usr/lib


CFLAGS := -O1 -g -Wall -fPIC -mfloat-abi=hard -mfpu=neon -Wl,-rpath,$(ROOTFS)/lib -Wl,-rpath,$(ROOTFS)/usr/lib $(INC)
CXXFLAGS = -Wall -ansi -g -fPIC -mfloat-abi=hard -mfpu=neon $(INC) -I$(ROOTFS)/include/c++/4.7.3/

LDFLAGS = -lm -lpthread -L$(LIBDIR) -lrt -ldrm -lmtdev -ldrm_omap -lstdc++
TARGET = camera_capture

all: $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) *.lo

$(TARGET): main.c v4l2.lo display-kms.lo util.lo input_cmd.lo
	$(CC) $(CFLAGS) -o $@ main.c v4l2.lo display-kms.lo util.lo input_cmd.lo $(LDFLAGS)

v4l2.lo: v4l2.c v4l2.h
	$(CC) -c $(CFLAGS) -o $@ v4l2.c

display-kms.lo: display-kms.c display-kms.h
	$(CC) -c $(CFLAGS) -o $@ display-kms.c

util.lo: util.c util.h
	$(CC) -c $(CFLAGS) -o $@ util.c

input_cmd.lo: input_cmd.cpp input_cmd.h
	$(CXX) -c $(CXXFLAGS) -o $@ input_cmd.cpp
