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

LDFLAGS = -lm -lpthread -L$(LIBDIR) -lrt -ldrm -lmtdev -ldrm_omap -lstdc++ -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_objdetect
TARGET = camera_opencv_disp

all: $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) *.lo

$(TARGET): main.c v4l2.lo display-kms.lo util.lo vpe-common.lo input_cmd.lo drawing.lo exam_cv.lo
	$(CC) $(CFLAGS) -o $@ main.c v4l2.lo display-kms.lo util.lo vpe-common.lo input_cmd.lo drawing.lo exam_cv.lo $(LDFLAGS)

v4l2.lo: v4l2.c v4l2.h
	$(CC) -c $(CFLAGS) -o $@ v4l2.c

display-kms.lo: display-kms.c display-kms.h
	$(CC) -c $(CFLAGS) -o $@ display-kms.c

util.lo: util.c util.h
	$(CC) -c $(CFLAGS) -o $@ util.c

vpe-common.lo: vpe-common.c vpe-common.h
	$(CC) -c $(CFLAGS) -o $@ vpe-common.c

drawing.lo: drawing.c drawing.h font_8x8.h
	$(CC) -c $(CFLAGS) -o $@ drawing.c

input_cmd.lo: input_cmd.cpp input_cmd.h
	$(CXX) -c $(CXXFLAGS) -o $@ input_cmd.cpp

exam_cv.lo: exam_cv.cpp exam_cv.h
	$(CXX) -c $(CXXFLAGS) -o $@ exam_cv.cpp

