CP_PATH=$(abspath ../..)/CrossCompiler
CROSS_COMPILE=$(CP_PATH)/gcc-linaro-arm-linux-gnueabihf/bin/arm-linux-gnueabihf-

ARCH=arm
CC=$(CROSS_COMPILE)gcc
OBJ = estop.o car_lib.c

OBJNAME = estop
$(OBJNAME): $(OBJ)
	$(CC) -o $(OBJNAME) $(OBJ)

clean:
	rm -rf *.o $(OBJNAME)
