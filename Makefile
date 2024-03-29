CXX = arm-linux-gnueabihf-g++
CC = arm-linux-gnueabihf-gcc

CXXFLAGS += -std=c++11

LDFLAGS += -pthread -static

OFILES = src/eqep.o src/main.o src/common.o src/pwm.o src/pins.o src/servo.o src/output.o src/motor.o src/input.o

.PHONY: all clean upload

all: invpendulum

upload: invpendulum
	scp ./invpendulum mech@mechatronics.local:inverted-pendulum-new/

clean:
	rm -f $(OFILES) invpendulum

invpendulum: $(OFILES)
	$(CXX) -std=c++0x -o invpendulum $(OFILES) $(CFLAGS) $(LDFLAGS)
