CXXFLAGS = -std=c++0x

LDFLAGS = -pthread

OFILES = src/eqep.o src/main.o src/common.o src/pwm.o src/pins.o src/servo.o

.PHONY: all clean

all: invpendulum

clean:
	rm -f $(OFILES) invpendulum

invpendulum: $(OFILES)
	$(CXX) -std=c++0x -o invpendulum $(OFILES) $(CFLAGS) $(LDFLAGS)
