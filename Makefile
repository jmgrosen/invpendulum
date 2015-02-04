LDFLAGS = -pthread

OFILES = src/eqep.o src/main.o

.PHONY: all clean

all: invpendulum

clean:
	rm -f $(OFILES) invpendulum

invpendulum: $(OFILES)
	$(CC) -o invpendulum $(OFILES) $(CFLAGS) $(LDFLAGS)
