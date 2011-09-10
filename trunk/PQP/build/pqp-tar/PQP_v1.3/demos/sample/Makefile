CC = g++

CFLAGS  = -O2 -I. -I../../include
LDFLAGS	= -L. -L../../lib
LDLIBS  = -lPQP -lm      

.SUFFIXES: .cpp

SRCS    = main.cpp

OBJECTS	= main.o

TARGET  = sample

CLEAN   = $(OBJECTS) $(TARGET)

.cpp.o:
	$(CC) ${CFLAGS} -c $<

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJECTS) -L. $(LDFLAGS) $(LDLIBS)

run: $(TARGET)
	$(TARGET)

clean:
	/bin/rm -f $(CLEAN)

