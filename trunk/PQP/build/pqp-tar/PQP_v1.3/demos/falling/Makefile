# Must set these gl and glut locations to build 'falling'

GL_INCPATH = -I/usr/local/include/
GL_LIBPATH = -L/usr/local/lib/ -L/usr/X11R6/lib/
GL_LIBS  =  -lglut -lGLU -lGL -lXext -lXmu -lXi -lX11

.SUFFIXES: .cpp

CC = g++
CFLAGS  = -O2 -I. -I../../include $(GL_INCPATH)
LDFLAGS	= -L. -L../../lib $(GL_LIBPATH)
LDLIBS  = -lPQP -lm       $(GL_LIBS) 

SRCS    = main.cpp model.cpp 

OBJECTS	= main.o model.o

TARGET = falling

CLEAN   = $(OBJECTS) $(TARGET)

.cpp.o:
	$(CC) ${CFLAGS} -c $<

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJECTS) -L. $(LDFLAGS) $(LDLIBS)

run: $(TARGET)
	$(TARGET)

clean:
	/bin/rm -f $(CLEAN)

