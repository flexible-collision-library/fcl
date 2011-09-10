# Must set these gl and glut locations to build 'spinning'

CC = g++

GL_INCPATH = -I/usr/include/
GL_LIBPATH = -L/usr/lib/ -L/usr/X11R6/lib/
GL_LIBS = -lGLU -lGL -lXext -lXmu -lXi -lX11 -lglut

.SUFFIXES: .cpp

CC = g++
CFLAGS  = -g -O2 -I. -I../../include $(GL_INCPATH)
LDFLAGS = -L. -L../../lib -L/usr/lib/ -L/usr/X11R6/lib/
LDLIBS  = -lPQP -lm $(GL_LIBS) 

OBJS   = main.o model.o
TARGET = spinning

.cpp.o:
	$(CC) ${CFLAGS} -c $<

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $(TARGET) $(LDFLAGS) $(LDLIBS)

run: $(TARGET)
	$(TARGET)

clean: 
	rm -f *~ $(OBJS) $(TARGET) 







