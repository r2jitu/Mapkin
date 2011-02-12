CC = gcc

OBJDIR = obj
TARGETDIR = .
TARGET = MonkeyBot
INCLUDES = -Ilibfreenect/include -Ilibfreenect/wrappers/c_sync 
CPPFLAGS = $(DEFINES) $(INCLUDES)
CFLAGS = -Wall -O2
CXXFLAGS = $(CFLAGS)
LDFLAGS = -Wall -rdynamic -Wl,-rpath,libfreenect/build/lib:  
LIBS = -lGLU -lGL -lSM -lICE -lX11 -lXext -lglut -lXmu -lXi -lusb-1.0 -lpthread -lm libfreenect/build/lib/libfreenect.so.0.0.1 libfreenect/build/lib/libfreenect_sync.so.0.0.1

all: mapkin serial_test

mapkin: matrix_util.c.o display_util.c grid_util.c.o planning_util.c.o sim_util.c.o mapkin.c.o serial_comm.c.o
	$(CC) -g -o $@ $(LDFLAGS) $(LIBS) $^

serial_test: serial_comm.c.o serial_test.c.o
	$(CC) -o $@ $(LDFLAGS) $(LIBS) $^

%.c.o: %.c
	$(CC) $(CPPFLAGS) $(INCLUDES) -o $@ -c $<

clean:
	rm -f *.o grid_main serial_test

