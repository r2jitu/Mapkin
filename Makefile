CC = gcc

OBJDIR = obj
TARGETDIR = .
TARGET = MonkeyBot
DEFINES = 
INCLUDES = -Ilibfreenect/include -Ilibfreenect/wrappers/c_sync 
CPPFLAGS = $(DEFINES) $(INCLUDES)
CFLAGS = -Wall -g
CXXFLAGS = $(CFLAGS)
LDFLAGS = -Wall -g -rdynamic -Wl,-rpath,libfreenect/build/lib:  
LIBS = -lGLU -lGL -lSM -lICE -lX11 -lXext -lglut -lXmu -lXi -lusb-1.0 -lpthread -lm libfreenect/build/lib/libfreenect.so.0.0.1 libfreenect/build/lib/libfreenect_sync.so.0.0.1

all: mapkin grid_main

mapkin: mapkin.c.o
	echo "foo"
	$(CC) -o $@ $(LDFLAGS) $(LIBS) $^

grid_main: matrix_util.c.o grid_main.c.o
	$(CC) -o $@ $(LDFLAGS) $(LIBS) $^

%.c.o: %.c
	$(CC) $(CPPFLAGS) $(INCLUDES) -o $@ -c $<

clean:
	rm -f *.o mapkin grid_main

