INCLUDE_DIRS =
LIB_DIRS =
CC=gcc

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS=

SDL2_CFLAGS = $(shell sdl2-config --cflags)
SDL2_LIBS   = $(shell sdl2-config --libs)

HFILES= 
CFILES= seqgenex0.c  seqgen3.c seqv4l2.c capturelib.c

SRCS= ${HFILES} ${CFILES}
OBJS= ${CFILES:.c=.o}

all:	seqgenex0 seqgen3 seqv4l2 clock_times capture

clean:
	-rm -f *.o *.d frames/*.pgm frames/*.ppm
	-rm -f seqgenex0 seqgen  seqgen3 seqv4l2 clock_times capture

seqgenex0: seqgenex0.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o -lpthread -lrt

seqv4l2: seqv4l2.o capturelib.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o capturelib.o -lpthread -lrt $(SDL2_LIBS)

seqv4l2.o: seqv4l2.c
	$(CC) $(CFLAGS) $(SDL2_CFLAGS) -c seqv4l2.c

seqgen3: seqgen3.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o -lpthread -lrt

clock_times: clock_times.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o -lpthread -lrt

capture: capture.o capturelib.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o capturelib.o -lrt

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<
