CC = gcc
# CFLAGS = -Wall `idris --include` -l:libBox2D.a
CFLAGS = -Wall `idris --include` -lBox2D
# CPPFLAGS = -fpermissive

box2d.o: api.o box2d.c box2d.h
api.o: api.cpp api.h

.PHONY: clean
clean:
	rm -f box2d.o api.o *.ibc
