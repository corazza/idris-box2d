CC = g++
# CFLAGS = -Wall `idris --include` -l:libBox2D.a
CPPFLAGS = -Wall `idris --include` -lBox2D -fpermissive
# CPPFLAGS = -fpermissive

# box2d.o: api.o box2d.c box2d.h
box2d.o: box2d.cpp box2d.h
# api.o: api.cpp api.h

.PHONY: clean
clean:
	# rm -f box2d.o api.o *.ibc
	rm -f box2d.o *.ibc
