CC=g++
CFLAGS= -std=c++11
all: convertfpbin

convertfpbin:shared.o matrix.o convertfpbin.cpp 
	$(CC) $(CFLAGS)  $(LPATH) shared.o matrix.o convertfpbin.cpp -o convertfpbin

shared.o:shared.cpp shared.h matrix.h 
	$(CC) $(CFLAGS) -c shared.cpp

matrix.o:matrix.cpp matrix.h
	$(CC) $(CFLAGS) -c matrix.cpp

clean:
	rm shared.o matrix.o convertfpbin
