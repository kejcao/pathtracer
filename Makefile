CC = g++
FLAGS = -std=c++20 -pedantic -Wall -lassimp
LIBS = libattopng/libattopng.c

raytracer: main.cpp $(wildcard *.h)
	$(CC) $(LIBS) -march=native -O3 $(FLAGS) main.cpp -o $@

run: raytracer
	./raytracer && sxiv *.ppm

clean:
	rm -f raytracer *.png *.ppm
