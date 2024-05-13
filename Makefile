CC = g++
FLAGS = -pedantic -Wall

raytracer: main.cpp $(wildcard *.h)
	$(CC) $(FLAGS) -c libattopng/libattopng.c -o libatto.o
	$(CC) $(FLAGS) -std=c++23 -lassimp libatto.o -march=native -O3 main.cpp -o $@

run: raytracer
	rm -f output/*
	./raytracer
	sxiv output/*

clean:
	rm -f raytracer
