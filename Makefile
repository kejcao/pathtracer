CC = g++
FLAGS = -std=c++20 -pedantic -Wall -lassimp

raytracer: main.cpp $(wildcard *.h)
	$(CC) -march=native -O3 $(FLAGS) main.cpp -o $@

debug: main.cpp $(wildcard *.h)
	# $(CC) -fanalyzer -fsanitize=address -ggdb $(FLAGS) main.cpp -o $@
	$(CC) -ggdb $(FLAGS) main.cpp -o $@

run: raytracer
	./raytracer && sxiv *.ppm

clean:
	rm -f raytracer *.png *.ppm
