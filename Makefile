raytracer: main.cpp $(wildcard *.h)
	g++ -march=native -std=c++20 -pedantic -Wall -O3 main.cpp -o $@

test: raytracer
	./raytracer && sxiv *.ppm

clean:
	rm -f raytracer *.png *.ppm
