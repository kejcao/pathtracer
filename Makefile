raytracer: main.cpp $(wildcard *.h)
	g++ -std=c++20 -pedantic -Wall -O3 main.cpp -o $@

test: raytracer
	./raytracer && convert out.ppm out.png && xdg-open out.png

clean:
	rm -f raytracer *.png *.ppm
