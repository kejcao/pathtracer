raytracer: main.cpp
	# g++ -std=c++20 -pedantic -Wall -O3 $^ -o $@
	g++ -std=c++20 -pedantic -Wall -g $^ -o $@

test: raytracer
	./raytracer && convert out.ppm out.png && sxiv out.png

clean:
	rm -f raytracer *.png *.ppm
