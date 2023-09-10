#include <exception>
#include <stdexcept>
#include "math.h"
#include "objects.h"
#include "lights.h"
#include "scene.h"
#include "camera.h"
#include "wavefront.h"

int main(void) {
    auto radians = [](double deg) { return deg * M_PI/180; };
    try {
        // Camera<512, 512>(vec(-.2, 1.3, -2), vec(.2, 0, 0)).render(
            // Scene(parseobj("examples/dragon.obj"), {new PointLight(vec(0, 5, -2))}), "out.ppm");
        Camera<256, 256>(vec(3, 1, 0), vec(M_PI/2, -M_PI/2, -M_PI/2)).render(
            Scene(parseobj("examples/pathtrace.obj"), {new PointLight(vec(3, .1, 0))}), "1.ppm");
    } catch(std::exception &e) {
        std::cout << e.what() << std::endl;
        return 1;
    }
}