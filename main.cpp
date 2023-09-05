#include <exception>
#include <stdexcept>
#include "math.h"
#include "objects.h"
#include "lights.h"
#include "scene.h"
#include "camera.h"
#include "wavefront.h"

int main(void) {
    try {
        // Camera<256, 256>(vec(-.2, 1.3, -2), vec(.2, 0, 0)).render(
            // Scene(parseobj("examples/dragon.obj"), {new PointLight(vec(0, 5, -2))}), "out.ppm");
        Camera<256, 256>(vec(0, 2, -10), vec(.2, 0, 0)).render(
            Scene(parseobj("examples/untitled.obj"), {new PointLight(vec(0, 5, -2))}), "out.ppm");
        // Camera<256, 256>(vec(-.2, 1, -3), vec(.2, 0, 0)).render(
        //     Scene(parseobj("examples/plane.obj"), {new PointLight(vec(0, 5, -2))}), "out.ppm");
        // Camera<256, 256>(vec(-.2, 1, -3), vec(.2, 0, 0)).render(
        //     Scene(parseobj("sphere.obj"), {new PointLight(vec(0, 5, -2))}), "out.ppm");
    } catch(std::exception &e) {
        std::cout << e.what() << std::endl;
        return 1;
    }
}