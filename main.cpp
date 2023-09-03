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
        Camera<256, 256>(vec(0, 5, -10), vec(.3, 0, 0)).render(
            // Scene(parseobj("examples/cornell-box.obj"), {new PointLight(vec(0, 5, -2))}), "out.ppm");
            Scene(parseobj("examples/untitled.obj"), {new PointLight(vec(0, 5, -2))}), "out.ppm");
    } catch(std::exception &e) {
        std::cout << e.what() << std::endl;
        return 1;
    }
}