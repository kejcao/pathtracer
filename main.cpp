#include "math.h"
#include "objects.h"
#include "lights.h"
#include "scene.h"
#include "camera.h"
#include "wavefront.h"

int main(void) {
    Camera<256, 256>(vec(0, 5, -10), vec(.3, 0, 0)).render(
        Scene(parseobj("examples/untitled.obj"), {new PointLight(vec(0, 5, -2))}), "out.ppm");
}