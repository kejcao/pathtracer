#include "camera.h"

int main(void) {
	Camera<256,256>(
		vec(3, 1, 0),
		vec(M_PI/2, -M_PI/2, -M_PI/2)
	).render(
		Scene("examples/pathtrace.obj"), "1.ppm");
}