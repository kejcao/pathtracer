# Pathtracer

I wrote a pathtracer to mostly practice my C++ skills. Definitely not complete, very slow, barely works.

## Installation

To use,

```bash
$ mkdir build && cd build
$ CXX=clang++ cmake -DCMAKE_BUILD_TYPE=Release -GNinja ..
$ ninja
$ cd .. && build/raytracer
```

## Resources

A list of resources I have found useful. For future me.

- [On building BVHs.](https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/).
- [On sampling both area lights and DOF.](https://courses.cs.washington.edu/courses/cse557/17au/assets/lectures/aa-and-mcpt-4pp.pdf).
- [On the math behind Monte Carlo integration.](https://jacco.ompf2.com/2019/12/11/probability-theory-for-physically-based-rendering/)
- [On generating random uniform directions.](http://blog.thomaspoulet.fr/uniform-sampling-on-unit-hemisphere/)
- [On generating random uniform directions.](https://math.stackexchange.com/questions/1163260/random-directions-on-hemisphere-oriented-by-an-arbitrary-vector)
