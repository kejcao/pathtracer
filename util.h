#ifndef UTIL_H
#define UTIL_H

#include <mutex>
#include <functional>
#include <thread>
#include <iostream>
#include <vector>
#include <iomanip>
#include "math.h"

#define START() auto start = std::chrono::high_resolution_clock::now();
#define END(msg) \
    auto end = std::chrono::high_resolution_clock::now(); \
    double ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); \
    std::cout << std::fixed << std::setprecision(2) << "took " << ms/1000 << " to " << (msg) << std::endl;

vec wavelength_to_rgb(double nm) {
    vec color;
    if (380 <= (int)nm && (int)nm <= 439) {
        color = vec(-(nm-440) / (440-380), 0, 1);
    } else if (440 <= (int)nm && (int)nm <= 489) {
        color = vec(0, (nm-440) / (490-440), 1);
    } else if (490 <= (int)nm && (int)nm <= 509) {
        color = vec(0, 1, -(nm-510) / (510-490));
    } else if (510 <= (int)nm && (int)nm <= 579) {
        color = vec((nm-510) / (580-510), 1, 0);
    } else if (580 <= (int)nm && (int)nm <= 644) {
        color = vec(1, -(nm-645) / (645-580), 0);
    } else if (645 <= (int)nm && (int)nm <= 780) {
        color = vec(1, 0, 0);
    }

    int factor = 0;
    if (380 <= (int)nm && (int)nm <= 419) {
        factor = .3 + .7*(nm-380) / (420-380);
    } else if (420 <= (int)nm && (int)nm <= 700) {
        factor = 1;
    } else if (701 <= (int)nm && (int)nm <= 780) {
        factor = .3 + .7*(780-nm) / (780-700);
    }

    if (color.x != 0) color.x = round(std::pow(color.x * factor, .8));
    if (color.y != 0) color.y = round(std::pow(color.y * factor, .8));
    if (color.z != 0) color.z = round(std::pow(color.z * factor, .8));

    return color;
}

// Loop from start (inclusive) to end (exclusive).
void parallel_for(
    int start, int end, std::function<void(int)> f,
    int threadcnt=std::thread::hardware_concurrency()
) {
    std::mutex m;
    auto runner = [&]() {
        for (;;) {
            int i;
            {
                std::lock_guard<std::mutex> lock(m);
                if ((i = start++) >= end) return;
                std::cout << "\r" << i*100 / end << "%";
                std::cout.flush();
            }
            f(i);
        }
    };
    std::vector<std::thread> threads;
    // threadcnt - 1 since main thread counts as one too.
    threadcnt = std::min(threadcnt-1, end - start - 1);
    for (int i = 0; i < threadcnt; ++i)
        threads.push_back(std::thread(runner));
    runner();
    for (auto &&t : threads) t.join();
    std::cout << "\r";
}

#endif
