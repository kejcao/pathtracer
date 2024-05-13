#include <chrono>
#include <format>
#include <iostream>

class Progress {
    using MS = std::chrono::milliseconds;
    using clock = std::chrono::steady_clock;

public:
    Progress(int total) : total{total} { start = clock::now(); }

    void increment() {
        progress += 1;
        update();
    }

    void update() {
        auto current = clock::now();
        double ms = std::chrono::duration_cast<MS>(current - start).count();

        if (progress >= total) {
            std::cout << "\r" << std::format("took {:.2f} secs\n", ms / 1000);
            return;
        }

        std::cout << std::format("\r{}/{}", progress, total);
        std::cout.flush();
    }

private:
    int total, progress = 0;
    std::chrono::time_point<clock> start;
};
