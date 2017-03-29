#include "edge_tools.hpp"
#include <vector>
#include <functional>
#include <cmath>
#include <cstdio>

std::vector<Vec2d> gen_curve(int N, std::function<double(double)> func) {
    std::vector<Vec2d> points;
    for (int i = 0; i < N; i++) {
        double x = M_PI * ((double)i / N);
        points.push_back(Vec2d(x, func(x)));
    }
    return points;
};

double identity(double x) {return x;}

int main(int argc, char* argv[]) {
    int N = 24;
    auto points = gen_curve(N, sin);
    auto inflection = find_inflections(points);
};