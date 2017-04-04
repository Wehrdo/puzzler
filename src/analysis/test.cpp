#include "edge_tools.hpp"
#include <vector>
#include <functional>
#include <cmath>
#include <cstdio>

std::vector<Vec2d> gen_curve(int N, double start, double end, std::function<double(double)> func) {
    std::vector<Vec2d> points;
    double range = end - start;
    for (int i = 0; i < N; i++) {
        double x = range * (double(i) / N) + start;
        points.push_back(Vec2d(x, func(x)));
    }
    return points;
};

double identity(double x) {return x;}

// 3x^4 - x^3 - 5x^2 + x
// inflection points should be at -0.85, 0.01, 1.0
double polynomial(double x) {
    return 3*pow(x, 4) - pow(x, 3) - 5*pow(x, 2) + x;
}
int main(int argc, char* argv[]) {
    int N = 48;
    auto points = gen_curve(N, -2, 2, polynomial);
    auto inflection = find_inflections(points);
};