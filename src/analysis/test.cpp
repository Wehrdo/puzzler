#include "edge_tools.hpp"
#include <vector>
#include <functional>
#include <cmath>
#include <cstdio>
#include <opencv2/opencv.hpp>

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

double whatever_func(double x) {
    return sin(3*x);
}

int main(int argc, char* argv[]) {
    int N = 48;
    std::vector<Vec2d> points = gen_curve(N, -1.25, 1.4, polynomial);
    auto infl_indices = find_inflections(points);
    std::vector<Vec2d> infl_points(infl_indices.size());
    for (int i = 0; i < infl_indices.size(); ++i) {
        infl_points[i] = points[infl_indices[i]];
    }
    cv::Mat img = draw_curve(points, 480, infl_points);

    cv::namedWindow("Holy cow", cv::WINDOW_AUTOSIZE);
    cv::imshow("Holy cow", img);
    cv::waitKey(0);
};