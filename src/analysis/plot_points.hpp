#ifndef PLOT_POINTS_H
#define PLOT_POINTS_H
#include "edge_tools.hpp"
#include <vector>
#include <opencv2/opencv.hpp>

cv::Mat plot_points(std::vector<Vec2d>& points, int width);

#endif // PLOT_POINTS_H