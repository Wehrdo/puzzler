#include "plot_points.hpp"
#include <algorithm>

cv::Mat plot_points(std::vector<Vec2d>& points, int width) {
    auto comp_x = [](Vec2d a, Vec2d b) {return a.x < b.x;};
    auto comp_y = [](Vec2d a, Vec2d b) {return a.y < b.y;};
    Vec2d max_x = *std::max_element(points.begin(), points.end(), comp_x);
    Vec2d max_y = *std::max_element(points.begin(), points.end(), comp_y);
    Vec2d min_x = *std::min_element(points.begin(), points.end(), comp_x);
    Vec2d min_y = *std::min_element(points.begin(), points.end(), comp_y);
    double scale = double(width) / (max_x.x - min_x.x);
    cv::Mat out_img(ceil(scale * (max_y.y - min_y.y)), width, CV_8UC1);
    for (Vec2d p : points) {
        int x_pos = (p.x - min_x.x) * scale;
        int y_pos = (p.y - min_y.y) * scale;
        out_img.at<uint8_t>(cv::Point(x_pos,y_pos)) = 255;
    }
    return out_img;
}