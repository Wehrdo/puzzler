#include "edge_tools.hpp"
#include <vector>
#include <algorithm>

cv::Mat draw_curve(const std::vector<Vec2d>& points, int width, std::vector<Vec2d> inflections) {
    auto comp_x = [](Vec2d a, Vec2d b) {return a.x < b.x;};
    auto comp_y = [](Vec2d a, Vec2d b) {return a.y < b.y;};
    Vec2d max_x = *std::max_element(points.begin(), points.end(), comp_x);
    Vec2d max_y = *std::max_element(points.begin(), points.end(), comp_y);
    Vec2d min_x = *std::min_element(points.begin(), points.end(), comp_x);
    Vec2d min_y = *std::min_element(points.begin(), points.end(), comp_y);
    double scale = double(width) / (max_x.x - min_x.x);
    int height = ceil(scale * (max_y.y - min_y.y));
    // Function converts x,y values to pixel opencv point representing pixel location
    // Also takes into account that positive Y points downward in graphics
    auto convert_coord = [scale, min_x, min_y, max_x, max_y](Vec2d pt) {
                return cv::Point((pt.x - min_x.x) * scale,
                                scale * (max_y.y - pt.y));};
    // Create image matrix with dimensions to fill width, and proportional height
    cv::Mat out_img(height, width, CV_8UC1);
    cv::Scalar color = cv::Scalar(255, 255, 255);


    Vec2d last_vec = points[points.size() - 1];
    cv::Point last_pt = convert_coord(last_vec);
    for (Vec2d p : points) {
        cv::Point this_pt = convert_coord(p);
        cv::line(out_img, last_pt, this_pt, color);
        last_pt = this_pt;
    }

    for (Vec2d p : inflections) {
        cv::Point infl_pt = convert_coord(p);
        cv::circle(out_img, infl_pt, 10, color);
    }

    return out_img;
}
