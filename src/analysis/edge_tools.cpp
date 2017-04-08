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
    cv::Mat out_img(height, width, CV_8UC1, cv::Scalar(0));
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

std::vector<std::size_t> find_inflections(std::vector<Vec2d > points, double threshold) {
    std::size_t N = points.size();
    if (N < 3) {
        return std::vector<std::size_t>();
    }
    double first_angle = calc_angle(Vec2d(1,0), points[0] - points[N-1]);
    double second_angle = calc_angle(Vec2d(1,0), points[1] - points[0]);
    printf("raw: %f\nraw: %f\n", first_angle, second_angle);
    int last_sign = sign(second_angle - first_angle);

    double last_angle = second_angle;
    std::vector<std::size_t> inflections;
    for (std::size_t i = 0; i < N; ++i) {
        // printf("x: %f, y: %f\n", points[i].x, points[i].y);
        Vec2d new_vec = points[i] - points[i-1];
        // double angle = calc_angle(new_vec, last_vec);
        double raw_angle = calc_angle(Vec2d(1,0), new_vec);
        double angle_diff = raw_angle - last_angle;
        int this_sign = sign(angle_diff);
        if (this_sign != last_sign) {
            printf("Turning point at %f\n", points[i].x);
            inflections.push_back(i);
        }
        printf("raw: %f, diff: %f\n", raw_angle, angle_diff);
        last_angle = raw_angle;
        last_sign = this_sign;
    }
    return inflections;
}

// std::vector<Vec2d> calc_curvature(const std::vector<Vec2d>& points) {
//     std::vector<Vec2d> curve();
//     std::vector<double> distances;
//     curve.push_back((points[0] - points[N-1]).normalized());
    
//     for (std::size_t i = 1; i < points.size(); i++) {
//         curve.push_back((points[i] - points[i-1]).normalized());
//     }
//     // Curve now contains vectors for all lines

// }
