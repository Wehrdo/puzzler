#include "edge_tools.hpp"
#include <vector>
#include <algorithm>

using namespace std;

Vec2d find_tangent_angle(int idx, vector<Vec2d> points) {
    int left_idx = idx == 0 ? points.size() - 1 : idx - 1;
    int right_idx = (idx + 1) % points.size();
    return points[left_idx] - points[right_idx];
}

cv::Mat draw_curve(const vector<Vec2d>& points, int width, vector<size_t> inflections, vector<size_t> defects, bool draw_tangents) {
    auto comp_x = [](Vec2d a, Vec2d b) {return a.x < b.x;};
    auto comp_y = [](Vec2d a, Vec2d b) {return a.y < b.y;};
    Vec2d max_x = *max_element(points.begin(), points.end(), comp_x);
    Vec2d max_y = *max_element(points.begin(), points.end(), comp_y);
    Vec2d min_x = *min_element(points.begin(), points.end(), comp_x);
    Vec2d min_y = *min_element(points.begin(), points.end(), comp_y);
    double scale = double(width) / (max_x.x - min_x.x);
    int height = ceil(scale * (max_y.y - min_y.y));
    // Function converts x,y values to pixel opencv point representing pixel location
    // Also takes into account that positive Y points downward in graphics
    auto convert_coord = [scale, min_x, min_y, max_x, max_y](Vec2d pt) {
                return cv::Point((pt.x - min_x.x) * scale,
                                scale * (max_y.y - pt.y));};
    // Create image matrix with dimensions to fill width, and proportional height
    cv::Mat out_img(height, width, CV_8UC3, cv::Scalar(0));
    cv::Scalar white = cv::Scalar(255, 255, 255);
    cv::Scalar red = cv::Scalar(0, 0, 255);
    cv::Scalar blue = cv::Scalar(255, 80, 80);
    cv::Scalar green = cv::Scalar(0, 255, 0);


    Vec2d last_vec = points[points.size() - 1];
    cv::Point last_pt = convert_coord(last_vec);
    for (Vec2d p : points) {
        cv::Point this_pt = convert_coord(p);
        cv::line(out_img, last_pt, this_pt, white);
        last_pt = this_pt;
    }

    for( size_t idx : defects )
       {
       Vec2d p = points[idx];
       cv::Point defect_pt = convert_coord(p);
       cv::circle(out_img, defect_pt, 5, green);
       }

    for (size_t idx : inflections) {
        Vec2d p = points[idx];
        cv::Point infl_pt = convert_coord(p);
        cv::circle(out_img, infl_pt, 10, white);
        cv::putText(out_img, to_string(idx), infl_pt + cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, white);
        if (draw_tangents) {
            Vec2d tangent = find_tangent_angle(idx, points);
            Vec2d scaled_tan = 40.0 * tangent.normalized();
            cv::Point2i tan_as_pt = cv::Point2i(scaled_tan.x, -scaled_tan.y);
            cv::line(out_img, infl_pt, infl_pt + tan_as_pt, red);
            cv::line(out_img, infl_pt, infl_pt - tan_as_pt, red);
        }
    }

    // Just for testing
    
    Vec2d first_tan = find_tangent_angle(inflections[0], points);
    Vec2d last_tan = find_tangent_angle(inflections[inflections.size() - 1], points);
    Vec2d first_point = points[inflections[0]];
    Vec2d last_point = points[inflections[inflections.size() - 1]];
    Vec2d intersection_pt;
    bool intersect = intersect_lines(first_tan, last_tan, first_point, last_point, intersection_pt);
    if (intersect) {cv::circle(out_img, convert_coord(intersection_pt), 10, blue);}
    
    return out_img;
}


vector<double> calc_curvature(const vector<Vec2d>& points) {
    size_t N = points.size();
    vector<Vec2d> der_1(points.size());
    vector<Vec2d> der_2(points.size());
    der_1[0] = points[0] - points[N-1];
    for (size_t i = 1; i < N; ++i) {
        der_1[i] = points[i] - points[i-1];
    }
    der_2[0] = der_1[0] - der_1[N-1];
    for (size_t i = 1; i < N; ++i) {
        der_2[i] = der_1[i] - der_1[i-1];
    }
    vector<double> curvature(points.size());
    for (size_t i = 0; i < N; ++i) {
        Vec2d d1 = der_1[i];
        Vec2d d2 = der_2[i];
        double numerator = d1.x * d2.y - d1.y * d2.x;
        double denom = pow(d1.x*d1.x + d1.y*d1.y, 1.5);
        if (denom != 0) {
            curvature[i] = numerator / denom;
        } else {curvature[i] = 0;}
    }
    return curvature;
}

double vec_angle_diff(Vec2d a, Vec2d b) {
    return acos((a * b) / (a.norm() * b.norm()));
}

vector<size_t> find_inflections(vector<Vec2d > points, double threshold) {
    size_t N = points.size();
    if (N < 3) {
        return vector<size_t>();
    }
    auto curvatures = calc_curvature(points);
    vector<size_t> infl_indices;

    int last_sign = sign(curvatures[0]);
    bool infl_valid = false;
    Vec2d last_vec = points[0] - points[N-1];
    for (size_t i = 1; i < curvatures.size(); ++i) {
        int this_sign = sign(curvatures[i]);
        Vec2d this_vec = points[i] - points[i-1];
        double angle_diff = vec_angle_diff(this_vec, last_vec);
        if (abs(angle_diff) >= threshold) {
            infl_valid = true;
        }
        if (this_sign != last_sign) {
            last_sign = this_sign;
            if (infl_valid) { // True inflection point
                infl_indices.push_back(i - 1);
                last_vec = this_vec;
                infl_valid = false;
            }
            else {
                continue;
            }
        }
    }
    return infl_indices;
}
