#ifndef EDGE_TOOLS_H
#define EDGE_TOOLS_H

#include <cstdint>
#include <vector>
#include <cmath>
#include <cstdio>
#include <opencv2/opencv.hpp>

#define ANGLE_THRESHOLD ((M_PI / 180) * 10)

template <typename T>
class Vec2 {
public:
    T x, y;
    Vec2() {
        x = y = 0;
    }
    Vec2(T new_x, T new_y) {
        x = new_x;
        y = new_y;
    };
    Vec2<T> operator+(const Vec2<T>& other) {
        Vec2<T> sum;
        sum.x = this->x + other.x;
        sum.y = this->y + other.y;
        return sum;
    };
    const Vec2<T> operator-(const Vec2<T>& other) const {
        Vec2<T> difference;
        difference.x = this->x - other.x;
        difference.y = this->y - other.y;
        return difference;
    };
    // Dot product
    T operator*(const Vec2<T>& other) {
        return (this->x * other.x) + (this->y * other.y);
    };
    // Scalar multiply
    T operator*(const T factor) {
        return Vec2<T>(this->x * factor, this->y * factor);
    }
    // Scalar divide
    T operator/(const T factor) {
        return Vec2<T>(this->x / factor, this->y / factor);
    }
    T norm();
    Vec2<T> normalized();
};



template <typename T>
T Vec2<T>::norm( ){
    return sqrt(*this * *this);
}

template <typename T>
Vec2<T> Vec2<T>::normalized() {
    Vec2<T> output;
    T norm_val = this->norm();
    output.x /= norm_val;
    output.y /= norm_val;
    return output;
}

// double vector
typedef Vec2<double> Vec2d;

template <typename T>
double calc_angle(Vec2<T> a, Vec2<T> b) {
    T y_diff = b.y - a.y;
    T x_diff = b.x - a.x;
    return atan2(y_diff, x_diff);
};

template <typename T>
int sign(T val) {return val < 0;}

// Returns indices of inflection points
std::vector<std::size_t> find_inflections(std::vector<Vec2d > points, double threshold=ANGLE_THRESHOLD);


cv::Mat draw_curve(const std::vector<Vec2d>& points, int width, std::vector<Vec2d> inflections);

#endif
