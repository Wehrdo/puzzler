#ifndef EDGE_TOOLS_H
#define EDGE_TOOLS_H

#include <cstdint>
#include <vector>
#include <cmath>
#include <cstdio>
#include <opencv2/opencv.hpp>

#define ANGLE_THRESHOLD ((M_PI / 180) * 40)

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
    Vec2<T> operator*(const T factor) {
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
Vec2<T> operator*(const T factor, const Vec2<T>& vec) {
    return Vec2<T>(vec.x * factor, vec.y * factor);
}


template <typename T>
T Vec2<T>::norm( ){
    return sqrt(*this * *this);
}

template <typename T>
Vec2<T> Vec2<T>::normalized() {
    Vec2<T> output;
    T norm_val = this->norm();
    output.x = this->x / norm_val;
    output.y = this->y / norm_val;
    return output;
}

/* v1 and v2 are the directions of the two lines
 * o1 and o2 are points that exist on line 1 and line 2, respectively
 * (Can also consider it as the origin of v1 and v2)
 * Populates intersection_pt with the intersection.
 * Returns false if lines don't intersect, true otherwise
 */
template <typename T>
bool intersect_lines(Vec2<T> v1, Vec2<T> v2, Vec2<T> o1, Vec2<T> o2, Vec2<T>& intersection_pt) {
    /* Parametric equation of a line
    p1 = o1 + v1 * u
    p2 = o2 + v2 * v
    Set p1 == p2, and solve for either u or v.
    Then put the solution into one of the line equations, and the result
    is the intersection
    */
    T denom = (v2.x * v1.y - v2.y * v1.x);
    if (denom != 0) {
        Vec2<T> o_diff = o2 - o1;
        T numerator = (o_diff.y * v2.x - o_diff.x * v2.y);
        T u = numerator / denom;
        intersection_pt = u * v1 + o1;
        return true;
    } else {
        return false;
    }
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


cv::Mat draw_curve(const std::vector<Vec2d>& points, int width, std::vector<size_t> inflections, bool draw_tangents);

#endif
