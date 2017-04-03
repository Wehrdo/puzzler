#include <cstdint>
#include <vector>
#include <cmath>
#include <cstdio>

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
    Vec2<T> operator-(const Vec2<T>& other) {
        Vec2<T> difference;
        difference.x = this->x - other.x;
        difference.y = this->y - other.y;
        return difference;
    };
    // Dot product
    T operator*(const Vec2<T>& other) {
        return (this->x * other.x) + (this->y * other.y);
    };
    T norm();
};



template <typename T>
T Vec2<T>::norm( ){
    return sqrt(*this * *this);
}

// double vector
typedef Vec2<double> Vec2d;

template <typename T>
double calc_angle(Vec2<T> a, Vec2<T> b) {
    T y_diff = b.y - a.y;
    T x_diff = b.x - a.x;
    return atan2(y_diff, x_diff);
};

// Returns indices of inflection points
template <typename T>
std::vector<std::size_t> find_inflections(std::vector<Vec2<T> > points, double threshold=10) {
    std::size_t N = points.size();
    if (N < 3) {
        return std::vector<std::size_t>();
    }
    double last_angle = calc_angle(points[0] - points[N-1], Vec2d(1,0));
    std::vector<std::size_t> inflections;
    for (std::size_t i = 0; i < N; ++i) {
        Vec2<T> new_vec = points[i] - points[i-1];
        // double angle = calc_angle(new_vec, last_vec);
        double raw_angle = calc_angle(new_vec, Vec2d(1,0));
        double angle_diff = raw_angle - last_angle;
        printf("raw: %f, diff: %f\n", raw_angle, angle_diff);
        last_angle = raw_angle;
    }
    return inflections;
};
