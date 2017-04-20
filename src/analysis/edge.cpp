#include "edge.hpp"
#include <math.h>
#include <iostream>
#include <algorithm>
#include <opencv2/surface_matching/icp.hpp>
#define RAD_TO_DEG (180.0 / M_PI)

using namespace std;
using namespace cv;

Edge::Edge() {}

Edge::Edge(Piece piece, vector<Curve> curves)
{
    this->origin = curves[0].origin;

    if (curves.size() >= 1)
        this->handle = curves[curves.size() - 1].origin;

    size_t idx = curves[0].start;
    do
       {
       if( idx == piece.contour.size() )
          idx = 0;
       this->points.push_back(piece.contour[idx]);
       idx++;
       }
    while(idx != curves[curves.size()-1].end);

    // this->points = vector<Point>(
    //     &(piece.contour[curves[0].start]),
    //     &(piece.contour[curves[curves.size() - 1].end]));

    for (Curve curve : curves)
        this->types.push_back(curve.type);
}

void Edge::draw(void)
   {
   cv::namedWindow("Edge");
   cv::imshow("augh", draw_curve(points, 480));
   cv::waitKey(0);

   }

void Edge::translate(void)
{
    // Translate wrt origin
    float delta_x = 0 - origin.x;
    float delta_y = 0 - origin.y;
    for (Point pt : points)
    {
        pt.x += delta_x;
        pt.y += delta_y;
    }
    handle.x += delta_x;
    handle.y += delta_y;
}

void Edge::rotate(Point that)
{
    // Rotate wrt point and handle

    float angle = this->handle.dot(that);

    float len1 = sqrt(this->handle.x * this->handle.x + this->handle.y * this->handle.y);
    float len2 = sqrt(that.x * that.x + that.y * that.y);

    float a = this->handle.dot(that) / (len1 * len2);

    angle = (a >= 0 ? 1.0 : -1.0) * acos(a);

    Mat transformation = getRotationMatrix2D(this->origin, angle, 1);
    transform(this->points, this->points, transformation);
}

// Takes a set of 2D points and fills a pre-allocated Nx6 array
// such that the first 3 columns contain the x, y, z, coordinates of the points
// and the last 3 columns contain the x, y, z coordinates of the normal vectors to the points
void to_3d_set(const vector<Point>& points, Mat &out_mat) {
    Mat last_pt(1, 3, CV_32F);
    // Vector pointing directly up
    Mat up_vec = (Mat_<float>(1, 3, CV_32F) << 0, 0, -1);
    for (size_t i = 0; i < points.size(); ++i) {
        // This point as a matrix
        Mat p = (Mat_<float>(1, 3, CV_32F) << (float)points[i].x, (float)points[i].y, 0);
        // Direction of the edge from the last vertex to this vertex
        Mat dir = p - last_pt;
        // Normal vertex, pointing outwards
        Mat normal = dir.cross(up_vec);
        normal = normal / norm(normal);
        // Copy point to matrix
        p.copyTo(out_mat.row(i).colRange(0, 3));
        // Copy normal vector in
        normal.copyTo(out_mat.row(i).colRange(3, 6));    

        last_pt = p;
    }
    // Duplicate second vector's normal to first vector's normal
    (out_mat.row(1).colRange(3, 6)).copyTo(out_mat.row(0).colRange(3, 6));
}

float Edge::compare(const Edge &that)
{
    // How much to translate the points by. Moves "that" to the origin of "this"
    Point translate_amt = origin - that.origin;
    // Find how much to rotate that by after translation so origins match
    double this_angle = atan2(origin.y - handle.y, handle.x - origin.x);
    double that_angle = atan2(that.origin.y - that.handle.y, that.handle.x - that.origin.x);
    double amt_to_rotate = this_angle - that_angle;
    // Matrix to rotate "that" about its origin by the correct amount
    Mat rot_mat = getRotationMatrix2D(Point(0, 0), amt_to_rotate * RAD_TO_DEG, 1);
    // Matrix to translate points
    double data[2][3] = {{1, 0, (double)translate_amt.x}, {0, 1, (double)translate_amt.y}};
    Mat trans_mat(2, 3, CV_64F, data);

    // Translate then rotate
    vector<Point> moved_pts;
    transform(that.points, moved_pts, trans_mat);
    transform(moved_pts, moved_pts, rot_mat);
    
    // Copy and reverse these points
    vector<Point> static_pts(points.begin(), points.end());
    reverse(static_pts.begin(), static_pts.end());

    Mat a6(static_pts.size(), 6, CV_32F);
    Mat b6(moved_pts.size(), 6, CV_32F);
    to_3d_set(static_pts, a6);
    to_3d_set(moved_pts, b6);

    cv::ppf_match_3d::ICP icp(100, 0.1, 2.5, 4);
    double pose[16];
    double error;
    int failure = 0;
    failure = icp.registerModelToScene(b6, a6, error, pose);
    if (failure) {
        cout << "Failed to find pose" << endl;
    }
    Mat pose4(4, 4, CV_64F, pose);
    

    Mat opt_tran(2, 3, CV_64F);
    pose4.rowRange(0, 2).colRange(0, 2).copyTo(opt_tran.colRange(0, 2));
    pose4.rowRange(0, 2).colRange(3, 4).copyTo(opt_tran.colRange(2, 3));

    // transform(moved_pts, moved_pts, opt_tran);
    vector<Point> all_pts(static_pts);
    all_pts.insert(all_pts.end(), moved_pts.begin(), moved_pts.end());
    Mat moved_curve = draw_curve(all_pts, 480);
    namedWindow("itsawindow");
    imshow("itsawindow", moved_curve);
    waitKey(0);

    cout << "Error = " << error << endl;
    cout << "pose4 = " << endl << pose4 << endl;
    cout << "opt_tran = " << endl << opt_tran << endl;
    return 0;
}
