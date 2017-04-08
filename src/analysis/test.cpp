#include "piece.hpp"
#include "find_pieces.hpp"
#include "edge_tools.hpp"

#include <string>
#include <vector>
#include <cstdio>
#include <cmath>
#include <functional>
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

int test_function(void)
   {




    int N = 48;
    std::vector<Vec2d> points = gen_curve(N, -1.25, 1.4, polynomial);
    auto infl_indices = find_inflections(points);
    std::vector<Vec2d> infl_points(infl_indices.size());
    for (int i = 0; i < infl_indices.size(); ++i) {
        infl_points[i] = points[infl_indices[i]];
    }
    cv::Mat img = draw_curve(points, 480, infl_points);
    cv::namedWindow("AWESOME", cv::WINDOW_AUTOSIZE);
    cv::imshow("AWESOME", img);
    cv::waitKey(0);

   }

int test_pieces(void)
   {
   std::vector<Piece> pieces;

   cv::Mat img = cv::imread( "../../images/rows/row1_shrunk.png", 1 );

   // Find pieces
   pieces = find_pieces( img );

   std::cout << "Found " << pieces.size() << " pieces." << std::endl;

   // Print found pieces to screen
   std::vector<std::size_t> infl_indices;
   std::vector<Vec2d> infl_points;
   std::string win_name = "Window ";
     for( int j = 0; j < pieces.size(); j++ )
      {
      //      int j = 1;
      // infl_indices = find_inflections(pieces[j].points);
      //      infl_points = std::vector<Vec2d>(infl_indices.size());
      // for (int i = 0; i < infl_indices.size(); ++i)
      //    {
      //    infl_points[i] = pieces[j].points[infl_indices[i]];
      //    }
      cv::Mat output = draw_curve(pieces[j].points, 480, infl_points);

      std::string name = win_name + std::to_string(j);
      cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
      cv::imshow(name, output);
      cv::waitKey(0);
      cv::destroyWindow(name );
      infl_indices.clear();
      infl_points.clear();
      }


   }

int main(int argc, char* argv[])
   {

   test_pieces();

};
