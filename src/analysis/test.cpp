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

void test_function(void)
   {

    int N = 48;
    std::vector<Vec2d> points = gen_curve(N, -1.25, 1.4, polynomial);
    auto infl_indices = find_inflections(points);
    // cv::Mat img = draw_curve(points, 480, infl_indices, std::vector<size_t>(), true);
    // cv::namedWindow("AWESOME", cv::WINDOW_AUTOSIZE);
    // cv::imshow("AWESOME", img);
    // cv::waitKey(0);

   }

void test_pieces(void)
   {
   std::vector<Piece> pieces;

   cv::Mat img = cv::imread( "../../images/rows/row1_shrunk.png", 1 );

   // Find pieces
   pieces = find_pieces( img );
   std::cout << "Found " << pieces.size() << " pieces." << std::endl;


   std::vector<std::size_t> infl_indices;
   std::vector<Vec2d> infl_points;

   std::string win_name = "Piece ";
   unsigned int i;
   for( i = 0; i < pieces.size(); i++ )
      {
      Piece processing = pieces[i];

      printf("Piece %d has %lu points\n", i, processing.points.size());
      fflush(stdout);

      // find convex hull
      processing.process_cvx_hull();
      std::cout << "Piece " << i << " has " << processing.hull_index.size() << " points in the convex hull." << std::endl;

      infl_indices = find_inflections(processing.points);
      std::cout << "Piece " << i << " has " << infl_indices.size() << " inflection points." << std::endl;

      auto straight_lines = find_straight_sides(processing.points, ((M_PI / 180) * 0.5));

      // draw to matrix
      cv::Mat output = draw_curve(processing.points, 480, infl_indices, processing.defect_index, straight_lines, true);


      // Show to screen
      std::string name = win_name + std::to_string(i);
      cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
      cv::imshow(name, output);
      cv::waitKey(0);
      cv::destroyWindow(name );
      }
   }



int main()
   {
   cv::Mat img = cv::imread( "../../images/rows/row1.png", 1 );

   test_pieces();
   
   return 0;
};
