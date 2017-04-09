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
    cv::Mat img = draw_curve(points, 480, infl_indices, true);
    cv::namedWindow("AWESOME", cv::WINDOW_AUTOSIZE);
    cv::imshow("AWESOME", img);
    cv::waitKey(0);

   }

void test_pieces(void)
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
   unsigned int j;
     for( j = 0; j < pieces.size(); j++ )
      {
      printf("Piece %d has %lu points\n", j, pieces[j].points.size());
      fflush(stdout);
      infl_indices = find_inflections(pieces[j].points);
      cv::Mat output = draw_curve(pieces[j].points, 480, infl_indices, true);

      std::string name = win_name + std::to_string(j);
      cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
      cv::imshow(name, output);
      cv::waitKey(0);
      cv::destroyWindow(name );
      infl_indices.clear();
      infl_points.clear();
      }


   }

void test_show_img(void)
   {
   cv::Mat img = cv::imread( "../../images/rows/row1.png", 1 );

   std::vector<Piece> pieces = find_pieces( img );
   cv::Mat output(img.size(), CV_8UC1, cv::Scalar(255,255,255) );

   std::cout << "Found " << pieces.size() << " pieces." << std::endl;
   std::cout << "Printing " << pieces[0].contour.size() << " points to screen." << std::endl;

   cv::Scalar color( 200, 200, 200 );
   cv::drawContours( output, std::vector<std::vector<cv::Point>>(1, pieces[0].contour), -1, color, CV_FILLED);


   std::string test_win = "Test";
   cv::namedWindow( test_win, CV_WINDOW_NORMAL );
   cv::imshow( test_win, output );
   cv::resizeWindow( test_win, 400, 800 );
   cv::imwrite("../../images/out/median_filter.png", output );
   cv::waitKey(0);
   }

int main()
   {

   test_pieces();

   return 0;
};
