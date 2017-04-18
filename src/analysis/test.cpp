#include "piece.hpp"
#include "edge.hpp"
#include "find_pieces.hpp"
#include "edge_tools.hpp"
#include "user_gui.hpp"

#include <string>
#include <vector>
#include <cstdio>
#include <cmath>
#include <functional>
#include <opencv2/opencv.hpp>


Piece piece_to_fake( Piece input, size_t start_idx, size_t end_idx )
   {
   // working points
   cv::Point2f start = input.contour[start_idx];
   cv::Point2f end = input.contour[end_idx];
   cv::Point2f corner;

   // length of corner we are creating
   float length = sqrt( pow(start.x - end.x, 2) + pow(start.y - end.y, 2 ) ) / sqrt(2);

   // Starting point, along X axis with offset
   corner.x = start.x + length;
   corner.y = start.y;

   // Equation of line
   float slope = (end.y - start.y)/(end.x - start.x);
   float inter = start.y - slope*start.x;

   // rotate 45 degrees from existing line
   float angle = -45.0;
   float y = slope*corner.x + inter;
   float side_part = sqrt( pow(start.x - corner.x, 2) + pow(start.y - y, 2));

   // rotate be amount existing line is from normal
   angle -= acos(length/side_part)*180.0/M_PI;
   cv::Mat trans = cv::getRotationMatrix2D( start, angle, 1 );
   std::vector<cv::Point> to_transform(1, corner );
   std::vector<cv::Point> transformed;
   cv::transform( to_transform, transformed, trans );

   Piece to_return;

   // Copy over selected points
   to_return.contour = std::vector<cv::Point>( &(input.contour[start_idx]), &(input.contour[end_idx]) );
   to_return.points = std::vector<Vec2d>( &(input.points[start_idx]), &(input.points[end_idx]) );

   // Add new point
   to_return.contour.push_back( transformed[0] );
   to_return.points.push_back( Vec2d(transformed[0].x, transformed[0].y ) );


   // Find inflection points, process
   std::vector<size_t> infl_idx = find_inflections( fake.points );
   fake.set_inflection( infl_idx );
   fake.process();

   return to_return;

   }

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

   // Pieces to process
   cv::Mat row1 = cv::imread( "../../images/rows/row1_shrunk.png", 1 );
   cv::Mat row2 = cv::imread( "../../images/rows/row2_shrunk.png", 1 );
   cv::Mat row3 = cv::imread( "../../images/rows/row3_shrunk.png", 1 );
   cv::Mat row4 = cv::imread( "../../images/rows/row4_shrunk.png", 1 );

   // Pieces matching to
   cv::Mat test_img = cv::imread( "../../images/pieces/test_1_2.png", 1 );

   // Find pieces
   std::vector<Piece> found = find_pieces( row1 );
   pieces.insert( pieces.end(), found.begin(), found.end() );
   found = find_pieces( row2 );
   pieces.insert( pieces.end(), found.begin(), found.end() );
   found = find_pieces( row3 );
   pieces.insert( pieces.end(), found.begin(), found.end() );
   found = find_pieces( row4 );
   pieces.insert( pieces.end(), found.begin(), found.end() );

   // Take in image that we are matching to
   std::vector<Piece> match_to_vec = find_pieces( test_img );
   Piece match_to = match_to_vec[0];

   std::cout << "Found " << pieces.size() << " pieces." << std::endl;
   PuzzleGUI gui("User GUI");
   std::vector<std::size_t> infl_indices;
   std::vector<Vec2d> infl_points;


   std::string win_name = "Piece ";
   unsigned int i;
   for( i = 0; i < pieces.size(); i++ )
      {
      Piece processing = pieces[i];

      // Find inflection points and process pieces
      infl_indices = find_inflections(processing.points, ((M_PI / 180) * 40));
      processing.set_inflection( infl_indices );
      processing.process();

      auto straight_lines = find_straight_sides(processing.points, ((M_PI / 180) * 0.5));

      // draw to matrix
      cv::Mat output = draw_curve(processing.points, 480, infl_indices, processing.defect_index, straight_lines, true);


      // Show to screen
      // std::string name = win_name + std::to_string(i);
      // cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
      // cv::imshow(name, output);
      // cv::waitKey(0);
      // cv::destroyWindow(name );@

      // Process the edge we are testing for

      //processing.draw( 480 );
      //std::cout << "Showing image " << i << std::endl;
      //cv::waitKey(0);
      }

   // // Prompt user for image
   std::pair<size_t, size_t> selection = gui.select_edge( match_to );

   // // Extract first and last points from selected region
   size_t start_idx = std::get<0>(selection);
   size_t end_idx = std::get<1>(selection);

   Piece fake = piece_to_fake( match_to, start_idx, end_idx );

   fake.draw( 480 );
   cv::waitKey(0);


   // // Create mocked up edge
   // Edge match_edge;
   // match_edge.origin = cv::Point( 389.761, 707.396 );
   // match_edge.handle = cv::Point( 541.796, 1003.53 );
   // match_edge.points = std::vector<cv::Point>(&(match_to.contour[start_idx]), &(match_to.contour[end_idx]) );
   // match_edge.types.push_back( Curve::outdent );
   // match_edge.types.push_back( Curve::indent );

   // Edge should_match;
   // should_match.origin = cv::Point( 880.307, 1970.96 );
   // should_match.handle = cv::Point( 995.534, 2282.31 );
   // std::vector<cv::Point> to_add_1( &(pieces[6].contour[65]), &(pieces[6].contour[110]));
   // std::vector<cv::Point> to_add_2( &(pieces[6].contour[0]), &(pieces[6].contour[7]));
   // std::vector<cv::Point> to_add;
   // to_add.insert(to_add.end(), to_add_1.begin(), to_add_1.end() );
   // to_add.insert(to_add.end(), to_add_2.begin(), to_add_2.end() );
   // should_match.points = to_add;
   // should_match.types.push_back( Curve::outdent );
   // should_match.types.push_back( Curve::indent );

   }



int main()
   {
   cv::Mat img = cv::imread( "../../images/rows/row1.png", 1 );

   test_pieces();

   return 0;
};
