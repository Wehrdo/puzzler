#include "piece.hpp"
#include "find_pieces.hpp"

#include <vector>
#include <cstdio>

int main( int argc, char* argv [] )
   {

   std::vector<Piece> pieces;

   cv::Mat img = cv::imread( "../../images/rows/row1.png", 1 );

   // Find pieces
   pieces = find_pieces( img );

   std::cout << "Found " << pieces.size() << " pieces." << std::endl;

   cv::waitKey(0);
   return 0;
   }
