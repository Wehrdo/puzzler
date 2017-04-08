#ifndef PIECE_H
#define PIECE_H

#include "edge_tools.hpp"

// to keep contours, can remove after testing
#include "opencv2/imgproc/imgproc.hpp"

class Piece
   {
   public:
      std::vector<Vec2d> points;
      std::vector<cv::Point> contour;
      Piece()
         {

         }

      Piece( std::vector<Vec2d> points )
         {
         this->points = points;
         }

   };

// Inflection point code here?

#endif /* PIECE_H */
