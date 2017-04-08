#ifndef PIECE_H
#define PIECE_H

#include "edge_tools.hpp"

// to keep contours, can remove after testing
#include "opencv2/imgproc/imgproc.hpp"

class Piece
   {
   public:

      // Member variables
      std::vector<Vec2d> points;
      std::vector<cv::Point> contour;
      std::vector<int> cvx_hull_max_dist;

      // Constructors
      Piece()
         {

         }

      Piece( std::vector<Vec2d> points )
         {
         this->points = points;
         }

      // Member functions
      void process_cvx_hull();

      // TODO: Should make a getter function, would generate points in-place...


   };


// Inflection point code here?

#endif /* PIECE_H */
