#ifndef PIECE_H
#define PIECE_H

#include "edge_tools.hpp"
#include "curve.hpp"

// to keep contours, can remove after testing
#include "opencv2/imgproc/imgproc.hpp"

class Piece
   {
   public:

      // Member variables
      std::vector<Vec2d> points;
      std::vector<cv::Point> contour;
      std::vector<int> hull_index;
      std::vector<size_t> defect_index;
      std::vector<size_t> inflection_index;
      std::vector<Curve> curves;

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

      //TODO: Move inflection point calc here
      //TODO: Move draw function here

      void find_indents( void );

   };


// Inflection point code here?

#endif /* PIECE_H */
