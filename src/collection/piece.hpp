#ifndef PIECE_H
#define PIECE_H

#include "../analysis/edge_tools.hpp"

class Piece
   {
   public:
      std::vector<Vec2d> points;

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
