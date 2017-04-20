#ifndef EDGE_H
#define EDGE_H

#include <opencv2/opencv.hpp>
#include "curve.hpp"
#include "piece.hpp"
#include <vector>

class Edge
   {
   public:
      cv::Point origin;
      cv::Point handle;
      std::vector<cv::Point> points;
      std::vector<Curve::curve_type> types;

      Edge( void );
      Edge( Piece piece, std::vector<Curve> curves );

      void draw( void );

      // Compare with another edge
      float compare(const Edge& that );
   };

#endif /* EDGE_H */
