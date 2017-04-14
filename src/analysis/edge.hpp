#ifndef EDGE_H
#define EDGE_H

#include "curve.hpp"

class Edge
   {
   public:
      cv::Point origin;
      cv::Point handle;
      std::vector<cv::Point> points;
      std::vector<curve_type> types;

      Edge( Piece piece, std::vector<Curve> curves )
         {
         this->origin = curves[0].origin;

         if( curves.size() >= 1 )
            this->handle = curves[1].origin;

         this->points = std::vector<cv::Point>(
            &(piece.contour[curves[0].start]),
            &(piece.contour[curves[curves.size()-1].end]) );

         this->types = curves.type;
         }


      void translate( void )
         {
         // Translate wrt origin
         float delta_x = 0 - origin.x;
         float delta_y = 0 - origin.y;
         for( cv::Point pt : points )
            {
            pt.x += delta_x;
            pt.y += delta_y;
            }
         handle.x += delta_x;
         handle.y += delta_y;
         }

      void rotate( cv::Point that )
         {
         // Rotate wrt point and handle

         angle = this.dot(that)

         float len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
         float len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

         float a = this.dot(that) / (len1 * len2);

         angle = (a >= 0 ? 1.0 : -1.0) * acos(a);

         cv::Mat transformation = cv::getRotationMatrix2D(this.origin,angl,1);
         cv::warpAffine(this.points,this.points,transformation, this.points.size());
         }

      float compare( Edge that )
         {
         // Compare with another edge
         }

   }

#endif /* EDGE_H */
