#ifndef EDGE_H
#define EDGE_H

#include "curve.hpp"

class Edge
   {
   public:
      cv::Point origin;
      cv::Point handle;
      std::vector<cv::Point> points;
      std::vector<Curve::curve_type> types;

      Edge( void )
         {

         }

      Edge( Piece piece, std::vector<Curve> curves )
         {
         this->origin = curves[0].origin;

         if( curves.size() >= 1 )
            this->handle = curves[curves.size()-1].origin;

         this->points = std::vector<cv::Point>(
            &(piece.contour[curves[0].start]),
            &(piece.contour[curves[curves.size()-1].end]) );

         for( Curve curve : curves )
            this->types.push_back( curve.type );
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

         float angle = this->handle.dot(that);

         float len1 = sqrt(this->handle.x * this->handle.x + this->handle.y * this->handle.y);
         float len2 = sqrt(that.x * that.x + that.y * that.y);

         float a = this->handle.dot(that) / (len1 * len2);

         angle = (a >= 0 ? 1.0 : -1.0) * acos(a);

         cv::Mat transformation = cv::getRotationMatrix2D(this->origin,angle,1);
         cv::transform(this->points,this->points,transformation);
         }

      float compare( Edge that )
         {
         // Compare with another edge
         }

   };

#endif /* EDGE_H */
