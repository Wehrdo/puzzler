
// openCV includes
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>

// System includes
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <vector>

// Local includes
#include "piece.hpp"
#include "edge_tools.hpp"

/**
* MORPH_RECT == 0
* MORPH_CROSS == 1
* MORPH_ELLIPSE == 2
*/
void erode( int type, int size, cv::Mat img )
{
cv::Mat element = cv::getStructuringElement( type, cv::Size( 2*size + 1, 2*size+1 ), cv::Point( size, size ) );

  /// Apply the erosion operation
  cv::erode( img, img, element );

}

/**
* MORPH_RECT == 0
* MORPH_CROSS == 1
* MORPH_ELLIPSE == 2
*/
void dilate( int type, int size, cv::Mat img )
   {
   cv::Mat element = cv::getStructuringElement( type, cv::Size( 2*size + 1, 2*size+1 ), cv::Point( size, size ) );

  /// Apply the erosion operation
  cv::dilate( img, img, element );
}



/**
* Given an image with one or more pieces,
* Find individual pieces, extract their points,
* and store in a vector of piece types.
*/
std::vector<Piece> find_pieces( cv::Mat img, std::vector<Piece> &partials )
   {

   // Vector of edges of objects
   std::vector<std::vector<cv::Point>> contours;
   std::vector<Piece> pieces;
   double threshold = 10000.0;
   double size_factor = 2.0;

   cv::Mat original_image;
   img.copyTo(original_image);
   // convert to greyscale
   cvtColor( img, img, CV_BGR2GRAY );
   img = img > 80;

   // apply median filter
   dilate(1, 5, img);
   erode(1, 5, img );
   cv::medianBlur(img, img, 25);



   // CV_CHAIN_APPROX_NONE stores absolutely all the contour points.
   //  That is, any 2 subsequent points (x1,y1) and (x2,y2) of the contour
   //  will be either horizontal, vertical or diagonal neighbors,
   //  that is, max(abs(x1-x2),abs(y2-y1))==1.
   // CV_CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and
   //  diagonal segments and leaves only their end points. For
   //  example, an up-right rectangular contour is encoded with 4 points.
   // CV_CHAIN_APPROX_TC89_L1,CV_CHAIN_APPROX_TC89_KCOS applies one of
   //  the flavors of the Teh-Chin chain approximation algorithm.

   // outline edges in points, store in vector
   findContours( img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );

   unsigned int i;
   double average_size = 0.0;
   double contour_area = 0.0;

   // Cycle through, skip/remove objects that are too small;
   for( i = 0; i < contours.size(); i++ )
      {
      contour_area = contourArea(contours[i]);
      if( contour_area < threshold )
         {
         contours.erase(contours.begin() + i);
         i--;
         continue;
         }
      average_size += contour_area;
      }
   average_size /= i;

   // Cycle through remaining edges, add average-sized objects to output
   for( i = 0; i < contours.size(); i++ )
      {
      Piece to_add;
      to_add.raw_image = original_image;

      cv::approxPolyDP(contours[i], to_add.contour, 1, true );
      unsigned int j;
      for( j = 0; j < to_add.contour.size(); j++ )
         {
         to_add.points.push_back(Vec2d(to_add.contour[j].x, img.rows -  to_add.contour[j].y));
         }
      if( contourArea(contours[i]) < (average_size*size_factor) )
         {
         pieces.push_back(to_add);
         }
      else
         {
         partials.push_back(to_add);
         }
      }

   return pieces;
   }
