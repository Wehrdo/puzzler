
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
#include "../analysis/edge_tools.hpp"


/**
* Given an image with one or more pieces,
* Find individual pieces, extract their points,
* and store in a vector of piece types.
*/
std::vector<Piece> find_pieces( cv::Mat img )
   {

   // Vector of edges of objects
   std::vector<std::vector<cv::Point>> contours;
   std::vector<Piece> pieces;
   double threshold = 10000.0;
   double size_factor = 2.0;

   // convert to greyscale
   cvtColor( img, img, CV_BGR2GRAY );
   img = img > 100;


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
   findContours( img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );

   int i;
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
      if( contourArea(contours[i]) < (average_size*size_factor) )
         {
         std::vector<Vec2d> points;
         for( int j = 0; j < contours[i].size(); j++ )
            {
            points.push_back(Vec2d((contours[i])[j].x, (contours[i][j].y)));
            }
         pieces.push_back(Piece(points));
         }
      }

   return pieces;
   }
