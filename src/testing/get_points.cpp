#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

int main( int argc, char* argv[] )
   {
   // is user dumb
   if( argc != 2 )
      {
      cout << "Please include image." << endl;
      exit(-1);
      }

   // read in image
   Mat src = imread(argv[1], 1 );

   // convert to BW
   cvtColor( src, src, CV_BGR2GRAY );
   src = src > 100;

   // Find contours
   Mat temp = src.clone();
   vector<vector<Point>> contours;
   vector<Vec4i> hierarchy;
   double contour_area;
   double threshold = 5;
   vector<int> too_small;

   // CV_CHAIN_APPROX_NONE stores absolutely all the contour points.
   //  That is, any 2 subsequent points (x1,y1) and (x2,y2) of the contour
   //  will be either horizontal, vertical or diagonal neighbors,
   //  that is, max(abs(x1-x2),abs(y2-y1))==1.
   // CV_CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and
   //  diagonal segments and leaves only their end points. For
   //  example, an up-right rectangular contour is encoded with 4 points.
   // CV_CHAIN_APPROX_TC89_L1,CV_CHAIN_APPROX_TC89_KCOS applies one of
   //  the flavors of the Teh-Chin chain approximation algorithm.
   cout << "Finding contours" << endl;
   findContours(temp, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

   Mat output(src.size(), CV_BW, Scalar(255,255,255);
   // find small contours
   cout << "Finding large contours" << endl;
   for(int i = 0; i < contours.size(); i++ )
      {
      contour_area = contourArea(contours[i]);
      if(contour_area > threshold)
         {
         drawContours(output, contours, i, Scalar(0), CV_FILLED, 8 );
         }
      }


   // Display image
   char* test_win = "Test";
   namedWindow( test_win, CV_WINDOW_NORMAL );
   imshow( test_win, output );
   resizeWindow( test_win, 400, 800 );

   imwrite("../../images/out/get_points_out.png", output );

   waitKey(0);


   return 0;
   }
