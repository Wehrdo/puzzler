#include "edge_tools.hpp"
#include "piece.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cstdio>



void Piece::process_cvx_hull()
   {
   float threshold = 10.0;
   /* ConvexityDefects
   *
   * start_index
   * stop_index
   * max_depth_index
   * max_depth
   */

   cv::convexHull( contour, hull_index );
   std::vector<cv::Vec4i> defects;
   cv::convexityDefects( contour, hull_index, defects );

   unsigned int i;
   for( i = 0; i < defects.size(); i++ )
      {
      if( defects[i][3]/256.0 > threshold )
         defect_index.push_back(defects[i][2]);
      }
   }

void Piece::find_indents( void )
   {
   for( unsigned int i; i < hull_index.size()-1; i++ )
      {
      int prev_hull = hull_index[i];
      int next_hull = hull_index[i+1];

      //inflection_index.find( inflection_index.begin(), inflection_index.end(),

      }
   }
