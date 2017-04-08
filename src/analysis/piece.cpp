#include "edge_tools.hpp"
#include "piece.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cstdio>



void Piece::process_cvx_hull()
   {

   /* ConvexityDefects
   *
   * start_index
   * stop_index
   * max_depth_index
   * max_depth
   */

   std::vector<cv::Point> hull;
   cv::convexHull( contour, hull );
   std::vector<cv::Vec4i> defects;
   cv::convexityDefects( contour, hull, defects );

   unsigned int i;
   for( i = 0; i < defects.size(); i++ )
      {
      cvx_hull_max_dist.push_back(defects[i][2]);
      }

   }
