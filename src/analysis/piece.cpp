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
   for( unsigned int defect : defect_index )
      {
      int inf_idx = 0;

      // Iterate until find inflection just past maxima
      while( inflection_index[inf_idx] < defect )
         {
         inf_idx = next_index( inflection_index, inf_idx );
         }

      // Find inflection points before and after maxima
      unsigned int prv_inf = inflection_index[prev_index( inflection_index, inf_idx )];
      unsigned int nxt_inf = inflection_index[inf_idx];

      // Calculate tanget lines from inflection points


      }
