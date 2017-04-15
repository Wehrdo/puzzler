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

template<typename T>
unsigned int next_index( std::vector<T> vector, unsigned int index, bool& wrapped)
   {
   if( index == vector.size()-1 )
      {
      wrapped = true;
      return 0;
      }
   else
      {
      wrapped = false;
      return index+1;
      }
   }

template<typename T>
unsigned int prev_index( std::vector<T> vector, unsigned int index, bool& wrapped )
   {
   if( index == 0 )
      {
      wrapped = true;
      return vector.size()-1;
      }
   else
      {
      wrapped = false;
      return index-1;
      }
   }

cv::Point find_tangent_angle(int idx, std::vector<cv::Point> points)
   {
   int left_idx = idx == 0 ? points.size() - 1 : idx - 1;
   int right_idx = (idx + 1) % points.size();
   return points[left_idx] - points[right_idx];
   }

bool intersect_lines(cv::Point v1, cv::Point v2, cv::Point o1, cv::Point o2, cv::Point& intersection_pt) {
    /* Parametric equation of a line
    p1 = o1 + v1 * u
    p2 = o2 + v2 * v
    Set p1 == p2, and solve for either u or v.
    Then put the solution into one of the line equations, and the result
    is the intersection
    */
    float denom = (v2.x * v1.y - v2.y * v1.x);
    if (denom != 0) {
    cv::Point o_diff = o2 - o1;
        float numerator = (o_diff.y * v2.x - o_diff.x * v2.y);
        float u = numerator / denom;
        intersection_pt = u * v1 + o1;
        return true;
    } else {
        return false;
    }
}

void Piece::find_indents( void )
   {
   for( unsigned int defect : defect_index )
      {
      std::cout << " looking at defect " << defect << std::endl;
      unsigned int inf_idx = 0;
      bool wrapped = false;

      // Iterate until find inflection just past maxima
      while( inflection_index[inf_idx] < defect && !wrapped )
         {
         inf_idx = next_index( inflection_index, inf_idx, wrapped );
         }


      // Find inflection points before and after maxima
      unsigned int prv_inf = inflection_index[prev_index( inflection_index, inf_idx, wrapped )];
      unsigned int nxt_inf = inflection_index[inf_idx];

      std::cout << "found inflection points " << prv_inf << " and " << nxt_inf << std::endl;

      // Calculate tanget lines from inflection points

      cv::Point prv_slp, nxt_slp, prv, nxt, ins_pt;
      prv_slp = find_tangent_angle( prv_inf, contour );
      nxt_slp = find_tangent_angle( nxt_inf, contour );

      std::cout << "found the tangent angles: " << prv_slp << ", " << nxt_slp << std::endl;

      prv = contour[prv_inf];
      nxt = contour[nxt_inf];

      bool intersect = intersect_lines( prv_slp, nxt_slp, prv, nxt, ins_pt );

      std::cout << "The lines do " << (!intersect ? "not ":"") << "intersect" << std::endl;

      int within = pointPolygonTest( contour, ins_pt, false );
      if( intersect && !( within > 0 ) )
         {
         // Found a curve
         std::cout << "Found a true curve!" << std::endl;



         }
      }
   }
