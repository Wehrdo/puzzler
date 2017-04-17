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
   for( unsigned int dft_idx = 0; dft_idx < defect_index.size(); dft_idx++ )
      {
      unsigned int defect = defect_index[dft_idx];
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

      // Calculate tanget lines from inflection points
      cv::Point prv_slp, nxt_slp, prv, nxt, ins_pt;
      prv_slp = find_tangent_angle( prv_inf, contour );
      nxt_slp = find_tangent_angle( nxt_inf, contour );

      prv = contour[prv_inf];
      nxt = contour[nxt_inf];

      bool intersect = intersect_lines( prv_slp, nxt_slp, prv, nxt, ins_pt );
      int within = pointPolygonTest( contour, ins_pt, false );
      if( intersect && !( within > 0 ) )
         {
         // Find the center of indent
         cv::RotatedRect best_fit = fitEllipse(
            std::vector<cv::Point>( &(contour[prv_inf]), &(contour[nxt_inf]) ) );

         bool tmp;

         Curve to_add( prv_inf, nxt_inf,  cv::Point(best_fit.center), Curve::indent );
         curves.push_back( to_add );
         }
      }
   }

void Piece::find_outdents( void )
   {
   for( unsigned int hull_idx = 0; hull_idx < hull_index.size(); hull_idx++ )
      {
      unsigned int hull = hull_index[hull_idx];
      unsigned int inf_idx = 0;
      bool wrapped = false;

      // Iterate until find inflection just past maxima
      while( inflection_index[inf_idx] < hull && !wrapped )
         {
         inf_idx = next_index( inflection_index, inf_idx, wrapped );
         }


      // Find inflection points before and after maxima
      unsigned int prv_inf = inflection_index[prev_index( inflection_index, inf_idx, wrapped )];
      unsigned int nxt_inf = inflection_index[inf_idx];

      // Calculate tanget lines from inflection points
      cv::Point prv_slp, nxt_slp, prv, nxt, ins_pt;
      prv_slp = find_tangent_angle( prv_inf, contour );
      nxt_slp = find_tangent_angle( nxt_inf, contour );

      prv = contour[prv_inf];
      nxt = contour[nxt_inf];

      bool intersect = intersect_lines( prv_slp, nxt_slp, prv, nxt, ins_pt );

      std::cout << "Distance between previous and ins: " << cv::norm(prv - ins_pt) <<
         "\nDistance between nxt and ins: " << cv::norm(nxt - ins_pt ) << std::endl;

      int within = pointPolygonTest( contour, ins_pt, false );
      if( intersect && ( within > 0 ) )
         {

         std::vector<cv::Point> curve;
         unsigned int start = prv_inf;
         bool wrapped = false;

         while( start != nxt_inf )
            {
            curve.push_back( contour[start] );
            start = next_index( contour, start, wrapped );
            }

         // Find the center of indent
         cv::RotatedRect best_fit = fitEllipse( curve );

         bool tmp;

         Curve to_add( prv_inf, nxt_inf,  cv::Point(best_fit.center), Curve::outdent );
         curves.push_back( to_add );
         }
      }
   }

void Piece::draw( unsigned int width )
   {
   std::string name = "Piece info";
   cv::namedWindow( name );

   auto comp_x = [](cv::Point a, cv::Point b) {return a.x < b.x;};
   auto comp_y = [](cv::Point a, cv::Point b) {return a.y < b.y;};

   cv::Point max_x = *max_element(contour.begin(), contour.end(), comp_x );
   cv::Point max_y = *max_element(contour.begin(), contour.end(), comp_y );
   cv::Point min_x = *min_element(contour.begin(), contour.end(), comp_x );
   cv::Point min_y = *min_element(contour.begin(), contour.end(), comp_y );

   double scale = double( width) / (max_x.x - min_x.x);
   unsigned int height = ceil( scale * (max_y.y - min_y.y) );

   // Function converts x,y values to pixel opencv point representing pixel location
   // Also takes into account that positive Y points downward in graphics
   auto convert_coord = [scale, min_x, min_y, max_x, max_y](cv::Point pt)
      {
         return cv::Point((pt.x - min_x.x) * scale,
                          scale * (max_y.y - pt.y));};

   // Create image matrix with dimensions to fill width, and proportional height
   cv::Mat out_img(height, width, CV_8UC3, cv::Scalar(0));
   cv::Scalar white = cv::Scalar(255, 255, 255);
   cv::Scalar red = cv::Scalar(0, 0, 255);
   cv::Scalar blue = cv::Scalar(255, 80, 80);
   cv::Scalar green = cv::Scalar(0, 255, 0);

   // Draw the piece to matrix
   cv::Point last_pt = convert_coord( contour[contour.size() -1] );
   for( cv::Point p : contour)
      {
      cv::Point this_pt = convert_coord( p );
      cv::line( out_img, last_pt, this_pt, white );
      last_pt = this_pt;
      }

   // Draw the defects
   for( size_t idx : defect_index )
      {
      cv::Point pt = convert_coord( contour[idx] );
      cv::circle(out_img, pt, 5, green );
      }

   for( size_t idx : inflection_index )
      {
      cv::Point infl_pt = convert_coord( contour[idx] );
      cv::circle(out_img, infl_pt, 10, white);
      cv::putText( out_img, std::to_string(idx), infl_pt + cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, white );

      }

   for( Curve curve : curves )
      {
      cv::Point centre = convert_coord( curve.origin );
      cv::Scalar color;
      if( curve.type == Curve::indent )
         color = blue;
      else
         color = red;

      cv::circle(out_img, centre, 20, color );
      }

   cv::imshow( name, out_img );

   }
