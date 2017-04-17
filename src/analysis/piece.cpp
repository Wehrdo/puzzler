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
   std::sort( hull_index.begin(), hull_index.end() );
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
   // Find our inflection point indexes.
   for( unsigned int first_infl = 0; first_infl < inflection_index.size(); first_infl++ )
      {
      bool wrapped = false;
      unsigned int second_infl = next_index( inflection_index, first_infl, wrapped );

      std::cout << "ifnl points: first, " << inflection_index[first_infl] << " second, " << inflection_index[second_infl] << std::endl;


      cv::Point first = contour[ inflection_index[ first_infl ] ];
      cv::Point second = contour[ inflection_index[ second_infl ] ];

      // Find the equation of the line between them
      float A = -1.0 * ( second.y - first.y );
      float B = second.x - first.x;
      float C = -1.0*(A*first.x) - (B*first.y);

      // Iterate until find first hull pt between first and second
      unsigned int hull_idx = 0;
      wrapped = false;
      while( hull_index[hull_idx] < inflection_index[ first_infl ] )
         {
         hull_idx = next_index( hull_index, hull_idx, wrapped );
         }


      if(
         (inflection_index[first_infl] < inflection_index[second_infl] &&
          hull_index[hull_idx] > inflection_index[second_infl]
         )
          ||
         (
          inflection_index[first_infl] > inflection_index[second_infl] &&
          hull_index[hull_idx] < inflection_index[first_infl] &&
          hull_index[hull_idx] > inflection_index[second_infl]
         )
        )
         {
         std::cout << "No convex between these infl pts" << std::endl;
         continue;
         }


      std::cout << "convex point: " << hull_index[hull_idx] << std::endl;

      // Find furthest pt between first and second from line
      wrapped = false;
      float max_distance = 0.0;
      cv::Point max_pt;
      while( (inflection_index[second_infl] > inflection_index[first_infl] &&
              hull_index[hull_idx] < inflection_index[ second_infl ])
             || (inflection_index[second_infl] < inflection_index[first_infl] &&
                 ( hull_index[hull_idx] < inflection_index[second_infl] ||
                   hull_index[hull_idx] > inflection_index[first_infl] )
                )
         )
         {

         cv::Point pt = contour[hull_index[hull_idx]];
         float distance = abs( A*pt.x + B*pt.y + C ) / sqrt( pow(A, 2) + pow(B, 2) );
         if( distance > max_distance )
            {
            max_pt = pt;
            max_distance = distance;
            }
         hull_idx = next_index( hull_index, hull_idx, wrapped );
         }

      // Calculate tangent lines
      cv::Point first_slp, second_slp, ins_pt;
      first_slp = find_tangent_angle( inflection_index[first_infl], contour );
      second_slp = find_tangent_angle( inflection_index[second_infl], contour );

      // do they intersect?
      bool intersect = intersect_lines( first_slp, second_slp, first, second, ins_pt );

      int within = pointPolygonTest( contour, ins_pt, false );

      std::cout << "Lines do " << (intersect?"":"not ") << "intersect" << std::endl;

      std::cout << "Lines intersect at " << ins_pt << std::endl;

      // If intersect within piece
      if( intersect && (within > 0 ) )
         {

         std::cout << "Tanget lines crossed inside, finding points along curve" << std::endl;

         // Find points that characterize curve
         unsigned int start = inflection_index[first_infl];
         std::vector<cv::Point> curve;
         wrapped = false;
         while( start != inflection_index[second_infl] )
            {
            curve.push_back( contour[start] );
            start = next_index( contour, start, wrapped );
            std::cout << "Adding point " << start << " to curve." << std::endl;
            }

         // Find ellipse of best fit
         cv::RotatedRect best_fit = fitEllipse( curve );
         cv::Point2f circ;
         float radius;
         cv::minEnclosingCircle( curve, circ, radius );

         // compare area of circle with area of rotated rect
         float circ_area = M_PI*pow(radius, 2);
         float rect_area = best_fit.size.area();
         float ratio_area = circ_area / rect_area;
         std::cout << "circle area: " << circ_area << std::endl;
         std::cout << "rect area: " << rect_area << std::endl;
         std:: cout << "ratio of areas " << ratio_area << std::endl;

         if( ratio_area < 0.8 || ratio_area > 1.5 )
            continue;

         // Add to the piece's curve list
         Curve to_add( inflection_index[first_infl], inflection_index[second_infl], cv::Point( best_fit.center), Curve::outdent );
         curves.push_back(to_add);

         std::cout << "Found a curve! We've found " << curves.size() << " so far." << std::endl;
         std::cout << "coords: first, max, second, centre: " << first << second << max_pt << best_fit.center << std::endl;
         }
      }
   }

void Piece::find_outdents_old( void )
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

      unsigned int idx = curve.start;
      while( idx != curve.end )
         {
         bool wrapped;
         cv::circle( out_img, convert_coord(contour[idx]), 5, color );
         idx = next_index( contour, idx, wrapped );
         }

      cv::circle(out_img, centre, 20, color );
      }

   cv::imshow( name, out_img );

   }
