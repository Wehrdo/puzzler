#include "edge_tools.hpp"
#include "piece.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cstdio>

// Sort the contours, so in order around piece
bool compare_curve( Curve c1, Curve c2 )
   {
   return c1.start < c2.start;
   }

void Piece::process( void )
   {
   process_cvx_hull();
   find_indents();
   find_outdents();

   // Sort so that are in order of apearance
   std::sort( curves.begin(), curves.end(), compare_curve );
   }

void Piece::set_inflection( std::vector<std::size_t> infl )
   {
   this->inflection_index = infl;
   }

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

void Piece::characterize_curve( size_t start_idx, size_t end_idx, Curve::curve_type type )
   {

   cv::Point start_slp, end_slp, start, end, ins_pt;

   // Do the start/end already belong to somebody
   for( Curve past_curve : curves )
      {
      if( (past_curve.start == end_idx) ||
          (past_curve.end == start_idx) )
         return;
      }

   // Find tangent lines
   start_slp = find_tangent_angle( start_idx, contour );
   end_slp = find_tangent_angle( end_idx, contour );
   start = contour[start_idx];
   end = contour[end_idx];

   bool intersect = intersect_lines( start_slp, end_slp, start, end, ins_pt );
   int within = pointPolygonTest( contour, ins_pt, false );
   bool desired = ((type == Curve::indent  && !(within >0) )||
                   (type == Curve::outdent && (within > 0 )));

   // If cross and in right place
   if( intersect && desired )
      {
      size_t temp = start_idx;
      bool wrapped = false;
      std::vector<cv::Point> curve;
      while( temp != (end_idx+1) )
         {
         curve.push_back( contour[temp] );
         temp = next_index( contour, temp, wrapped );
         }

      // Find ellipse and compare sizes
      cv::RotatedRect best_fit = fitEllipse( curve );
      cv::Point2f circ;
      float radius;
      cv::minEnclosingCircle( curve, circ, radius );
      float circ_area = M_PI*pow(radius, 2 );
      float rect_area = best_fit.size.area();
      float ratio_area = circ_area / rect_area;

      if( ratio_area < 0.8 || ratio_area > 1.5 )
         return;

      // Add and return
      Curve to_add(start_idx, end_idx, cv::Point(best_fit.center), type );
      curves.push_back( to_add );
      std::cout << "Found a curve! Coords: first, second, centre: " << start << ", " << end << ", " << best_fit.center << std::endl;
      }

   }

void Piece::find_indents( void )
   {
   std::cout << "Finding indents" << std::endl;

   // For every defect, find our inflection points
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
      size_t prv_inf = inflection_index[prev_index( inflection_index, inf_idx, wrapped )];
      size_t nxt_inf = inflection_index[inf_idx];

      characterize_curve( prv_inf, nxt_inf, Curve::indent );
      }
   }

void Piece::find_outdents( void )
   {
   std::cout << "Finding outdents" << std::endl;
   // Find our inflection point indexes.
   for( size_t first_infl_idx = 0; first_infl_idx < inflection_index.size(); first_infl_idx++ )
      {
      bool wrapped = false;
      size_t first_infl = inflection_index[first_infl_idx];
      size_t second_infl = inflection_index[next_index( inflection_index, first_infl_idx, wrapped )];

      // Iterate until find first hull pt between first and second
      size_t hull_idx = 0;
      wrapped = false;
      while( hull_index[hull_idx] < first_infl && !wrapped)
         {
         hull_idx = next_index( hull_index, hull_idx, wrapped );
         }

      // Determine wrether we are at the end of our rope
      size_t hull = hull_index[hull_idx];
      if( (first_infl < second_infl &&
           ( hull >= second_infl || hull <= first_infl )
           )
          ||
          (first_infl > second_infl && hull < first_infl && hull > second_infl)
         )
         {
         std::cout << "No convex between these infl pts" << std::endl;
         continue;
         }

      // Find it!
      characterize_curve( first_infl, second_infl, Curve::outdent );

      }
   }


void Piece::draw( unsigned int width ) {
    draw(width, false);
}

void Piece::draw( unsigned int width, bool with_image )
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
   if (with_image) {
       cv::Mat scaled_img;
       int max_y_orig = raw_image.rows - min_y.y;
       int min_y_orig = raw_image.rows - max_y.y;
       cv::resize(raw_image.rowRange(min_y.y, max_y.y).colRange(min_x.x, max_x.x), out_img, cv::Size(width, height));
       // Flip vertically
       cv::flip(out_img, out_img, 0);
   }
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

   int count = 0;
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
      cv::putText( out_img, std::to_string(count++), centre + cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, white );

      }

   cv::imshow( name, out_img );
   }
