#ifndef PIECE_H
#define PIECE_H

#include "edge_tools.hpp"
#include "curve.hpp"

// to keep contours, can remove after testing
#include "opencv2/imgproc/imgproc.hpp"

class Piece
{
  public:
    // Member variables
    std::vector<Vec2d> points;
    std::vector<cv::Point> contour;
    std::vector<int> hull_index;
    std::vector<size_t> defect_index;
    std::vector<size_t> inflection_index;
    std::vector<Curve> curves;
    // Index of the original input this piece belongs to
    cv::Mat raw_image;

    // Constructors
    Piece()
    {
    }

    Piece(std::vector<Vec2d> points, cv::Mat img)
    {
        this->points = points;
        this->raw_image = img;
    }

    // Member functions
    void process_cvx_hull();

    //TODO: Move inflection point calc here
    //TODO: Move draw function here

    void process(void);
    void set_inflection(std::vector<std::size_t> infl);
    void find_indents(void);
    void find_outdents(void);
    void characterize_curve( size_t start_idx, size_t end_idx, Curve::curve_type type );
    void draw(unsigned int width);
    void draw(unsigned int width, bool with_image);
};

// Inflection point code here?

#endif /* PIECE_H */
