#ifndef FIND_POINTS_H
#define FIND_POINTS_H

// openCV includes
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// System includes
#include <stdlib.h>

// Local includes
#include "piece.hpp"

std::vector<Piece> find_pieces( cv::Mat img );


#endif /* FIND_POINTS_H */
