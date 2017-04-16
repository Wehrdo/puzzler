#ifndef USER_GUI_HPP
#define USER_GUI_HPP

#include <functional>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "piece.hpp"

class PuzzleGUI {
public:
    /*
     * Pass in a unique window name and a callback function
     * to be called after edge selection is done
     */
    PuzzleGUI(std::string win_name);
    
    std::pair<size_t, size_t> select_edge(Piece p);
    /*
     * Returns the index of the point nearest to the given point
     */
    size_t find_nearest_point(cv::Point given_p);
    void draw_piece();
    static void mouse_mv_cb(int event, int x, int y, int flags, void *obj);
    std::string window_name;
    int window_width = 560;


    // Image of the piece
    cv::Mat piece_img;
    // Buffer to draw to
    cv::Mat buffer;
    size_t start_pt_idx;
    size_t end_pt_idx;
    int finding_pt;
    cv::Point2d min_x, min_y, max_x, max_y;
    double scale;
    cv::Point pt2img_pt(cv::Point pt);
    cv::Point img_pt2pt(cv::Point pt);
    // Copy of the points, converted to pixel coordinates
    std::vector<cv::Point> local_pts;
};

#endif // USER_GUI_HPP