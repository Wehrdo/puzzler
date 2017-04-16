#ifndef USER_GUI_HPP
#define USER_GUI_HPP

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
    /*
     * When called, launches a window and lets the user select a start
     * and end point on the puzzle. After the user hits the spacebar, 
     * returns an inclusive start and end index
     */
    std::pair<size_t, size_t> select_edge(Piece p);
private:
    /*
     * Returns the index of the point nearest to the given point
     */
    size_t find_nearest_point(cv::Point given_p);
    /*
     * Draws the piece, as well as the user's selection on it
     */
    void draw_piece();
    /*
     * Callback for mouse events on the window
     */
    // Static version is to pass as OpenCV callback, where the *obj is *this
    // It simply calls the class-member version of the function
    static void mouse_cb(int event, int x, int y, int flags, void *obj);
    // class-member version that gets called from the static version
    void mouse_cb(int event, int x, int y, int flags);
    
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