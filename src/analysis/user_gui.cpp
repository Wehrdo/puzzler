#include "user_gui.hpp"
#include "edge_tools.hpp"
#include <math.h>


using namespace std;
using namespace cv;

void PuzzleGUI::mouse_cb(int event, int x, int y, int flags, void *obj) {
    PuzzleGUI* that = static_cast<PuzzleGUI*>(obj);
    that->mouse_cb(event, x, y, flags);
}

void PuzzleGUI::mouse_cb(int event, int x, int y, int flags __attribute__((unused))) {
    cv::Scalar red(0, 0, 255);
    cv::Scalar green(0, 255, 0);
    piece_img.copyTo(buffer);
    if (event == EVENT_LBUTTONUP) {
        // Finding first point
        if (finding_pt == 0) {
            start_pt_idx = find_nearest_point(Point(x, y));
            circle(piece_img, local_pts[start_pt_idx], 10, green, CV_FILLED);
            // Toggle to second point
            finding_pt = 1;
        }
        else {
            end_pt_idx = find_nearest_point(Point(x, y));
            finding_pt = 0;
            draw_piece();
        }
    }
    if (finding_pt == 0) {
        circle(buffer, Point(x, y), 10, green);
    }
    else {
        circle(buffer, Point(x, y), 10, red);
    }
    imshow(window_name, buffer);
}

PuzzleGUI::PuzzleGUI(string name)
{
    window_name = name;
}

// Function converts x,y values to pixel opencv point representing pixel location
// Also takes into account that positive Y points downward in graphics
Point PuzzleGUI::pt2img_pt(Point pt) {
    return Point((pt.x - min_x.x) * scale, scale * (pt.y - min_y.y));
}

Point PuzzleGUI::img_pt2pt(Point pt) {
    return Point((pt.x / scale) + min_x.x,
                 (pt.y / scale) + min_y.y);
}

pair<size_t, size_t> PuzzleGUI::select_edge(Piece piece)
{
    // Start finding the start point
    finding_pt = 0;
    start_pt_idx = end_pt_idx = 0;

    namedWindow(window_name);
    // Any mouse event will call mouse_mv_cb
    setMouseCallback(window_name, mouse_cb, this);


    auto comp_x = [](cv::Point a, cv::Point b) { return a.x < b.x; };
    auto comp_y = [](cv::Point a, cv::Point b) { return a.y < b.y; };

    max_x = *max_element(piece.contour.begin(), piece.contour.end(), comp_x);
    max_y = *max_element(piece.contour.begin(), piece.contour.end(), comp_y);
    min_x = *min_element(piece.contour.begin(), piece.contour.end(), comp_x);
    min_y = *min_element(piece.contour.begin(), piece.contour.end(), comp_y);

    scale = double(window_width) / (max_x.x - min_x.x);
    unsigned int height = ceil(scale * (max_y.y - min_y.y));

    // Create image matrix with dimensions to fill width, and proportional height
    piece_img = Mat(height, window_width, CV_8UC3, cv::Scalar(0));
    buffer = piece_img.clone();

    // Convert points to pixel coordinate system
    local_pts.clear();
    for (cv::Point p : piece.contour) {
        local_pts.push_back(pt2img_pt(p));
    }
    draw_piece();
    imshow(window_name, piece_img);

    // Wait until done picking
    while (waitKey(30) != ' ') {
        ;
    }
    // Take down the selection window
    destroyWindow(window_name);
    return pair<size_t, size_t>(start_pt_idx, end_pt_idx);
}


size_t PuzzleGUI::find_nearest_point(Point given_pt) {
    int nearest_idx;
    double nearest_dist = INFINITY;
    for (size_t i = 0; i < local_pts.size(); ++i) {

        double dist = norm(local_pts[i] - given_pt);
        if (dist < nearest_dist) {
            nearest_dist = dist;
            nearest_idx = i;
        }
    }
    return nearest_idx;
}

void PuzzleGUI::draw_piece() {
    Scalar white(255, 255, 255);
    Scalar red(0, 0, 255);

    // Clear matrix
    piece_img = Scalar(0, 0, 0);
    // Draw the piece to matrix
    cv::Point last_pt = local_pts[local_pts.size() - 1];
    for (size_t i = 0; i < local_pts.size(); ++i) {
        cv::Point pt = local_pts[i];
        if ((i > start_pt_idx && i <= end_pt_idx && start_pt_idx <= end_pt_idx) || // Standard bounds
            (start_pt_idx > end_pt_idx && (i > start_pt_idx || i < end_pt_idx))) // Bounds wrap around 0
            {
            cv::line(piece_img, last_pt, pt, red, 3);
        } else {
            cv::line(piece_img, last_pt, pt, white);
        }
        last_pt = pt;
    }
}