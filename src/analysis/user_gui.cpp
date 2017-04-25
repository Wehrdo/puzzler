#include "user_gui.hpp"
#include "edge_tools.hpp"
#include <math.h>
#include <numeric>
#include <map>


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
            piece_img.copyTo(buffer);
        }
    }
    if (finding_pt == 0) {
        circle(buffer, Point(x, y), 10, green, 2);
    }
    else {
        circle(buffer, Point(x, y), 10, red, 2);
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

    namedWindow(window_name, WINDOW_NORMAL);
    resizeWindow(window_name, 560, 560);
    // Any mouse event will call mouse_mv_cb
    setMouseCallback(window_name, mouse_cb, this);


    auto comp_x = [](cv::Point a, cv::Point b) { return a.x < b.x; };
    auto comp_y = [](cv::Point a, cv::Point b) { return a.y < b.y; };

    max_x = *max_element(piece.contour.begin(), piece.contour.end(), comp_x);
    max_y = *max_element(piece.contour.begin(), piece.contour.end(), comp_y);
    min_x = *min_element(piece.contour.begin(), piece.contour.end(), comp_x);
    min_y = *min_element(piece.contour.begin(), piece.contour.end(), comp_y);

    scale = double(window_width) / (max_x.x - min_x.x);
    window_height = ceil(scale * (max_y.y - min_y.y));

    // Copy over original raw image
    raw_image = piece.raw_image;
    // Create image matrix with dimensions to fill width, and proportional height
    piece_img = Mat(window_height, window_width, CV_8UC3, cv::Scalar(0));
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

    //
    cv::resize(raw_image.rowRange(min_y.y, max_y.y).colRange(min_x.x, max_x.x), piece_img, cv::Size(window_width, window_height));
    
    // Draw the piece to matrix
    cv::Point last_pt = local_pts[local_pts.size() - 1];
    for (size_t i = 0; i < local_pts.size(); ++i) {
        cv::Point pt = local_pts[i];
        if ((i > start_pt_idx && i <= end_pt_idx && start_pt_idx <= end_pt_idx) || // Standard bounds
            (start_pt_idx > end_pt_idx && (i > start_pt_idx || i < end_pt_idx))) // Bounds wrap around 0
            {
            cv::line(piece_img, last_pt, pt, red, 4);
        } else {
            cv::line(piece_img, last_pt, pt, white, 2);
        }
        last_pt = pt;
    }
}

void highlight_matches(Edge match_edge, std::vector<Edge> potential) {
   // No valid matches
   if (potential.size() == 0) {
       cout << "No valid potential pieces" << endl;
       return;
   }
   vector<pair<ssize_t, float>> piece_errors;

   // Calculate all errors
   for (size_t i = 0; i < potential.size(); ++i) {
        Edge edge = potential[i];
        float piece_err = match_edge.compare(edge);
        piece_errors.push_back(std::make_pair(i, piece_err));
    }

    // Sort matches by least error
    std::sort(piece_errors.begin(), piece_errors.end(),
        [](const std::pair<ssize_t, float>& first, const std::pair<ssize_t, float>& second) {
            return first.second < second.second;
        });
    // for (auto err : piece_errors) {
    //     potential[err.first].owner_piece.draw(480, true);
    //     cv::waitKey(0);
    // }

    // Found no matches
    if (piece_errors[0].second == INFINITY) {
        cout << "Sorry, found no matches." << endl;
        return;
    }

    // Calculate standard deviation of errors
    double mean_x = 0;
    double mean_x2 = 0;
    int n_valid = 0;
    for (auto edge_match : piece_errors) {
        // Invalid matches have error of infinity
        if (edge_match.second != INFINITY) {
            mean_x += edge_match.second;
            mean_x2 += (edge_match.second * edge_match.second);
            n_valid++;
        }
    }
    mean_x /= n_valid;
    mean_x2 /= n_valid;
    double stdev = sqrt(mean_x2 - (mean_x * mean_x));
    // Allow up to 0.5 standard deviations above minimum error
   //  double max_valid_error = piece_errors[0].second + 0.4*stdev;
    double max_valid_error = piece_errors[0].second + 0.007;

    Scalar red(0, 0, 255);

    // Images that need to be shown 
    map<uchar*, Mat> shown_images;
    for (pair<ssize_t, float> match : piece_errors) {
        // Stop searching after error is too high
        if (match.second > max_valid_error) {
            break;
        }
        printf("Highlighting piece %ld with error %f\n", match.first, match.second);
        Edge edge = potential[match.first];
        // First time image is shown
        Mat raw_image = edge.owner_piece.raw_image;
        if (shown_images.find(raw_image.data) == shown_images.end()) {
            shown_images[raw_image.data] = raw_image.clone();
        }
        Mat highlighted = shown_images[raw_image.data];
        Mat poly_overlay = Mat(highlighted.rows, highlighted.cols, highlighted.type());
        poly_overlay = Scalar(0, 0, 0);
        int n_points = edge.owner_piece.contour.size();
        const Point* points[1] = {&edge.owner_piece.contour[0]};
        fillPoly(poly_overlay, points, &n_points, 1, red);
        polylines(poly_overlay, points, &n_points, 1, true, red, highlighted.rows / 150);
        addWeighted(highlighted, 1.0, poly_overlay, 0.8, 0, highlighted);
        // add(highlighted, poly_overlay, shown_images[raw_image.data]);
    }
    int img_id = 0;
    for (auto highlight : shown_images) {
        string window_name = string("Picture ") + to_string(img_id);
        namedWindow(window_name, WINDOW_NORMAL);
        imshow(window_name, highlight.second);
        resizeWindow(window_name, 560, 560);
        img_id++;
    }
    waitKey(0);
}