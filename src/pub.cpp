
#include "lanefollower_ros2/vision.hpp"
using std::placeholders::_1;

const int def_speed = 100;
const double k = 0.32;

Mat preprocess(Mat input){
    Mat cut_gray;
    cvtColor(input(Rect(0, 270, 640, 90)), cut_gray, COLOR_BGR2GRAY);
    cut_gray = cut_gray + (Scalar(190) - mean(cut_gray));
    GaussianBlur(cut_gray, cut_gray, Size(9, 9), 3, 3);
    cv::imshow("win1", cut_gray);

    threshold(cut_gray, cut_gray, 235, 256, THRESH_BINARY);
    morphologyEx(cut_gray, cut_gray, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(9, 9)));
    return cut_gray;
}


int calc_err(Mat gray_img) {
    Mat labels, stats, centroids;
    int cnt = connectedComponentsWithStats(gray_img, labels, stats, centroids);

    Mat dst;
    cvtColor(gray_img, dst, COLOR_GRAY2BGR);
    static Point center_variable_l(160, 45);
    static Point center_variable_r(160+320, 45);

    int min_norm_l = INT_MAX;
    int min_norm_r = INT_MAX;

    Rect sel_l;
    Rect sel_r;

    for (int i = 1; i < cnt; i++) {
        int* p = stats.ptr<int>(i);
        Rect r(p[0], p[1], p[2], p[3]);
        rectangle(dst, r, Scalar(150, 150, 150), 1, 8);

        Point r_center = (r.br() + r.tl()) * 0.5;
        int diff_length = norm(center_variable_l - r_center);
        drawMarker(dst, r_center, Scalar(0, 0, 255), 0, 10, 2, 8);
        //L
        if (min_norm_l > diff_length && p[1] + p[3] > 60 && diff_length < 120) {
            sel_l = r;
            min_norm_l = diff_length;
            center_variable_l = r_center;
        }
    }

    for (int i = 1; i < cnt; i++) {
        int* p = stats.ptr<int>(i);
        Rect r(p[0], p[1], p[2], p[3]);

        Point r_center = (r.br() + r.tl()) * 0.5;
        int diff_length = norm(center_variable_r - r_center);
        drawMarker(dst, r_center, Scalar(0, 0, 255), 0, 10, 2, 8);
        //R
        if (min_norm_r > diff_length && p[1] + p[3] > 60 && diff_length < 120) {
            sel_r = r;
            min_norm_r = diff_length;
            center_variable_r = r_center;
        }
    }

    if (!sel_l.empty()) rectangle(dst, sel_l, Scalar(0, 255, 255), 2);
    if (!sel_r.empty()) rectangle(dst, sel_r, Scalar(0, 255, 255), 2);
    drawMarker(dst, center_variable_l, Scalar(255, 128, 255), 3, 10, 2, 8);
    drawMarker(dst, center_variable_r, Scalar(255, 128, 255), 3, 10, 2, 8);
    line(dst, center_variable_l, center_variable_r, Scalar(255, 0, 128), 2);

    Point pt_err((center_variable_l.x+center_variable_r.x)/2, (center_variable_l.y+center_variable_r.y)/2);
    drawMarker(dst, pt_err, Scalar(200, 0, 0), 4, 10, 2, 8);
    cv::imshow("win2", dst);
    return pt_err.x;
}
  
void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    static auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    static auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("/pterr", qos_profile);
    geometry_msgs::msg::Vector3 msg_err;

    cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    cv::imshow("win0", img);
    
    RCLCPP_INFO(node->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(),img.rows,img.cols);
    
    cv::Mat pre = preprocess(img);

    int err = calc_err(pre) - 320;

    cv::waitKey(1);

    int vel1 = def_speed + (err*k);
    int vel2 = -1 * (def_speed - (err*k));
    
    msg_err.x = vel1;
    msg_err.y = vel2;

    RCLCPP_INFO(node->get_logger(), "Publish: %lf, %lf", msg_err.x,  msg_err.y);
    mypub->publish(msg_err);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("lanefollower_ros2_calc");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed",qos_profile,fn);
    rclcpp::WallRate loop_rate(40.0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
