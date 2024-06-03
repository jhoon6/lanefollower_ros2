#ifndef __VISION_HPP__
#define __VISION_HPP__

#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "lanefollower_ros2/dxl.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <memory>

using namespace std;
using namespace cv;

int calc_err(Mat gray_img);
Mat preprocess(Mat input);

#endif