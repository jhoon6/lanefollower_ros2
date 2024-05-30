#include "lanefollower_ros2/vision.hpp"

using namespace std::placeholders;

const int def_speed = 100;
const double k = 0.32;

void mysub_callback(rclcpp::Node::SharedPtr node, Dxl& dxl, const std_msgs::msg::Int32::SharedPtr msg)
{
    static bool allow_to_start = false;
    if (dxl.kbhit()) //키보드입력 체크
    {
        char c = dxl.getch(); //키입력 받기
        if (c == 's') allow_to_start = true;
    }

    RCLCPP_INFO(node->get_logger(), "Received message: %d", msg->data);
    if (allow_to_start)
    {
        int error = msg->data - 320;
        int vel1 = def_speed + (error*k);
        int vel2 = -1 * (def_speed - (error*k));
        dxl.setVelocity(vel1, vel2);
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    Dxl dxl;
    auto node = std::make_shared<rclcpp::Node>("node_dxlsub");

    if(!dxl.open())
    {
        RCLCPP_ERROR(node->get_logger(), "dynamixel open error");
        rclcpp::shutdown();
        return -1;
    }
    

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    std::function<void(const std_msgs::msg::Int32::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, dxl, _1);
    auto mysub = node->create_subscription<std_msgs::msg::Int32>("/pterr",qos_profile,fn);
    RCLCPP_INFO(node->get_logger(), "Press 's'...");
    
    rclcpp::spin(node);
    
    dxl.close();
    rclcpp::shutdown();
    return 0;
}