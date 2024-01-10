
// example appsrc for gstreamer 1.0 with own mainloop & external buffers. based on example from gstreamer docs.
// public domain, 2015 by Florian Echtler <floe@butterbrot.org>. compile with:
// gcc --std=c99 -Wall $(pkg-config --cflags gstreamer-1.0) -o gst gst.c $(pkg-config --libs gstreamer-1.0) -lgstapp-1.0



#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"
#include "ros22gst.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto node = std::make_shared<ros22gst::Ros22gst>(options);
    RCLCPP_INFO(node->get_logger(), "start ros2 to gst");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
