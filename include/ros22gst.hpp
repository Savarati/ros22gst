#ifndef PIPELINE_H
#define PIPELINE_H
#include <string>
#include <opencv2/core/core.hpp>
extern "C" {
#include "gst/gst.h"
#include "gst/app/gstappsrc.h"
}

#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace ros22gst
{

class Ros22gst : public rclcpp::Node
{
public:
    Ros22gst(const rclcpp::NodeOptions & options);
    GstElement *pipeline_;
    GstElement *source_;
    image_transport::Subscriber sub_image;
    GstBuffer* buffer_;
    std::string encoding;
    
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
    void callback1(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
    ~Ros22gst();
};

}

#endif // PIPELINE_H
