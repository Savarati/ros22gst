#ifndef PIPELINE_H
#define PIPELINE_H
#include <string>
#include <opencv2/core/core.hpp>
extern "C" {
#include "gst/gst.h"
#include "gst/app/gstappsrc.h"
#include <gst/rtsp-server/rtsp-server.h>
}
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#define CAPS "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96"

namespace ros22gst
{

class Ros22gst : public rclcpp::Node
{
public:
    Ros22gst(const rclcpp::NodeOptions & options);
    ~Ros22gst();
    GstElement *pipeline_;
    GstElement *source_;
    image_transport::Subscriber sub_image;
    GstBuffer* buffer_;
    
private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
    void callback1(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
    std::thread pipeline_thread_;
    void run_pushRtsp();
    void configure();
    void push_udpstream();
    std::string    gsconfig_;
    std::string    encoding;
    std::string    fps;
    int32_t        width;
    int32_t        height;
    std::string    topiccfg;
    
    int32_t        rtspport;
    std::string    rtspmount;
    int32_t        udpport;
    bool           rtspRecord;
    
};

}

#endif // PIPELINE_H
