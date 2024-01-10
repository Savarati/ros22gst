#include <stdlib.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <iostream>
#include <string>
#include "ros22gst.hpp"

using namespace ros22gst;

Ros22gst::Ros22gst(const rclcpp::NodeOptions & options)
: rclcpp::Node("Ros22gst", options)
{
    RCLCPP_INFO(get_logger(), "start Ros22gst node... ");
    std::stringstream ss;
    std::string gsconfig_;
    GError * error = 0;
    gst_init (NULL, NULL);

    encoding = declare_parameter("encoding", "rgb8");
    
    const auto fps = declare_parameter("fps", "30/1");
    const auto width = declare_parameter("width", 640);
    const auto height = declare_parameter("height", 480);
    
    const auto topiccfg = declare_parameter("topic", "");
    const auto gsconfig_rosparam = declare_parameter("gscam_config", "");
    
    if(!gsconfig_rosparam.empty()) {
    	if(encoding == "rgb8")
            ss << "appsrc name=appsrc0 caps=video/x-raw," << "framerate=" << fps << ",width=" << width << ",height=" << height << ",format="
            << "RGB" << " ! " << gsconfig_rosparam;
        else
            ss << "appsrc name=appsrc0 caps=video/x-raw," << "framerate=" << fps << ",width=" << width << ",height=" << height << ",format="
            << "GRAY8" << " ! " << gsconfig_rosparam;
    	gsconfig_ = ss.str();
    } else {
        gsconfig_ = "appsrc name=appsrc0 caps=video/x-raw,framerate=30/1,width=640,height=480,format=RGB ! queue ! videoconvert ! autovideosink";
    }
    RCLCPP_INFO(get_logger(), "gsconfig_: %s.", gsconfig_.c_str());
    
    pipeline_ = gst_parse_launch(gsconfig_.c_str(), &error);
    if (pipeline_ == NULL) {
    	RCLCPP_FATAL_STREAM(get_logger(), error->message);
    	return;
    }
    source_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsrc0");

    gst_app_src_set_stream_type(GST_APP_SRC(source_), GST_APP_STREAM_TYPE_STREAM);
    g_object_set(G_OBJECT(source_), "format", GST_FORMAT_TIME, NULL);
    g_object_set(G_OBJECT(source_), "is-live", 1, NULL);
    g_object_set(G_OBJECT(source_), "do-timestamp", true, NULL);
    g_object_set(G_OBJECT(source_), "max-latency", 0, NULL);

    if(gst_element_set_state(GST_ELEMENT(pipeline_), GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {

        RCLCPP_ERROR(this->get_logger(),"gst_element_set_state(): cannot put pipeline to play");
        return;
    }
    
    auto topic = rclcpp::expand_topic_or_service_name(
    topiccfg.c_str(), this->get_name(), this->get_namespace());

    sub_image = image_transport::create_subscription(
    this, topic, std::bind(&Ros22gst::callback, this, std::placeholders::_1), "raw");

    RCLCPP_INFO(this->get_logger(), "Waiting for topic %s...", topic.c_str());
}

void Ros22gst::callback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
    static GstClockTime timestamp = 0;
    GstMapInfo info;
    //std::shared_ptr<cv::Mat> image;
    cv::Mat image;
    try {
      image = cv_bridge::toCvShare(image_msg, encoding.c_str())->image;
    } catch (const cv_bridge::Exception &) {
      RCLCPP_ERROR(
        this->get_logger(), "Unable to convert %s image for ",
        image_msg->encoding.c_str());
      return;
    }

    size_t size = image.total() * image.elemSize();
    buffer_ = gst_buffer_new_allocate (NULL, size, NULL);
    
    gst_buffer_map(buffer_, &info, GST_MAP_READ);
    memcpy(info.data, image.data, size);
    gst_buffer_unmap(buffer_, &info);

    RCLCPP_INFO(this->get_logger(), "push buffer size %ld, image code %s....", size, image_msg->encoding.c_str());
    if (gst_app_src_push_buffer(GST_APP_SRC(source_), buffer_) != GST_FLOW_OK) {
        RCLCPP_ERROR(get_logger(), "gst_app_src_push_buffer(): Error pushing buffer to GStreamer pipeline");
        return;
    }
}

// image black/white test
void Ros22gst::callback1(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
  static gboolean white = FALSE;
  static GstClockTime timestamp = 0;
  GstBuffer *buffer;
  guint size;
  GstFlowReturn ret;

  size = 640 * 480 * 3;

  buffer = gst_buffer_new_allocate (NULL, size, NULL);

  /* this makes the image black/white */
  gst_buffer_memset (buffer, 0, white ? 0xff : 0x0, size);

  white = !white;

  GST_BUFFER_PTS (buffer) = timestamp;
  GST_BUFFER_DURATION (buffer) = gst_util_uint64_scale_int (1, GST_SECOND, 2);

  timestamp += GST_BUFFER_DURATION (buffer);

  if (gst_app_src_push_buffer(GST_APP_SRC(source_), buffer) != GST_FLOW_OK) {
        RCLCPP_ERROR(get_logger(), "gst_app_src_push_buffer(): Error pushing buffer to GStreamer pipeline");
        return;
  }
  //gst_buffer_unref (buffer);

}

Ros22gst::~Ros22gst()
{
    if (pipeline_)
    {
        if (gst_app_src_end_of_stream(GST_APP_SRC(source_)) != GST_FLOW_OK)
        {
            RCLCPP_ERROR(get_logger(),"Cannot send EOS to GStreamer pipeline\n");
            return;
        }
        
        RCLCPP_INFO(get_logger(), "Stopping gstreamer pipeline...");
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = NULL;
        
        gst_deinit();

    }

       
}

