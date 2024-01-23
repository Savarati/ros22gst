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

typedef struct
{
  GstElement *pipeline_;
  GstElement *appsrc_;
  GMutex         m_mutex;
} RtspContext;

namespace ros22gst
{

class Ros22gst : public rclcpp::Node
{
public:
    Ros22gst(const rclcpp::NodeOptions & options);
    ~Ros22gst();
    image_transport::Subscriber sub_image;
    GstBuffer* buffer_;
    
private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
    void callback1(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
    void appsrc_set_caps(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
    //static void media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media, RtspContext *ctx);
    //static void ctx_free (RtspContext * ctx);
    //void media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media, gpointer user_data);
    GstRTSPServer *server;
    std::thread pipeline_thread_;
    void push_rtspstream();
    void configure();
    GMutex         m_mutex;
    GMainLoop      *loop;
    std::string    gsconfig_;
    std::string    encoding;

    bool           rtspReady;
    bool           isFristCall;

    int32_t        fps;
    int32_t        width;
    int32_t        height;
    std::string    topiccfg;
    
    std::string    rtspHost;
    std::string    rtspMount;
    int32_t        rtspPort;
    bool           rtspRecord;
    bool           needSetCaps;
    guint          gst_server_id;

    bool           clientconnnect;

    RtspContext    *rtspCtx;;
    
};

}

static gboolean timeout(GstRTSPServer *server);
static void media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media, RtspContext *ctx);
#endif // PIPELINE_H
