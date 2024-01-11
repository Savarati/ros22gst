#include <stdlib.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <iostream>
#include <sstream>
#include "ros22gst.hpp"

using namespace ros22gst;

Ros22gst::Ros22gst(const rclcpp::NodeOptions & options)
: rclcpp::Node("Ros22gst", options)
{
    RCLCPP_INFO(get_logger(), "start Ros22gst node... ");

    gst_init (NULL, NULL);

    this->configure();
    
    this->push_udpstream();
    
    pipeline_thread_ = std::thread(
    [this]()
    {
      run_pushRtsp();
    });
}

 void Ros22gst::configure()
 {
    RCLCPP_INFO(get_logger(), "start Ros22gst configure... ");
    std::stringstream ss;
    
    rtspRecord = false;
    encoding = declare_parameter("encoding", "rgb8");
    
    fps = declare_parameter("fps", "30/1");
    width = declare_parameter("width", 640);
    height = declare_parameter("height", 480);
    topiccfg = declare_parameter("topic", "");
    rtspport = declare_parameter("rtspport", 8554);
    rtspmount = declare_parameter("rtspmount", "/live0");
    udpport = declare_parameter("udpport", 8555);
    
    const auto gsconfig_rosparam = declare_parameter("gscam_config", "");
    if(!gsconfig_rosparam.empty()) {
    	if(encoding == "rgb8")
            ss << "appsrc name=appsrc0 caps=video/x-raw," << "framerate=" << fps << ",width=" << width << ",height=" << height << ",format="
            << "RGB" << " ! " << gsconfig_rosparam << " port=" << udpport;
        else
            ss << "appsrc name=appsrc0 caps=video/x-raw," << "framerate=" << fps << ",width=" << width << ",height=" << height << ",format="
            << "GRAY8" << " ! " << gsconfig_rosparam << " port=" << udpport;
    	gsconfig_ = ss.str();
    } else {
        gsconfig_ = "appsrc name=appsrc0 caps=video/x-raw,framerate=30/1,width=640,height=480,format=RGB ! queue ! videoconvert ! autovideosink";
    }
    RCLCPP_INFO(get_logger(), "gsconfig_: %s.", gsconfig_.c_str());
 }
 
static gboolean timeout(GstRTSPServer *server)
{
    GstRTSPSessionPool *pool;

    pool = gst_rtsp_server_get_session_pool(server);
    gst_rtsp_session_pool_cleanup(pool);
    g_object_unref(pool);

    return TRUE;
}

 
void Ros22gst::run_pushRtsp()
{
    RCLCPP_INFO(get_logger(),"start run_pushRtsp..");
    gchar * str;
    GstRTSPServer *server;
    GstRTSPMountPoints *mounts;
    GstRTSPMediaFactory *factory;
    GOptionContext *optctx;
    GError *error = NULL;

    loop = g_main_loop_new(NULL, FALSE);
    
    // Initialize RTSP server.
    server = gst_rtsp_server_new();
    if(server == NULL)
    {
        RCLCPP_INFO(get_logger(),"CamRtspStream: server get fail\n");
        return;
    }

    // Set the mServer IP address.
    //gst_rtsp_server_set_address (Server, rtspHost.c_str());

    // Set the mServer port.
    gst_rtsp_server_set_service (server, std::to_string(rtspport).c_str());

    // Get the mount points for this server.
    mounts = gst_rtsp_server_get_mount_points(server);
    if(mounts == NULL)
    {
        RCLCPP_INFO(get_logger(),"CamRtspStream: mounts get fail\n");
        return;
    }
    
    str = g_strdup_printf ("( udpsrc name=pay0 port=%d caps=\"" CAPS "\" )",udpport);
    RCLCPP_INFO(get_logger(),"RTSP pipeline at %s", str);

    // Create a media factory.
    factory = gst_rtsp_media_factory_new ();
    if(factory == NULL)
    {
        RCLCPP_INFO(get_logger(),"CamRtspStream: mfactory get fail\n");
        return;
    }

    gst_rtsp_media_factory_set_shared (factory, TRUE);

    // Add the factory with given path to the mount points.
    gst_rtsp_mount_points_add_factory (mounts, rtspmount.c_str(), factory);

    gst_rtsp_media_factory_set_transport_mode (factory,
      rtspRecord ? GST_RTSP_TRANSPORT_MODE_RECORD : GST_RTSP_TRANSPORT_MODE_PLAY);
    
    gst_rtsp_media_factory_set_launch (factory, str);


    // Add a timeout for the session cleanup.
    //g_timeout_add_seconds (5, (GSourceFunc)timeout, server);

    // Attach the RTSP mServer to the main context.
    if (0 == gst_rtsp_server_attach (server, NULL))
        RCLCPP_INFO(get_logger(),"Failed to attach RTSP server to main loop context!\n");

    // No need to keep reference for below objects.
    g_object_unref (mounts);

    RCLCPP_INFO(get_logger(),"Stream ready at rtsp://127.0.0.1:%d%s...", rtspport, rtspmount.c_str());
    
    g_main_loop_run(loop);
  
    g_object_unref(server);
    g_object_unref (factory);
    server = nullptr;
    factory = nullptr;
    g_main_loop_unref (loop);
}
 
void Ros22gst::push_udpstream()
{
    RCLCPP_INFO(get_logger(), "start Ros22gst push_udpstream... ");
    GError * error = 0;

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

    //RCLCPP_INFO(this->get_logger(), "push buffer size %ld, image code %s....", size, image_msg->encoding.c_str());
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

  size = 640 * 480 * 3;  //RGB8 size 

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
    g_main_loop_quit(loop);
    RCLCPP_INFO(get_logger(), "Stopping node...");
    pipeline_thread_.join();

    if (pipeline_)
    {
        if (gst_app_src_end_of_stream(GST_APP_SRC(source_)) != GST_FLOW_OK)
        {
            RCLCPP_ERROR(get_logger(),"Cannot send EOS to GStreamer pipeline\n");
            //return;
        }
        
        RCLCPP_INFO(get_logger(), "Stopping gstreamer pipeline...");
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = NULL;

    }
    
    gst_deinit();
}

