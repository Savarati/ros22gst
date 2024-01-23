#include <stdlib.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <iostream>
#include <sstream>
#include "ros2gst.hpp"

using namespace ros22gst;

Ros22gst::Ros22gst(const rclcpp::NodeOptions & options)
: rclcpp::Node("Ros22gst", options)
{
    RCLCPP_INFO(get_logger(), "start Ros2gst node... ");

    gst_init (NULL, NULL);

    this->configure();
    
    this->push_rtspstream();

    pipeline_thread_ = std::thread(
    [this]()
    {
       g_main_loop_run(loop);
       RCLCPP_INFO(get_logger(),"thread exit...");
    });
    
    auto topic = rclcpp::expand_topic_or_service_name(
    topiccfg.c_str(), this->get_name(), this->get_namespace());

    sub_image = image_transport::create_subscription(
    this, topic, std::bind(&Ros22gst::callback, this, std::placeholders::_1), "raw");

    RCLCPP_INFO(this->get_logger(), "Waiting for topic %s...", topic.c_str());

}

void Ros22gst::configure()
{
    RCLCPP_INFO(get_logger(), "start Ros2gst configure... ");
    std::stringstream ss;
    
    rtspCtx = NULL;
    needSetCaps = true;
    rtspRecord = false;
    isFristCall = true;
    clientconnnect = false;
    encoding = declare_parameter("encoding", "rgb8");

    fps = declare_parameter("fps", 30);
    width = declare_parameter("width", 640);
    height = declare_parameter("height", 480);
    topiccfg = declare_parameter("topic", "image");
    rtspPort = declare_parameter("rtspport", 8554);
    rtspMount = declare_parameter("rtspmount", "/live0");

    const auto gsconfig_rosparam = declare_parameter("gscam_config", "");
    if(!gsconfig_rosparam.empty()) {
    	if(encoding == "rgb8")
            ss << "appsrc name=appsrc0 format=3 do-timestamp=1 is-live=1 caps=video/x-raw,framerate=30/1,width=640,height=480,format=RGB" << " ! " << gsconfig_rosparam;
        else
            ss << "appsrc name=appsrc0 " << " ! " << gsconfig_rosparam;
    	gsconfig_ = ss.str();
    } else {
        gsconfig_ = "appsrc name=appsrc0 caps=video/x-raw,framerate=30/1,width=640,height=480,format=RGB ! queue ! videoconvert ! nvh264enc ! h264parse config-interval=1 ! rtph264pay pt=96 name=pay0";
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

 
void Ros22gst::push_rtspstream()
{
    RCLCPP_INFO(get_logger(),"start run_pushRtsp..");
    gchar * str;
    //GstRTSPServer *server;
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
    gst_rtsp_server_set_service (server, std::to_string(rtspPort).c_str());

    // Get the mount points for this server.
    mounts = gst_rtsp_server_get_mount_points(server);
    if(mounts == NULL)
    {
        RCLCPP_INFO(get_logger(),"CamRtspStream: mounts get fail\n");
        return;
    }
    
    //RCLCPP_INFO(get_logger(),"RTSP pipeline at %s", pipeline_.c_str());

    // Create a media factory.
    factory = gst_rtsp_media_factory_new ();
    if(factory == NULL)
    {
        RCLCPP_INFO(get_logger(),"CamRtspStream: mfactory get fail\n");
        return;
    }

    gst_rtsp_media_factory_set_shared (factory, TRUE);

    // Add the factory with given path to the mount points.
    gst_rtsp_mount_points_add_factory (mounts, rtspMount.c_str(), factory);

    gst_rtsp_media_factory_set_transport_mode (factory,
      rtspRecord ? GST_RTSP_TRANSPORT_MODE_RECORD : GST_RTSP_TRANSPORT_MODE_PLAY);
    
    gst_rtsp_media_factory_set_launch (factory, gsconfig_.c_str());
    rtspCtx = g_new0 (RtspContext, 1);
    rtspCtx->appsrc_ = NULL;
    g_mutex_init(&rtspCtx->m_mutex);
    g_signal_connect(factory, "media-configure", (GCallback)media_configure, rtspCtx);
    //Add a timeout for the session cleanup.
    g_timeout_add_seconds (5, (GSourceFunc)timeout, server);

    // Attach the RTSP mServer to the main context.
    gst_server_id = gst_rtsp_server_attach (server, NULL);
    if (!gst_server_id ) {
        RCLCPP_ERROR(get_logger(),"Failed to attach RTSP server to main loop context!\n");
        return;
    }
    //g_signal_connect(server, "client-connected", reinterpret_cast<GCallback>(clientConnected), nullptr);

    // No need to keep reference for below objects.
    g_object_unref (mounts);

    rtspReady = true;
    RCLCPP_INFO(get_logger(),"Stream ready at rtsp://127.0.0.1:%d%s...", rtspPort, rtspMount.c_str());
    return;
}
/*void Ros22gst::clientConnected()
{
    g_print("client-connected -- count: %u\n", ++g_clientCount);
}*/

static void ctx_free (RtspContext * ctx)
{
  g_mutex_lock(&ctx->m_mutex);
  g_print("ctx_free \n");

  gst_object_unref (ctx->appsrc_);
  //gst_object_unref (ctx->pipeline_);
  ctx->appsrc_ = NULL;
  g_mutex_unlock(&ctx->m_mutex);
  //g_free (ctx);
}


static void media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media, RtspContext *ctx)
{
    //g_mutex_lock(&m_mutex);
    if(media)
    {
        //clientconnnect = true;
        g_print("media_configure\n");
        //ctx = g_new0 (RtspContext, 1);
        if(ctx == NULL)
        {
            g_print("g_new0 error!\n");
            // g_mutex_unlock(&m_mutex);
            return;
        }
        /* make sure the data is freed when the media is gone */
        g_object_set_data_full (G_OBJECT (media), "rtsp-extra-data", ctx, (GDestroyNotify) ctx_free);

        ctx->pipeline_ = gst_rtsp_media_get_element(media);
        ctx->appsrc_ = gst_bin_get_by_name_recurse_up (GST_BIN (ctx->pipeline_), "appsrc0");
        gst_object_unref (ctx->pipeline_);
    }else{
	    g_print("media_configure fail! ,not have appsrc! ... ");
         //g_mutex_unlock(&m_mutex);
	    return;
    }
     //g_mutex_unlock(&m_mutex);
}

void Ros22gst::appsrc_set_caps(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
    g_print("appsrc_set_caps\n");
     // http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
    static const std::map<std::string, std::string> known_formats = {
        {sensor_msgs::image_encodings::RGB8, "RGB"},
        {sensor_msgs::image_encodings::RGB16, "RGB16"},
        {sensor_msgs::image_encodings::RGBA8, "RGBA"},
        {sensor_msgs::image_encodings::RGBA16, "RGBA16"},
        {sensor_msgs::image_encodings::BGR8, "BGR"},
        {sensor_msgs::image_encodings::BGR16, "BGR16"},
        {sensor_msgs::image_encodings::BGRA8, "BGRA"},
        {sensor_msgs::image_encodings::BGRA16, "BGRA16"},
        {sensor_msgs::image_encodings::MONO8, "GRAY8"},
        {sensor_msgs::image_encodings::MONO16, "GRAY16_LE"},
    };

    auto format = known_formats.find(msg->encoding);
    if (format == known_formats.end()){
        RCLCPP_ERROR(this->get_logger(), "GST: image format '%s' unknown", msg->encoding.c_str());
        needSetCaps = true;
        return;
    }

    RCLCPP_INFO(get_logger(),"appsrc_set_caps format %s %d %d ",format->second.c_str(),msg->width, msg->height);
    GstCaps * caps = gst_caps_new_simple("video/x-raw",
                               "format", G_TYPE_STRING, format->second.c_str(),
                               "width", G_TYPE_INT, msg->width,
                               "height", G_TYPE_INT, msg->height,
                               "framerate", GST_TYPE_FRACTION, fps, 1,
                               nullptr);
    
    if(rtspCtx->appsrc_ == NULL)
    {
        RCLCPP_ERROR(this->get_logger(),"appsrc_set_caps fail! ,get appsrc fail! ... ");
        needSetCaps = true;
        return;
    }

    /* this instructs appsrc that we will be dealing with timed buffer */
    gst_app_src_set_stream_type(GST_APP_SRC(rtspCtx->appsrc_), GST_APP_STREAM_TYPE_STREAM);
    g_object_set(G_OBJECT(rtspCtx->appsrc_), "format", GST_FORMAT_TIME, NULL);
    g_object_set(G_OBJECT(rtspCtx->appsrc_), "is-live", 1, NULL);
    g_object_set(G_OBJECT(rtspCtx->appsrc_), "do-timestamp", true, NULL);
    g_object_set(G_OBJECT(rtspCtx->appsrc_), "max-latency", 0, NULL);

    gst_app_src_set_caps(GST_APP_SRC(rtspCtx->appsrc_), caps);
}

void Ros22gst::callback1(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
    if(!rtspReady || !rtspCtx || !rtspCtx->appsrc_)
    {
        //RCLCPP_INFO(get_logger(), "rtsp streamer have not ready or client not connect!");
        return;
    }
    g_mutex_lock(&rtspCtx->m_mutex);

    if(needSetCaps)
    {
        RCLCPP_INFO(get_logger(), "needSetCaps  ........!");
        needSetCaps = false;
        appsrc_set_caps(image_msg);
    }

    GstMapInfo info;
    cv::Mat image;
    try {
      image = cv_bridge::toCvShare(image_msg, encoding.c_str())->image; //convert image_msg image -> encoding eg: bgr8 ->rgb8
    } catch (const cv_bridge::Exception &) {
      RCLCPP_ERROR(
        this->get_logger(), "Unable to convert %s image to %s",
        image_msg->encoding.c_str(),encoding.c_str());
      return;
    }

    //if(isFristCall)
    //{
    //    isFristCall = false;
    //    appsrc_set_caps(image_msg);
    //}

    //size_t size = image.total() * image.elemSize();
    size_t size = image.total() * image.elemSize();
    buffer_ = gst_buffer_new_allocate (NULL, size, NULL);
    
    gst_buffer_map(buffer_, &info, GST_MAP_READ);
    memcpy(info.data, image.data, size);
    gst_buffer_unmap(buffer_, &info);

    //RCLCPP_INFO(this->get_logger(), "push buffer size %ld, image code %s....", size, image_msg->encoding.c_str());
    if (gst_app_src_push_buffer(GST_APP_SRC(rtspCtx->appsrc_), buffer_) != GST_FLOW_OK) {
        RCLCPP_ERROR(get_logger(), "gst_app_src_push_buffer(): Error pushing buffer to GStreamer pipeline");
        g_mutex_unlock(&rtspCtx->m_mutex);
        return;
    }
    //gst_buffer_unref (buffer_);
     g_mutex_unlock(&rtspCtx->m_mutex);
}

void Ros22gst::callback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
    if(!rtspReady || !rtspCtx || !rtspCtx->appsrc_)
    {
        //RCLCPP_INFO(get_logger(), "rtsp streamer have not ready or client not connect!");
        return;
    }
    g_mutex_lock(&rtspCtx->m_mutex);

    if(needSetCaps)
    {
        RCLCPP_INFO(get_logger(), "needSetCaps  ........!");
        needSetCaps = false;
        appsrc_set_caps(image_msg);
    }

    /*GstMapInfo info;
    cv::Mat image;
    try {
      image = cv_bridge::toCvShare(image_msg, encoding.c_str())->image; //convert image_msg image -> encoding eg: bgr8 ->rgb8
    } catch (const cv_bridge::Exception &) {
      RCLCPP_ERROR(
        this->get_logger(), "Unable to convert %s image to %s",
        image_msg->encoding.c_str(),encoding.c_str());
      return;
    }*/

    //if(isFristCall)
    //{
    //    isFristCall = false;
    //    appsrc_set_caps(image_msg);
    //}

    //size_t size = image.total() * image.elemSize();
    GstMapInfo info;
    buffer_ = gst_buffer_new_allocate (NULL, image_msg->data.size(), NULL);
    
    gst_buffer_map(buffer_, &info, GST_MAP_READ);
    memcpy(info.data, image_msg->data.data(), image_msg->data.size());
    gst_buffer_unmap(buffer_, &info);
    //RCLCPP_INFO(get_logger(),"size %ld, encoding %s ...", size, );
    //RCLCPP_INFO(this->get_logger(), "push buffer size %ld, image code %s....", image_msg->data.size(), image_msg->encoding.c_str());
    if(!rtspCtx->appsrc_)
        return;
    if (gst_app_src_push_buffer(GST_APP_SRC(rtspCtx->appsrc_), buffer_) != GST_FLOW_OK) {
        RCLCPP_ERROR(get_logger(), "gst_app_src_push_buffer(): Error pushing buffer to GStreamer pipeline");
         g_mutex_unlock(&rtspCtx->m_mutex);
        return;
    }
    //gst_buffer_unref (buffer_);
     g_mutex_unlock(&rtspCtx->m_mutex);;
}
/*
void Ros22gst::callback1(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
    RCLCPP_INFO(node->get_logger(), "callback1");

    GstCaps *caps; // image properties. see return of Image2rtsp::gst_caps_new_from_image
    char *gst_type, *gst_format = (char *)"";
    if (appsrc != NULL){
        // Set caps from message
        caps = gst_caps_new_from_image(msg);
        gst_app_src_set_caps(appsrc, caps);
        buf = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
        gst_buffer_fill(buf, 0, msg->data.data(), msg->data.size());
        GST_BUFFER_FLAG_SET(buf, GST_BUFFER_FLAG_LIVE);
        gst_app_src_push_buffer(appsrc, buf);
    }
}*/

static GstRTSPFilterResult clientFilter(GstRTSPServer* server, GstRTSPClient* client, gpointer user)
{
    return GST_RTSP_FILTER_REMOVE;
}

Ros22gst::~Ros22gst()
{
    RCLCPP_INFO(get_logger(), "Stopping rtsp pipeline_thread_...");
    
    gst_rtsp_server_client_filter(server, clientFilter, nullptr);

    g_main_loop_quit(loop);
    g_main_loop_unref(loop);
    //pipeline_thread_.join();
    pipeline_thread_.join();
    g_mutex_clear(&rtspCtx->m_mutex);
    if (rtspCtx->appsrc_)
    {
        if (gst_app_src_end_of_stream(GST_APP_SRC(rtspCtx->appsrc_)) != GST_FLOW_OK)
        {
            RCLCPP_ERROR(get_logger(),"Cannot send EOS to GStreamer pipeline\n");
            //return;
        }
    }
    g_free (rtspCtx);

    
    gst_deinit();
}


//这样写 ros2节点rclcpp::spin可以收到ctrl+c信号退出，但是gst-launch相关不可收到ctrl+c，使用g_main_loop_quit退出线程循环，但是gst-rtsp管道线程仍然存在需要强制退出
//gst-launch appsrc udp线程置为GST_STATE_NULL相当于释放资源了。




