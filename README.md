# ros22gst
ros2 sensor_msgs::msg::Image topic to gstreamer pipeline example

Data flow :

ros2 image topic --> gstreamer udpstream --> gstramer rtspstream --> QGC

configure optional in xml file,
the topics, width, height, fps, options must be the same as publish topic.(appsrc pulgin requirements)

encoding : there are two options. rgb8: colour picture   mono8: black/white picture

build: 

colcon build 


run:

ros2 launch ros22gst ros22gst.launch.xml
