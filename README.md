# ros22gst
ros2 sensor_msgs::msg::Image topic to gstreamer pipeline example

ros2 image 主题消息到 gstreamer 例子

configure optional in xml file, 
the topics,width,height,fps options must be the same as publish topic.(appsrc pulgin requirements)

build: 

colcon build 


run:

ros2 launch ros22gst ros22gst.launch.xml
