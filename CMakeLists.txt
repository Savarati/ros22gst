cmake_minimum_required(VERSION 3.5)
project(ros22gst)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig)
pkg_search_module(GST REQUIRED gstreamer-1.0)
pkg_search_module(GST_RTSP REQUIRED gstreamer-rtsp-server-1.0)
pkg_search_module(GST_APP REQUIRED gstreamer-app-1.0)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(camera_info_manager REQUIRED)


include_directories(
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${GST_INCLUDE_DIRS}
    ${GLIB_INCLUDE_DIRS}
    ${GST_APP_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(ros22gst SHARED
    src/ros2gst.cpp
)

ament_target_dependencies(ros22gst
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
)

target_link_libraries(ros22gst
   ${Boost_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${GST_LIBRARIES}
   ${GST_APP_LIBRARIES}
   ${GST_RTSP_LIBRARIES}
   ${GSTREAMER_LIBRARIES}
)

add_executable(ros22gst_node src/main.cpp)
target_link_libraries(ros22gst_node ros22gst)


install(TARGETS ros22gst ros22gst_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
	DESTINATION "share/${PROJECT_NAME}"
)

ament_export_include_directories(include)
ament_export_libraries(mavgscam)
#ament_export_dependencies(eigen3_cmake_module)
#ament_export_dependencies(Eigen3)
ament_package()

