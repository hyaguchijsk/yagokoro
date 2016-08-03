#ifndef YAGOKORO_UTILS_POINTCLOUD_TO_IMAGE_NODE_HPP_
#define YAGOKORO_UTILS_POINTCLOUD_TO_IMAGE_NODE_HPP_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "yagokoro_utils/PointCloudToImageConfig.h"

#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "yagokoro_utils/pcl_to_cv.hpp"

namespace yagokoro_utils {

class PointCloudToImageNode {
 public:
  PointCloudToImageNode(ros::NodeHandle& n, ros::NodeHandle& np);
  ~PointCloudToImageNode();

  void ConfigCallback(
      yagokoro_utils::PointCloudToImageConfig& config, uint32_t level);
  void PointsCallback(const sensor_msgs::PointCloud2ConstPtr& input);

 private:
  ros::NodeHandle n_;
  ros::NodeHandle np_;
  image_transport::ImageTransport it_;
  ros::Subscriber sub_points_;
  image_transport::Publisher pub_color_image_;
  image_transport::Publisher pub_depth_image_;

  boost::shared_ptr <
    dynamic_reconfigure::Server<PointCloudToImageConfig> > server_;

  double min_z_;
  double max_z_;
};

}  // namespace

#endif  // YAGOKORO_UTILS_POINTCLOUD_TO_IMAGE_NODE_HPP_
