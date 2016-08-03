#include "yagokoro_utils/pointcloud_to_image_node.hpp"

namespace yagokoro_utils {

PointCloudToImageNode::PointCloudToImageNode(ros::NodeHandle& n,
                                             ros::NodeHandle& np) :
    n_(n), np_(np), it_(np) {
  min_z_ = 0.0;
  max_z_ = 2.0;

  server_ = boost::make_shared
      <dynamic_reconfigure::Server<PointCloudToImageConfig> > (np_);
  dynamic_reconfigure::Server<PointCloudToImageConfig>::CallbackType f =
      boost::bind (&PointCloudToImageNode::ConfigCallback, this, _1, _2);
  server_->setCallback (f);

  pub_color_image_ = it_.advertise("image_color", 1);
  pub_depth_image_ = it_.advertise("image_depth", 1);
  sub_points_ = np_.subscribe("points", 1,
                              &PointCloudToImageNode::PointsCallback, this);
}

PointCloudToImageNode::~PointCloudToImageNode() {
}

void PointCloudToImageNode::ConfigCallback(
    yagokoro_utils::PointCloudToImageConfig& config, uint32_t level) {
  min_z_ = config.min_z;
  max_z_ = config.max_z;

  // max_z_ must be greater than min_z_
  if (max_z_ < min_z_) {
    double tmp = max_z_;
    max_z_ = min_z_;
    min_z_ = tmp;
  }
  // max_z_ must not equals min_z_
  if ((max_z_ - min_z_) < 1e-6) {
    max_z_ = min_z_ + 0.01;
  }
}

void PointCloudToImageNode::PointsCallback(
    const sensor_msgs::PointCloud2ConstPtr& input) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
      cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::fromROSMsg(*input, *cloud);

  cv::Mat image;
  cv::Mat depth;
  PointCloudToCvMat<pcl::PointXYZRGBA>(cloud, image, depth,
                                       min_z_, max_z_);

  std_msgs::Header header;
  header.stamp = input->header.stamp;
  header.frame_id = input->header.frame_id;

  sensor_msgs::ImagePtr color_msg =
      cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  sensor_msgs::ImagePtr depth_msg =
      cv_bridge::CvImage(header, "mono8", depth).toImageMsg();

  pub_color_image_.publish(color_msg);
  pub_depth_image_.publish(depth_msg);
}

}  // namespace

int main (int argc, char** argv) {
  ros::init(argc, argv, "pointcloud_to_image_node");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  yagokoro_utils::PointCloudToImageNode pcltoimg(n, np);
  ros::spin();
}
