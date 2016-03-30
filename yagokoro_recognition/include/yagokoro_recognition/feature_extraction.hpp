#ifndef YAGOKORO_RECOGNITION_FEATURE_EXTRACTION_HPP_
#define YAGOKORO_RECOGNITION_FEATURE_EXTRACTION_HPP_

#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>

using namespace message_filters::sync_policies;

namespace yagokoro_recognition {

typedef ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo> ExactPolicy;
typedef ApproximateTime<sensor_msgs::Image,
                        sensor_msgs::CameraInfo> ApproximatePolicy;
typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

class FeatureExtraction {
 public:
  FeatureExtraction(ros::NodeHandle& n, ros::NodeHandle& np);
  ~FeatureExtraction();

  void ImageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                     const sensor_msgs::CameraInfoConstPtr& info_msg);

 private:
  ros::NodeHandle n_;
  ros::NodeHandle np_;
  image_transport::ImageTransport it_;

  /// using CameraSubscriber
  image_transport::CameraSubscriber sub_camera_;

  /// using sync policy
  image_transport::SubscriberFilter sub_image_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  /// feature extraction
  cv::Ptr<cv::FeatureDetector> detector_;
  cv::Ptr<cv::DescriptorExtractor> extractor_;

  std::vector<cv::KeyPoint> keypoints_;
  cv::Mat descriptors_;
};

}  // namespace

#endif  // YAGOKORO_RECOGNITION_FEATURE_EXTRACTION_HPP_
