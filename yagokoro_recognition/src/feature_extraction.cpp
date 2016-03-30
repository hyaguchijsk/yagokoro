#include "yagokoro_recognition/feature_extraction.hpp"

namespace yagokoro_recognition {

FeatureExtraction::FeatureExtraction(ros::NodeHandle& n, ros::NodeHandle& np)
    : n_(n), np_(np), it_(n) {
  /// feature extractor
  detector_ = cv::FeatureDetector::create("GridFAST");
  extractor_ = cv::DescriptorExtractor::create("OpponentBRIEF");

  /// subscriber
  image_transport::TransportHints hints("raw", ros::TransportHints(), np_);

  /// using CameraSubscriber
  sub_camera_ =
      it_.subscribeCamera("image", 1,
                          &FeatureExtraction::ImageCallback, this, hints);

  /// using sync policy
  // sub_image_.subscribe(it_, "image", 1, hints);
  // sub_info_ .subscribe(n_, "camera_info", 1);
  // approximate_sync_.reset(
  //     new ApproximateSync(ApproximatePolicy(10),
  //                         sub_image_, sub_info_));
  // approximate_sync_->registerCallback(
  //     boost::bind(&FeatureExtraction::ImageCallback, this, _1, _2));
}

FeatureExtraction::~FeatureExtraction() {
}

void FeatureExtraction::ImageCallback(
    const sensor_msgs::ImageConstPtr& image_msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg) {
  try {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat image;

    cv_ptr = cv_bridge::toCvCopy(image_msg,
                                 sensor_msgs::image_encodings::BGR8);
    cv_ptr->image.copyTo(image);

    detector_->detect(image, keypoints_);
    extractor_->compute(image, keypoints_, descriptors_);

    cv::Mat out_image;
    cv::drawKeypoints(image, keypoints_, out_image);
    cv::imshow("FeatureExtraction", out_image);
  } catch (cv_bridge::Exception& ex) {
    ROS_ERROR("%s", ex.what());
  }

  /// for cv::imshow
  cv::waitKey(10);
}


}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv,"feature_extraction");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  yagokoro_recognition::FeatureExtraction fe(n, np);

  ros::spin();

  return 0;
}
