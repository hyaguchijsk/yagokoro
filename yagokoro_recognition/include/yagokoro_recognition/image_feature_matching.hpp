#ifndef YAGOKORO_RECOGNITION_IMAGE_FEATURE_MATCHING_HPP_
#define YAGOKORO_RECOGNITION_IMAGE_FEATURE_MATCHING_HPP_

#include <iostream>
#include <vector>
#include <string>

#include <boost/shared_ptr.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>

namespace yagokoro_recognition {

class ImageFeatureMatching {
 public:
  ImageFeatureMatching(const std::string& detector_type,
                       const std::string& extractor_type);
  ~ImageFeatureMatching();


  /// accessors
  /// @brief set input image
  void InputImage(const cv::Mat& img) {
    input_ = img;
  }
  /// @brief get input image
  const cv::Mat& InputImage() {
    return input_;
  }
  /// @brief get keypoints
  std::vector<cv::KeyPoint>& KeyPoints() {
    return keypoints_;
  }
  /// @brief get descriptors
  cv::Mat& Descriptors() {
    return descriptors_;
  }
  /// @brief get matching result
  std::vector<cv::DMatch>& MatchingResult() {
    return feature_matches_;
  }

  /// @brief clear feature points table
  void ClearFeatures() {
    image_vector_.clear();
    keypoints_vector_.clear();
    descriptors_vector_.clear();
  }
  /// @brief add current feature points to table
  void AddCurrentFeatures() {
    image_vector_.push_back(input_);
    keypoints_vector_.push_back(keypoints_);
    descriptors_vector_.push_back(descriptors_);
  }


  /// processing
  /// @brief detect features
  void DetectFeatures() {
    detector_->detect(input_, keypoints_);
  }
  /// @brief extract descriptors
  void ExtractDescriptors() {
    extractor_->compute(input_, keypoints_, descriptors_);
  }
  /// @biref match current features vs feature tables
  size_t MatchFeatures();


 private:
  cv::Mat input_;

  /// feature detection
  cv::Ptr<cv::FeatureDetector> detector_;
  std::vector<cv::KeyPoint> keypoints_;

  /// descriptor extraction
  cv::Ptr<cv::DescriptorExtractor> extractor_;
  cv::Mat descriptors_;

  /// matching
  std::vector<cv::DMatch> feature_matches_;
  /// for feature points
  std::vector<cv::Mat> image_vector_;
  std::vector<std::vector<cv::KeyPoint> > keypoints_vector_;
  std::vector<cv::Mat> descriptors_vector_;
};

typedef boost::shared_ptr<ImageFeatureMatching> ImageFeatureMatchingPtr;

}  // namespace

#endif  // YAGOKORO_RECOGNITION_IMAGE_FEATURE_MATCHING_HPP_
