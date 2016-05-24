#include "yagokoro_recognition/image_feature_matching.hpp"

namespace yagokoro_recognition {

ImageFeatureMatching::ImageFeatureMatching() {
}
ImageFeatureMatching::~ImageFeatureMatching() {
}

size_t ImageFeatureMatching::MatchFeatures() {
  size_t n_descriptors = descriptors_vector_.size();
  std::vector<cv::DMatch> dmatch;

  cv::Ptr<cv::DescriptorMatcher> matcher;
  if (descriptors_vector_[0].type() == CV_8U) {
          matcher =
              new cv::FlannBasedMatcher(
                  new cv::flann::LshIndexParams(30, 10, 2));
  } else {
    matcher = new cv::FlannBasedMatcher();
  }
  matcher->add(descriptors_vector_);

  matcher->match(descriptors_, dmatch);

  // counting num of matched features
  std::vector<size_t> n_matches_list(n_descriptors, 0);
  for (size_t i = 0; i < dmatch.size(); i++) {
    int32_t iidx = dmatch[i].imgIdx;
    if (iidx >= 0 && iidx < n_descriptors) {
      n_matches_list[iidx]++;
    }
  }

  // seeking max num of features
  int32_t i_max_matches = 0;
  size_t n_max_matches = n_matches_list[i_max_matches];
  for (size_t i = 1; i < n_matches_list.size(); i++) {
    if (n_max_matches < n_matches_list[i]) {
      i_max_matches = i;
      n_max_matches = n_matches_list[i];
    }
  }

  // writing matched features
  feature_matches_.clear();
  for (size_t i = 0; i < dmatch.size(); i++) {
    int32_t iidx = dmatch[i].imgIdx;
    if (iidx == i_max_matches) {
      feature_matches_.push_back(dmatch[i]);
    }
  }

  return i_max_matches;
}


}  // namespace
