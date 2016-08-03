#ifndef YAGOKORO_UTILS_PCL_TO_CV_HPP_
#define YAGOKORO_UTILS_PCL_TO_CV_HPP_

#include <iostream>
#include <vector>
#include <string>

#include <boost/shared_ptr.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace yagokoro_utils {

template <typename PointT>
void PointToColorPixel(const PointT& point,
                       cv::Vec3b& pixel) {
  pixel[0] = point.b;
  pixel[1] = point.g;
  pixel[2] = point.r;
}

template <typename PointT>
uint8_t PointToDepthPixel(const PointT& point,
                          double min_z = 0.0,
                          double z_tick = 0.008) {
  uint8_t px;
  px = 0;
  if (!isnan(point.z)) {
    int32_t px_i = 255 - static_cast<int32_t>((point.z - min_z) / z_tick);
    if (px_i < 0) {
      px = 0;
    } else if (px_i > 255) {
      px = 255;
    } else {
      px = static_cast<uint8_t>(px_i);
    }
  }
  return px;
}

template <typename PointT> void PointCloudToCvMat(
  const typename pcl::PointCloud<PointT>::Ptr& cloud,
  cv::Mat& image,
  cv::Mat& depth,
  double min_z = 0.0,
  double max_z = 2.0) {
  size_t width = cloud->width;
  size_t height = cloud->height;
  image.create(cv::Size(width, height), CV_8UC3);
  depth.create(cv::Size(width, height), CV_8UC1);

  double z_tick = (max_z - min_z) / 255.0;

  for (size_t y = 0; y < height; y++) {
    size_t xidx = y * width;
    for (size_t x = 0; x < width; x++) {
      const PointT& pt = cloud->points[xidx + x];
      cv::Vec3b& px = image.at<cv::Vec3b>(y, x);
      PointToColorPixel<PointT>(pt, px);
      depth.at<uint8_t>(y, x) = PointToDepthPixel<PointT>(pt, min_z, z_tick);
    }
  }
}

}  // namespace

#endif  // YAGOKORO_UTILS_PCL_TO_CV_HPP_
