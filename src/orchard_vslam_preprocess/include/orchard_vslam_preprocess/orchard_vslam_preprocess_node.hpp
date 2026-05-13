#ifndef ORCHARD_VSLAM_PREPROCESS__ORCHARD_VSLAM_PREPROCESS_NODE_HPP_
#define ORCHARD_VSLAM_PREPROCESS__ORCHARD_VSLAM_PREPROCESS_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace orchard_vslam_preprocess
{

class OrchardVslamPreprocessNode : public rclcpp::Node
{
public:
  explicit OrchardVslamPreprocessNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
  using StereoSyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;
  using StereoSynchronizer = message_filters::Synchronizer<StereoSyncPolicy>;

  struct ProcessResult
  {
    cv::Mat output_mono8;
    cv::Mat reliability_mono8;
    cv::Mat debug_bgr8;
    int total_keypoints = 0;
    int selected_keypoints = 0;
    int occupied_grid_cells = 0;
    int total_grid_cells = 0;
    double grid_entropy = 0.0;
    double mean_reliability_score = 0.0;
    double mean_temporal_difference = 0.0;
  };

  void declareParameters();
  void updateRuntimeObjects();
  void stereoCallback(const ImageMsg::ConstSharedPtr & left_msg, const ImageMsg::ConstSharedPtr & right_msg);
  void leftCameraInfoCallback(const CameraInfoMsg::SharedPtr msg) const;
  void rightCameraInfoCallback(const CameraInfoMsg::SharedPtr msg) const;

  ProcessResult processImage(const ImageMsg::ConstSharedPtr & msg, cv::Mat & previous_frame, const std::string & side);

  cv::Mat toMono8(const ImageMsg::ConstSharedPtr & msg) const;
  cv::Mat applyGamma(const cv::Mat & mono8) const;
  cv::Mat computePercentileNormalized(const cv::Mat & src32f, float percentile = 0.95F) const;
  cv::Mat computeLocalVariance(const cv::Mat & mono8) const;
  cv::Mat computeTemporalDifference(const cv::Mat & mono8, cv::Mat & previous_frame) const;
  cv::Mat computeTopPenalty(int rows, int cols) const;
  cv::Mat computeReliabilityMap(
    const cv::Mat & gradient, const cv::Mat & laplacian, const cv::Mat & variance,
    const cv::Mat & temporal, const cv::Mat & top_penalty) const;
  cv::Mat applyReliabilityWeight(const cv::Mat & mono8, const cv::Mat & reliability) const;
  cv::Mat drawDebugKeypoints(const cv::Mat & mono8, const cv::Mat & reliability, ProcessResult & result) const;
  void logMetrics(
    const ProcessResult & left, const ProcessResult & right, double processing_time_ms,
    const rclcpp::Time & stamp);

  bool use_gamma_;
  double gamma_;
  bool use_clahe_;
  double clahe_clip_limit_;
  int clahe_tile_grid_size_;

  double w_sobel_;
  double w_laplacian_;
  double w_variance_;
  double w_temporal_;
  double w_top_;

  double top_penalty_ratio_;
  double min_intensity_weight_;
  double mask_power_;
  int reliability_blur_kernel_size_;
  int local_variance_kernel_size_;

  int grid_rows_;
  int grid_cols_;
  int max_keypoints_per_cell_;
  int orb_nfeatures_;
  int log_interval_;

  cv::Mat gamma_lut_;
  cv::Ptr<cv::CLAHE> clahe_;
  cv::Ptr<cv::ORB> orb_;

  message_filters::Subscriber<ImageMsg> left_image_sub_;
  message_filters::Subscriber<ImageMsg> right_image_sub_;
  std::shared_ptr<StereoSynchronizer> stereo_sync_;

  rclcpp::Subscription<CameraInfoMsg>::SharedPtr left_camera_info_sub_;
  rclcpp::Subscription<CameraInfoMsg>::SharedPtr right_camera_info_sub_;

  rclcpp::Publisher<ImageMsg>::SharedPtr left_image_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr right_image_pub_;
  rclcpp::Publisher<CameraInfoMsg>::SharedPtr left_camera_info_pub_;
  rclcpp::Publisher<CameraInfoMsg>::SharedPtr right_camera_info_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr left_reliability_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr right_reliability_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr left_debug_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr right_debug_pub_;

  cv::Mat previous_left_;
  cv::Mat previous_right_;
  uint64_t processed_frames_;
};

}  // namespace orchard_vslam_preprocess

#endif  // ORCHARD_VSLAM_PREPROCESS__ORCHARD_VSLAM_PREPROCESS_NODE_HPP_
