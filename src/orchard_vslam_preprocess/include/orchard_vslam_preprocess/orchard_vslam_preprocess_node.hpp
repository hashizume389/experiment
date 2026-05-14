#ifndef ORCHARD_VSLAM_PREPROCESS__ORCHARD_VSLAM_PREPROCESS_NODE_HPP_
#define ORCHARD_VSLAM_PREPROCESS__ORCHARD_VSLAM_PREPROCESS_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
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
    cv::Mat current_frame_mono8;
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

  struct ScoredKeypoint
  {
    cv::KeyPoint keypoint;
    float score = 0.0F;
    int cell_index = 0;
    size_t original_index = 0;
    bool selected = false;
  };

  void declareParameters();
  void updateRuntimeObjects();
  void stereoCallback(const ImageMsg::ConstSharedPtr & left_msg, const ImageMsg::ConstSharedPtr & right_msg);
  void leftImageCallback(const ImageMsg::ConstSharedPtr & msg);
  void rightImageCallback(const ImageMsg::ConstSharedPtr & msg);
  void leftCameraInfoCallback(const CameraInfoMsg::SharedPtr msg) const;
  void rightCameraInfoCallback(const CameraInfoMsg::SharedPtr msg) const;

  ProcessResult processImage(const ImageMsg::ConstSharedPtr & msg, const cv::Mat & previous_frame);
  void processAndPublishImage(
    const ImageMsg::ConstSharedPtr & msg, cv::Mat & previous_frame,
    const image_transport::Publisher & image_pub,
    const image_transport::Publisher & reliability_pub,
    const image_transport::Publisher & debug_pub,
    const std::string & side_label);

  cv::Mat toMono8(const ImageMsg::ConstSharedPtr & msg) const;
  cv::Mat applyGamma(const cv::Mat & mono8) const;
  cv::Mat computePercentileNormalized(const cv::Mat & src32f, float percentile = 0.95F) const;
  cv::Mat computeLocalVariance(const cv::Mat & mono8) const;
  cv::Mat computeTemporalDifference(const cv::Mat & mono8, const cv::Mat & previous_frame) const;
  cv::Mat computeTopPenalty(int rows, int cols) const;
  cv::Mat computeReliabilityMap(
    const cv::Mat & gradient, const cv::Mat & laplacian, const cv::Mat & variance,
    const cv::Mat & temporal, const cv::Mat & top_penalty) const;
  void selectKeypointsAndModulate(
    const cv::Mat & mono8, const cv::Mat & gradient, const cv::Mat & laplacian,
    const cv::Mat & variance, const cv::Mat & temporal, const cv::Mat & top_penalty,
    const cv::Mat & reliability, ProcessResult & result);
  std::vector<cv::KeyPoint> detectKeypoints(const cv::Mat & mono8) const;
  cv::Mat buildModulationMap(
    const cv::Size & size, const cv::Mat & reliability,
    const std::vector<ScoredKeypoint> & scored_keypoints) const;
  void logMetrics(
    const ProcessResult & left, const ProcessResult & right, double processing_time_ms,
    const rclcpp::Time & stamp) const;

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
  double w_density_;

  double top_penalty_ratio_;
  int reliability_blur_kernel_size_;
  int local_variance_kernel_size_;

  int grid_rows_;
  int grid_cols_;
  int max_keypoints_per_cell_;
  std::string feature_detector_type_;
  int orb_nfeatures_;
  int fast_threshold_;
  double gaussian_sigma_;
  double keypoint_boost_strength_;
  double min_modulation_weight_;
  double max_modulation_weight_;
  bool publish_debug_;
  bool use_stereo_sync_;
  int log_interval_;

  std::string left_image_topic_;
  std::string right_image_topic_;
  std::string left_camera_info_topic_;
  std::string right_camera_info_topic_;
  std::string left_output_image_topic_;
  std::string right_output_image_topic_;
  std::string left_output_camera_info_topic_;
  std::string right_output_camera_info_topic_;
  std::string left_reliability_topic_;
  std::string right_reliability_topic_;
  std::string left_debug_topic_;
  std::string right_debug_topic_;

  cv::Mat gamma_lut_;
  cv::Ptr<cv::CLAHE> clahe_;
  cv::Ptr<cv::ORB> orb_;
  cv::Ptr<cv::FastFeatureDetector> fast_;

  image_transport::SubscriberFilter left_image_sub_;
  image_transport::SubscriberFilter right_image_sub_;
  std::shared_ptr<StereoSynchronizer> stereo_sync_;

  rclcpp::Subscription<CameraInfoMsg>::SharedPtr left_camera_info_sub_;
  rclcpp::Subscription<CameraInfoMsg>::SharedPtr right_camera_info_sub_;

  image_transport::Publisher left_image_pub_;
  image_transport::Publisher right_image_pub_;
  rclcpp::Publisher<CameraInfoMsg>::SharedPtr left_camera_info_pub_;
  rclcpp::Publisher<CameraInfoMsg>::SharedPtr right_camera_info_pub_;
  image_transport::Publisher left_reliability_pub_;
  image_transport::Publisher right_reliability_pub_;
  image_transport::Publisher left_debug_pub_;
  image_transport::Publisher right_debug_pub_;

  cv::Mat previous_left_;
  cv::Mat previous_right_;
  uint64_t processed_frames_;
};

}  // namespace orchard_vslam_preprocess

#endif  // ORCHARD_VSLAM_PREPROCESS__ORCHARD_VSLAM_PREPROCESS_NODE_HPP_
