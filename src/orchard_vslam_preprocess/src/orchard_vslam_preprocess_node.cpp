#include "orchard_vslam_preprocess/orchard_vslam_preprocess_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <functional>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace orchard_vslam_preprocess
{
namespace
{
int makeOddAtLeast(int value, int minimum)
{
  value = std::max(value, minimum);
  return (value % 2 == 0) ? value + 1 : value;
}

cv::Mat makeContinuous(const cv::Mat & image)
{
  return image.isContinuous() ? image : image.clone();
}
}  // namespace

OrchardVslamPreprocessNode::OrchardVslamPreprocessNode(const rclcpp::NodeOptions & options)
: Node("orchard_vslam_preprocess_node", options),
  processed_frames_(0),
  left_input_frames_(0),
  right_input_frames_(0),
  stereo_input_frames_(0)
{
  declareParameters();
  updateRuntimeObjects();

  const auto sensor_qos = rclcpp::SensorDataQoS();
  const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  rmw_qos_profile_t output_image_qos = rmw_qos_profile_default;
  output_image_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  output_image_qos.depth = 10;
  output_image_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  output_image_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

  left_image_pub_ = image_transport::create_publisher(this, left_output_image_topic_, output_image_qos);
  right_image_pub_ = image_transport::create_publisher(this, right_output_image_topic_, output_image_qos);
  left_camera_info_pub_ = create_publisher<CameraInfoMsg>(left_output_camera_info_topic_, output_qos);
  right_camera_info_pub_ = create_publisher<CameraInfoMsg>(right_output_camera_info_topic_, output_qos);
  if (publish_reliability_) {
    left_reliability_pub_ = image_transport::create_publisher(this, left_reliability_topic_, output_image_qos);
    right_reliability_pub_ = image_transport::create_publisher(this, right_reliability_topic_, output_image_qos);
  }
  if (publish_debug_) {
    left_debug_pub_ = image_transport::create_publisher(this, left_debug_topic_, output_image_qos);
    right_debug_pub_ = image_transport::create_publisher(this, right_debug_topic_, output_image_qos);
  }

  left_camera_info_sub_ = create_subscription<CameraInfoMsg>(
    left_camera_info_topic_, sensor_qos,
    std::bind(&OrchardVslamPreprocessNode::leftCameraInfoCallback, this, std::placeholders::_1));
  right_camera_info_sub_ = create_subscription<CameraInfoMsg>(
    right_camera_info_topic_, sensor_qos,
    std::bind(&OrchardVslamPreprocessNode::rightCameraInfoCallback, this, std::placeholders::_1));

  left_image_sub_.subscribe(this, left_image_topic_, "raw", rmw_qos_profile_sensor_data);
  right_image_sub_.subscribe(this, right_image_topic_, "raw", rmw_qos_profile_sensor_data);

  if (use_stereo_sync_) {
    stereo_sync_ = std::make_shared<StereoSynchronizer>(StereoSyncPolicy(10), left_image_sub_, right_image_sub_);
    stereo_sync_->registerCallback(
      std::bind(&OrchardVslamPreprocessNode::stereoCallback, this, std::placeholders::_1, std::placeholders::_2));
  } else {
    left_image_sub_.registerCallback(
      std::bind(&OrchardVslamPreprocessNode::leftImageCallback, this, std::placeholders::_1));
    right_image_sub_.registerCallback(
      std::bind(&OrchardVslamPreprocessNode::rightImageCallback, this, std::placeholders::_1));
  }

  RCLCPP_INFO(
    get_logger(),
    "orchard_vslam_preprocess_node started: left=%s right=%s out_left=%s out_right=%s mode=%s publish_every_n_frames=%d",
    left_image_topic_.c_str(), right_image_topic_.c_str(),
    left_output_image_topic_.c_str(), right_output_image_topic_.c_str(),
    use_stereo_sync_ ? "approximate_time_sync" : "independent", publish_every_n_frames_);
}

void OrchardVslamPreprocessNode::declareParameters()
{
  left_image_topic_ = declare_parameter<std::string>(
    "left_image_topic", "/camera/camera/infra1/image_rect_raw");
  right_image_topic_ = declare_parameter<std::string>(
    "right_image_topic", "/camera/camera/infra2/image_rect_raw");
  left_camera_info_topic_ = declare_parameter<std::string>(
    "left_camera_info_topic", "/camera/camera/infra1/camera_info");
  right_camera_info_topic_ = declare_parameter<std::string>(
    "right_camera_info_topic", "/camera/camera/infra2/camera_info");
  left_output_image_topic_ = declare_parameter<std::string>(
    "left_output_image_topic", "/orchard/left/image_preprocessed");
  right_output_image_topic_ = declare_parameter<std::string>(
    "right_output_image_topic", "/orchard/right/image_preprocessed");
  left_output_camera_info_topic_ = declare_parameter<std::string>(
    "left_output_camera_info_topic", "/orchard/left/camera_info");
  right_output_camera_info_topic_ = declare_parameter<std::string>(
    "right_output_camera_info_topic", "/orchard/right/camera_info");
  left_reliability_topic_ = declare_parameter<std::string>(
    "left_reliability_topic", "/orchard/left/reliability_map");
  right_reliability_topic_ = declare_parameter<std::string>(
    "right_reliability_topic", "/orchard/right/reliability_map");
  left_debug_topic_ = declare_parameter<std::string>(
    "left_debug_topic", "/orchard/left/debug_keypoints");
  right_debug_topic_ = declare_parameter<std::string>(
    "right_debug_topic", "/orchard/right/debug_keypoints");

  use_gamma_ = declare_parameter<bool>("use_gamma", true);
  gamma_ = declare_parameter<double>("gamma", 0.8);
  use_clahe_ = declare_parameter<bool>("use_clahe", true);
  clahe_clip_limit_ = declare_parameter<double>("clahe_clip_limit", 2.0);
  clahe_tile_grid_size_ = declare_parameter<int>("clahe_tile_grid_size", 8);

  feature_detector_type_ = declare_parameter<std::string>("feature_detector_type", "ORB");
  orb_nfeatures_ = declare_parameter<int>("orb_nfeatures", 1500);
  fast_threshold_ = declare_parameter<int>("fast_threshold", 20);

  w_sobel_ = declare_parameter<double>("w_sobel", 0.30);
  w_laplacian_ = declare_parameter<double>("w_laplacian", 0.20);
  w_variance_ = declare_parameter<double>("w_variance", 0.20);
  w_temporal_ = declare_parameter<double>("w_temporal", 0.20);
  w_top_ = declare_parameter<double>("w_top", 0.10);
  w_density_ = declare_parameter<double>("w_density", 0.10);

  top_penalty_ratio_ = declare_parameter<double>("top_penalty_ratio", 0.30);
  reliability_blur_kernel_size_ = declare_parameter<int>("reliability_blur_kernel_size", 3);
  local_variance_kernel_size_ = declare_parameter<int>("local_variance_kernel_size", 9);

  grid_rows_ = declare_parameter<int>("grid_rows", 6);
  grid_cols_ = declare_parameter<int>("grid_cols", 8);
  max_keypoints_per_cell_ = declare_parameter<int>("max_keypoints_per_cell", 30);
  gaussian_sigma_ = declare_parameter<double>("gaussian_sigma", 9.0);
  keypoint_boost_strength_ = declare_parameter<double>("keypoint_boost_strength", 0.35);
  min_modulation_weight_ = declare_parameter<double>("min_modulation_weight", 0.60);
  max_modulation_weight_ = declare_parameter<double>("max_modulation_weight", 1.20);
  publish_reliability_ = declare_parameter<bool>("publish_reliability", true);
  publish_debug_ = declare_parameter<bool>("publish_debug", true);
  use_stereo_sync_ = declare_parameter<bool>("use_stereo_sync", false);
  publish_every_n_frames_ = declare_parameter<int>("publish_every_n_frames", 1);
  log_interval_ = declare_parameter<int>("log_interval", 30);
}

void OrchardVslamPreprocessNode::updateRuntimeObjects()
{
  gamma_ = std::max(gamma_, 0.05);
  clahe_tile_grid_size_ = std::max(clahe_tile_grid_size_, 1);
  local_variance_kernel_size_ = makeOddAtLeast(local_variance_kernel_size_, 3);
  reliability_blur_kernel_size_ =
    reliability_blur_kernel_size_ <= 1 ? 0 : makeOddAtLeast(reliability_blur_kernel_size_, 3);
  grid_rows_ = std::max(grid_rows_, 1);
  grid_cols_ = std::max(grid_cols_, 1);
  max_keypoints_per_cell_ = std::max(max_keypoints_per_cell_, 1);
  orb_nfeatures_ = std::max(orb_nfeatures_, 1);
  fast_threshold_ = std::max(fast_threshold_, 1);
  gaussian_sigma_ = std::max(gaussian_sigma_, 1.0);
  keypoint_boost_strength_ = std::max(keypoint_boost_strength_, 0.0);
  min_modulation_weight_ = std::clamp(min_modulation_weight_, 0.0, 1.0);
  max_modulation_weight_ = std::max(max_modulation_weight_, min_modulation_weight_);
  publish_every_n_frames_ = std::max(publish_every_n_frames_, 1);
  log_interval_ = std::max(log_interval_, 1);
  top_penalty_ratio_ = std::clamp(top_penalty_ratio_, 0.0, 1.0);
  std::transform(
    feature_detector_type_.begin(), feature_detector_type_.end(), feature_detector_type_.begin(),
    [](unsigned char c) {return static_cast<char>(std::toupper(c));});

  gamma_lut_ = cv::Mat(1, 256, CV_8UC1);
  auto * lut = gamma_lut_.ptr<uchar>(0);
  for (int i = 0; i < 256; ++i) {
    const double normalized = static_cast<double>(i) / 255.0;
    lut[i] = cv::saturate_cast<uchar>(std::pow(normalized, gamma_) * 255.0);
  }

  clahe_ = cv::createCLAHE(clahe_clip_limit_, cv::Size(clahe_tile_grid_size_, clahe_tile_grid_size_));
  orb_ = cv::ORB::create(orb_nfeatures_);
  fast_ = cv::FastFeatureDetector::create(fast_threshold_, true);
}

void OrchardVslamPreprocessNode::stereoCallback(
  const ImageMsg::ConstSharedPtr & left_msg, const ImageMsg::ConstSharedPtr & right_msg)
{
  if (!shouldProcessFrame(stereo_input_frames_)) {
    return;
  }

  const auto start = std::chrono::steady_clock::now();

  try {
    ProcessResult left = processImage(left_msg, previous_left_);
    previous_left_ = left.current_frame_mono8.clone();
    ProcessResult right = processImage(right_msg, previous_right_);
    previous_right_ = right.current_frame_mono8.clone();

    auto left_out_msg = cv_bridge::CvImage(
      left_msg->header, sensor_msgs::image_encodings::MONO8, makeContinuous(left.output_mono8)).toImageMsg();
    auto right_out_msg = cv_bridge::CvImage(
      right_msg->header, sensor_msgs::image_encodings::MONO8, makeContinuous(right.output_mono8)).toImageMsg();
    left_image_pub_.publish(*left_out_msg);
    right_image_pub_.publish(*right_out_msg);
    if (publish_reliability_) {
      auto left_rel_msg = cv_bridge::CvImage(
        left_msg->header, sensor_msgs::image_encodings::MONO8, makeContinuous(left.reliability_mono8)).toImageMsg();
      auto right_rel_msg = cv_bridge::CvImage(
        right_msg->header, sensor_msgs::image_encodings::MONO8, makeContinuous(right.reliability_mono8)).toImageMsg();
      left_reliability_pub_.publish(*left_rel_msg);
      right_reliability_pub_.publish(*right_rel_msg);
    }
    if (publish_debug_) {
      auto left_dbg_msg = cv_bridge::CvImage(
        left_msg->header, sensor_msgs::image_encodings::BGR8, makeContinuous(left.debug_bgr8)).toImageMsg();
      auto right_dbg_msg = cv_bridge::CvImage(
        right_msg->header, sensor_msgs::image_encodings::BGR8, makeContinuous(right.debug_bgr8)).toImageMsg();
      left_debug_pub_.publish(*left_dbg_msg);
      right_debug_pub_.publish(*right_dbg_msg);
    }

    const auto end = std::chrono::steady_clock::now();
    const double processing_time_ms =
      std::chrono::duration<double, std::milli>(end - start).count();
    ++processed_frames_;
    if (processed_frames_ % static_cast<uint64_t>(log_interval_) == 0) {
      logMetrics(left, right, processing_time_ms, left_msg->header.stamp);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Image preprocessing failed: %s", e.what());
  }
}

void OrchardVslamPreprocessNode::leftImageCallback(const ImageMsg::ConstSharedPtr & msg)
{
  if (!shouldProcessFrame(left_input_frames_)) {
    return;
  }

  processAndPublishImage(
    msg, previous_left_, left_image_pub_, left_reliability_pub_, left_debug_pub_, "L");
}

void OrchardVslamPreprocessNode::rightImageCallback(const ImageMsg::ConstSharedPtr & msg)
{
  if (!shouldProcessFrame(right_input_frames_)) {
    return;
  }

  processAndPublishImage(
    msg, previous_right_, right_image_pub_, right_reliability_pub_, right_debug_pub_, "R");
}

bool OrchardVslamPreprocessNode::shouldProcessFrame(uint64_t & frame_count) const
{
  const uint64_t current = frame_count++;
  return current % static_cast<uint64_t>(publish_every_n_frames_) == 0;
}

void OrchardVslamPreprocessNode::processAndPublishImage(
  const ImageMsg::ConstSharedPtr & msg, cv::Mat & previous_frame,
  const image_transport::Publisher & image_pub,
  const image_transport::Publisher & reliability_pub,
  const image_transport::Publisher & debug_pub,
  const std::string & side_label)
{
  const auto start = std::chrono::steady_clock::now();

  try {
    ProcessResult result = processImage(msg, previous_frame);
    previous_frame = result.current_frame_mono8.clone();

    auto out_msg = cv_bridge::CvImage(
      msg->header, sensor_msgs::image_encodings::MONO8, makeContinuous(result.output_mono8)).toImageMsg();
    image_pub.publish(*out_msg);
    if (publish_reliability_) {
      auto rel_msg = cv_bridge::CvImage(
        msg->header, sensor_msgs::image_encodings::MONO8, makeContinuous(result.reliability_mono8)).toImageMsg();
      reliability_pub.publish(*rel_msg);
    }

    if (publish_debug_) {
      auto dbg_msg = cv_bridge::CvImage(
        msg->header, sensor_msgs::image_encodings::BGR8, makeContinuous(result.debug_bgr8)).toImageMsg();
      debug_pub.publish(*dbg_msg);
    }

    const auto end = std::chrono::steady_clock::now();
    const double processing_time_ms =
      std::chrono::duration<double, std::milli>(end - start).count();
    ++processed_frames_;
    if (processed_frames_ % static_cast<uint64_t>(log_interval_) == 0) {
      const double selected_ratio = result.total_keypoints > 0 ?
        static_cast<double>(result.selected_keypoints) / static_cast<double>(result.total_keypoints) : 0.0;
      const double occupied_ratio = result.total_grid_cells > 0 ?
        static_cast<double>(result.occupied_grid_cells) / static_cast<double>(result.total_grid_cells) : 0.0;
      RCLCPP_INFO(
        get_logger(),
        "stamp=%.3f side=%s processing_time_ms=%.2f total_keypoints=%d selected_keypoints=%d "
        "selected_ratio=%.2f occupied_grid_cells_ratio=%.2f normalized_grid_entropy=%.2f "
        "mean_reliability_score=%.3f mean_temporal_difference=%.3f",
        rclcpp::Time(msg->header.stamp).seconds(), side_label.c_str(), processing_time_ms,
        result.total_keypoints, result.selected_keypoints, selected_ratio, occupied_ratio,
        result.grid_entropy, result.mean_reliability_score, result.mean_temporal_difference);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "%s image preprocessing failed: %s", side_label.c_str(), e.what());
  }
}

void OrchardVslamPreprocessNode::leftCameraInfoCallback(const CameraInfoMsg::SharedPtr msg) const
{
  left_camera_info_pub_->publish(*msg);
}

void OrchardVslamPreprocessNode::rightCameraInfoCallback(const CameraInfoMsg::SharedPtr msg) const
{
  right_camera_info_pub_->publish(*msg);
}

OrchardVslamPreprocessNode::ProcessResult OrchardVslamPreprocessNode::processImage(
  const ImageMsg::ConstSharedPtr & msg, const cv::Mat & previous_frame)
{
  ProcessResult result;
  cv::Mat mono8 = toMono8(msg);

  cv::Mat enhanced = use_gamma_ ? applyGamma(mono8) : mono8;
  if (use_clahe_) {
    cv::Mat clahe_out;
    clahe_->apply(enhanced, clahe_out);
    enhanced = std::move(clahe_out);
  }

  cv::Mat enhanced32;
  enhanced.convertTo(enhanced32, CV_32F, 1.0 / 255.0);

  cv::Mat sobel_x;
  cv::Mat sobel_y;
  cv::Sobel(enhanced32, sobel_x, CV_32F, 1, 0, 3);
  cv::Sobel(enhanced32, sobel_y, CV_32F, 0, 1, 3);
  cv::Mat gradient_mag;
  cv::magnitude(sobel_x, sobel_y, gradient_mag);
  cv::Mat gradient = computePercentileNormalized(gradient_mag);

  cv::Mat laplacian_raw;
  cv::Laplacian(enhanced32, laplacian_raw, CV_32F, 3);
  cv::Mat laplacian = computePercentileNormalized(cv::abs(laplacian_raw));

  cv::Mat variance = computeLocalVariance(enhanced);
  cv::Mat temporal = computeTemporalDifference(enhanced, previous_frame);
  result.mean_temporal_difference = cv::mean(temporal)[0];

  cv::Mat top_penalty = computeTopPenalty(enhanced.rows, enhanced.cols);
  cv::Mat reliability = computeReliabilityMap(gradient, laplacian, variance, temporal, top_penalty);

  result.current_frame_mono8 = enhanced;
  reliability.convertTo(result.reliability_mono8, CV_8U, 255.0);
  result.mean_reliability_score = cv::mean(reliability)[0];
  selectKeypointsAndModulate(
    enhanced, gradient, laplacian, variance, temporal, top_penalty, reliability, result);
  return result;
}

cv::Mat OrchardVslamPreprocessNode::toMono8(const ImageMsg::ConstSharedPtr & msg) const
{
  const auto cv_ptr = cv_bridge::toCvShare(msg);
  const std::string & encoding = msg->encoding;

  if (encoding == sensor_msgs::image_encodings::MONO8 || encoding == "8UC1") {
    return cv_ptr->image.clone();
  }
  if (encoding == sensor_msgs::image_encodings::BGR8) {
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    return gray;
  }
  if (encoding == sensor_msgs::image_encodings::RGB8) {
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_RGB2GRAY);
    return gray;
  }
  if (encoding == sensor_msgs::image_encodings::MONO16 || encoding == "16UC1") {
    cv::Mat gray;
    cv_ptr->image.convertTo(gray, CV_8U, 1.0 / 256.0);
    return gray;
  }

  cv_bridge::CvImageConstPtr mono_ptr =
    cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  return mono_ptr->image.clone();
}

cv::Mat OrchardVslamPreprocessNode::applyGamma(const cv::Mat & mono8) const
{
  cv::Mat corrected;
  cv::LUT(mono8, gamma_lut_, corrected);
  return corrected;
}

cv::Mat OrchardVslamPreprocessNode::computePercentileNormalized(const cv::Mat & src32f, float percentile) const
{
  CV_Assert(src32f.type() == CV_32F);

  double max_value = 0.0;
  cv::minMaxLoc(src32f, nullptr, &max_value);
  if (max_value <= 1.0e-6) {
    return cv::Mat::zeros(src32f.size(), CV_32F);
  }

  percentile = std::clamp(percentile, 0.01F, 0.999F);
  const int hist_size = 512;
  const float range[] = {0.0F, static_cast<float>(max_value)};
  const float * hist_range = range;
  cv::Mat hist;
  cv::calcHist(&src32f, 1, 0, cv::Mat(), hist, 1, &hist_size, &hist_range, true, false);

  const float target = static_cast<float>(src32f.total()) * percentile;
  float cumulative = 0.0F;
  int threshold_bin = hist_size - 1;
  for (int i = 0; i < hist_size; ++i) {
    cumulative += hist.at<float>(i);
    if (cumulative >= target) {
      threshold_bin = i;
      break;
    }
  }

  const float clip_value =
    std::max((static_cast<float>(threshold_bin + 1) / static_cast<float>(hist_size)) *
    static_cast<float>(max_value), 1.0e-6F);

  cv::Mat clipped;
  cv::min(src32f, clip_value, clipped);
  clipped.convertTo(clipped, CV_32F, 1.0 / clip_value);
  cv::threshold(clipped, clipped, 1.0, 1.0, cv::THRESH_TRUNC);
  cv::threshold(clipped, clipped, 0.0, 0.0, cv::THRESH_TOZERO);
  return clipped;
}

cv::Mat OrchardVslamPreprocessNode::computeLocalVariance(const cv::Mat & mono8) const
{
  cv::Mat float_image;
  mono8.convertTo(float_image, CV_32F, 1.0 / 255.0);

  cv::Mat mean;
  cv::Mat mean_sq;
  cv::Mat sq = float_image.mul(float_image);
  const cv::Size kernel(local_variance_kernel_size_, local_variance_kernel_size_);
  cv::blur(float_image, mean, kernel);
  cv::blur(sq, mean_sq, kernel);

  cv::Mat variance = mean_sq - mean.mul(mean);
  cv::threshold(variance, variance, 0.0, 0.0, cv::THRESH_TOZERO);
  return computePercentileNormalized(variance);
}

cv::Mat OrchardVslamPreprocessNode::computeTemporalDifference(
  const cv::Mat & mono8, const cv::Mat & previous_frame) const
{
  if (previous_frame.empty() || previous_frame.size() != mono8.size()) {
    return cv::Mat::zeros(mono8.size(), CV_32F);
  }

  cv::Mat diff8;
  cv::absdiff(mono8, previous_frame, diff8);
  cv::Mat diff32;
  diff8.convertTo(diff32, CV_32F, 1.0 / 255.0);
  return computePercentileNormalized(diff32);
}

cv::Mat OrchardVslamPreprocessNode::computeTopPenalty(int rows, int cols) const
{
  (void)cols;
  cv::Mat penalty(rows, cols, CV_32F, cv::Scalar(0.0F));
  const int penalty_rows = static_cast<int>(std::round(static_cast<double>(rows) * top_penalty_ratio_));
  if (penalty_rows <= 0) {
    return penalty;
  }

  for (int y = 0; y < std::min(rows, penalty_rows); ++y) {
    const float value = 1.0F - static_cast<float>(y) / static_cast<float>(penalty_rows);
    penalty.row(y).setTo(value);
  }
  return penalty;
}

cv::Mat OrchardVslamPreprocessNode::computeReliabilityMap(
  const cv::Mat & gradient, const cv::Mat & laplacian, const cv::Mat & variance,
  const cv::Mat & temporal, const cv::Mat & top_penalty) const
{
  cv::Mat reliability =
    w_sobel_ * gradient + w_laplacian_ * laplacian + w_variance_ * variance -
    w_temporal_ * temporal - w_top_ * top_penalty;

  const double positive_weight_sum = std::max(w_sobel_ + w_laplacian_ + w_variance_, 1.0e-6);
  reliability.convertTo(reliability, CV_32F, 1.0 / positive_weight_sum);
  cv::threshold(reliability, reliability, 0.0, 0.0, cv::THRESH_TOZERO);
  cv::threshold(reliability, reliability, 1.0, 1.0, cv::THRESH_TRUNC);

  if (reliability_blur_kernel_size_ > 1) {
    cv::GaussianBlur(
      reliability, reliability,
      cv::Size(reliability_blur_kernel_size_, reliability_blur_kernel_size_), 0.0);
    cv::threshold(reliability, reliability, 1.0, 1.0, cv::THRESH_TRUNC);
  }
  return reliability;
}

std::vector<cv::KeyPoint> OrchardVslamPreprocessNode::detectKeypoints(const cv::Mat & mono8) const
{
  std::vector<cv::KeyPoint> keypoints;
  if (feature_detector_type_ == "FAST") {
    fast_->detect(mono8, keypoints);
  } else {
    orb_->detect(mono8, keypoints);
  }
  return keypoints;
}

cv::Mat OrchardVslamPreprocessNode::buildModulationMap(
  const cv::Size & size, const cv::Mat & reliability,
  const std::vector<ScoredKeypoint> & scored_keypoints) const
{
  cv::Mat modulation;
  reliability.convertTo(
    modulation, CV_32F, 1.0 - min_modulation_weight_, min_modulation_weight_);

  const int radius = std::max(1, static_cast<int>(std::ceil(gaussian_sigma_ * 3.0)));
  const double inv_two_sigma_sq = 1.0 / (2.0 * gaussian_sigma_ * gaussian_sigma_);

  for (const auto & scored : scored_keypoints) {
    if (!scored.selected) {
      continue;
    }
    const int cx = static_cast<int>(std::round(scored.keypoint.pt.x));
    const int cy = static_cast<int>(std::round(scored.keypoint.pt.y));
    const int x0 = std::max(0, cx - radius);
    const int x1 = std::min(size.width - 1, cx + radius);
    const int y0 = std::max(0, cy - radius);
    const int y1 = std::min(size.height - 1, cy + radius);
    const float boost = static_cast<float>(keypoint_boost_strength_ * scored.score);

    for (int y = y0; y <= y1; ++y) {
      float * row = modulation.ptr<float>(y);
      const int dy = y - cy;
      for (int x = x0; x <= x1; ++x) {
        const int dx = x - cx;
        const float gaussian =
          static_cast<float>(std::exp(-static_cast<double>(dx * dx + dy * dy) * inv_two_sigma_sq));
        row[x] += boost * gaussian;
      }
    }
  }

  cv::threshold(modulation, modulation, max_modulation_weight_, max_modulation_weight_, cv::THRESH_TRUNC);
  cv::threshold(modulation, modulation, min_modulation_weight_, min_modulation_weight_, cv::THRESH_TOZERO);
  cv::max(modulation, min_modulation_weight_, modulation);
  return modulation;
}

void OrchardVslamPreprocessNode::selectKeypointsAndModulate(
  const cv::Mat & mono8, const cv::Mat & gradient, const cv::Mat & laplacian,
  const cv::Mat & variance, const cv::Mat & temporal, const cv::Mat & top_penalty,
  const cv::Mat & reliability, ProcessResult & result)
{
  std::vector<cv::KeyPoint> keypoints = detectKeypoints(mono8);
  result.total_keypoints = static_cast<int>(keypoints.size());
  result.total_grid_cells = grid_rows_ * grid_cols_;
  if (publish_debug_) {
    cv::cvtColor(mono8, result.debug_bgr8, cv::COLOR_GRAY2BGR);
  }

  if (keypoints.empty()) {
    cv::Mat modulation = buildModulationMap(mono8.size(), reliability, {});
    cv::Mat mono32;
    mono8.convertTo(mono32, CV_32F);
    cv::multiply(mono32, modulation, mono32);
    mono32.convertTo(result.output_mono8, CV_8U);
    return;
  }

  const double cell_width = static_cast<double>(mono8.cols) / static_cast<double>(grid_cols_);
  const double cell_height = static_cast<double>(mono8.rows) / static_cast<double>(grid_rows_);
  std::vector<int> candidate_counts(result.total_grid_cells, 0);

  std::vector<ScoredKeypoint> scored;
  scored.reserve(keypoints.size());
  for (size_t i = 0; i < keypoints.size(); ++i) {
    const auto & keypoint = keypoints[i];
    const int col = std::clamp(static_cast<int>(keypoint.pt.x / cell_width), 0, grid_cols_ - 1);
    const int row = std::clamp(static_cast<int>(keypoint.pt.y / cell_height), 0, grid_rows_ - 1);
    const int cell_index = row * grid_cols_ + col;
    candidate_counts[cell_index]++;
    scored.push_back({keypoint, 0.0F, cell_index, i, false});
  }

  std::vector<float> raw_scores(scored.size(), 0.0F);
  float min_score = std::numeric_limits<float>::max();
  float max_score = std::numeric_limits<float>::lowest();
  for (size_t i = 0; i < scored.size(); ++i) {
    const auto & keypoint = scored[i].keypoint;
    const int x = std::clamp(static_cast<int>(std::round(keypoint.pt.x)), 0, reliability.cols - 1);
    const int y = std::clamp(static_cast<int>(std::round(keypoint.pt.y)), 0, reliability.rows - 1);
    const float density = std::clamp(
      static_cast<float>(candidate_counts[scored[i].cell_index]) /
      static_cast<float>(std::max(max_keypoints_per_cell_, 1)), 0.0F, 1.0F);
    const float raw =
      static_cast<float>(
      w_sobel_ * gradient.at<float>(y, x) +
      w_laplacian_ * laplacian.at<float>(y, x) +
      w_variance_ * variance.at<float>(y, x) -
      w_temporal_ * temporal.at<float>(y, x) -
      w_top_ * top_penalty.at<float>(y, x) -
      w_density_ * density);
    raw_scores[i] = raw;
    min_score = std::min(min_score, raw);
    max_score = std::max(max_score, raw);
  }

  const float range = max_score - min_score;
  std::vector<std::vector<size_t>> cells(result.total_grid_cells);
  for (size_t i = 0; i < scored.size(); ++i) {
    if (range > 1.0e-6F) {
      scored[i].score = std::clamp((raw_scores[i] - min_score) / range, 0.0F, 1.0F);
    } else {
      const int x = std::clamp(
        static_cast<int>(std::round(scored[i].keypoint.pt.x)), 0, reliability.cols - 1);
      const int y = std::clamp(
        static_cast<int>(std::round(scored[i].keypoint.pt.y)), 0, reliability.rows - 1);
      scored[i].score = reliability.at<float>(y, x);
    }
    cells[scored[i].cell_index].push_back(i);
  }

  std::vector<int> selected_counts(result.total_grid_cells, 0);
  for (auto & cell : cells) {
    std::sort(cell.begin(), cell.end(), [&scored](size_t a, size_t b) {
      return scored[a].score > scored[b].score;
    });
    const int count = std::min(max_keypoints_per_cell_, static_cast<int>(cell.size()));
    for (int i = 0; i < count; ++i) {
      scored[cell[i]].selected = true;
      selected_counts[scored[cell[i]].cell_index]++;
    }
  }

  result.selected_keypoints =
    static_cast<int>(std::count_if(scored.begin(), scored.end(), [](const ScoredKeypoint & keypoint) {
      return keypoint.selected;
    }));

  if (publish_debug_) {
    for (const auto & scored_keypoint : scored) {
      const auto center = cv::Point(
        static_cast<int>(std::round(scored_keypoint.keypoint.pt.x)),
        static_cast<int>(std::round(scored_keypoint.keypoint.pt.y)));
      const cv::Scalar color = scored_keypoint.selected ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 0, 220);
      cv::circle(result.debug_bgr8, center, 2, color, -1, cv::LINE_AA);
    }
  }

  result.occupied_grid_cells =
    static_cast<int>(std::count_if(selected_counts.begin(), selected_counts.end(), [](int v) {return v > 0;}));

  if (result.selected_keypoints > 0) {
    double entropy = 0.0;
    for (const int count : selected_counts) {
      if (count <= 0) {
        continue;
      }
      const double p = static_cast<double>(count) / static_cast<double>(result.selected_keypoints);
      entropy -= p * std::log(p);
    }
    const double normalizer = std::log(static_cast<double>(result.total_grid_cells));
    result.grid_entropy = normalizer > 0.0 ? entropy / normalizer : 0.0;
  }

  cv::Mat modulation = buildModulationMap(mono8.size(), reliability, scored);
  cv::Mat mono32;
  mono8.convertTo(mono32, CV_32F);
  cv::multiply(mono32, modulation, mono32);
  mono32.convertTo(result.output_mono8, CV_8U);
}

void OrchardVslamPreprocessNode::logMetrics(
  const ProcessResult & left, const ProcessResult & right, double processing_time_ms,
  const rclcpp::Time & stamp) const
{
  const auto ratio = [](const ProcessResult & result) {
      if (result.total_keypoints <= 0) {
        return 0.0;
      }
      return static_cast<double>(result.selected_keypoints) / static_cast<double>(result.total_keypoints);
    };
  const auto grid_ratio = [](const ProcessResult & result) {
      if (result.total_grid_cells <= 0) {
        return 0.0;
      }
      return static_cast<double>(result.occupied_grid_cells) / static_cast<double>(result.total_grid_cells);
    };

  RCLCPP_INFO(
    get_logger(),
    "stamp=%.3f processing_time_ms=%.2f | L total_keypoints=%d selected_keypoints=%d "
    "selected_ratio=%.2f occupied_grid_cells_ratio=%.2f normalized_grid_entropy=%.2f "
    "mean_reliability_score=%.3f mean_temporal_difference=%.3f | R total_keypoints=%d "
    "selected_keypoints=%d selected_ratio=%.2f occupied_grid_cells_ratio=%.2f "
    "normalized_grid_entropy=%.2f mean_reliability_score=%.3f mean_temporal_difference=%.3f",
    stamp.seconds(), processing_time_ms,
    left.total_keypoints, left.selected_keypoints, ratio(left), grid_ratio(left), left.grid_entropy,
    left.mean_reliability_score, left.mean_temporal_difference,
    right.total_keypoints, right.selected_keypoints, ratio(right), grid_ratio(right), right.grid_entropy,
    right.mean_reliability_score, right.mean_temporal_difference);
}

}  // namespace orchard_vslam_preprocess

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<orchard_vslam_preprocess::OrchardVslamPreprocessNode>());
  rclcpp::shutdown();
  return 0;
}
