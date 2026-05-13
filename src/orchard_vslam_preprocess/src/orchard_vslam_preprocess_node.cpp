#include "orchard_vslam_preprocess/orchard_vslam_preprocess_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <numeric>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>

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

cv::Mat makeImageMessageMat(const cv::Mat & image)
{
  return image.isContinuous() ? image : image.clone();
}
}  // namespace

OrchardVslamPreprocessNode::OrchardVslamPreprocessNode(const rclcpp::NodeOptions & options)
: Node("orchard_vslam_preprocess_node", options),
  left_image_sub_(this, "/camera/camera/infra1/image_rect_raw", rmw_qos_profile_sensor_data),
  right_image_sub_(this, "/camera/camera/infra2/image_rect_raw", rmw_qos_profile_sensor_data),
  processed_frames_(0)
{
  declareParameters();
  updateRuntimeObjects();

  const auto sensor_qos = rclcpp::SensorDataQoS();

  left_image_pub_ = create_publisher<ImageMsg>("/orchard/left/image_preprocessed", sensor_qos);
  right_image_pub_ = create_publisher<ImageMsg>("/orchard/right/image_preprocessed", sensor_qos);
  left_camera_info_pub_ = create_publisher<CameraInfoMsg>("/orchard/left/camera_info", sensor_qos);
  right_camera_info_pub_ = create_publisher<CameraInfoMsg>("/orchard/right/camera_info", sensor_qos);
  left_reliability_pub_ = create_publisher<ImageMsg>("/orchard/left/reliability_map", sensor_qos);
  right_reliability_pub_ = create_publisher<ImageMsg>("/orchard/right/reliability_map", sensor_qos);
  left_debug_pub_ = create_publisher<ImageMsg>("/orchard/left/debug_keypoints", sensor_qos);
  right_debug_pub_ = create_publisher<ImageMsg>("/orchard/right/debug_keypoints", sensor_qos);

  left_camera_info_sub_ = create_subscription<CameraInfoMsg>(
    "/camera/camera/infra1/camera_info", sensor_qos,
    std::bind(&OrchardVslamPreprocessNode::leftCameraInfoCallback, this, std::placeholders::_1));
  right_camera_info_sub_ = create_subscription<CameraInfoMsg>(
    "/camera/camera/infra2/camera_info", sensor_qos,
    std::bind(&OrchardVslamPreprocessNode::rightCameraInfoCallback, this, std::placeholders::_1));

  stereo_sync_ = std::make_shared<StereoSynchronizer>(StereoSyncPolicy(10), left_image_sub_, right_image_sub_);
  stereo_sync_->registerCallback(
    std::bind(&OrchardVslamPreprocessNode::stereoCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "orchard_vslam_preprocess_node started with ApproximateTime stereo synchronization.");
}

void OrchardVslamPreprocessNode::declareParameters()
{
  use_gamma_ = declare_parameter<bool>("use_gamma", true);
  gamma_ = declare_parameter<double>("gamma", 0.8);
  use_clahe_ = declare_parameter<bool>("use_clahe", true);
  clahe_clip_limit_ = declare_parameter<double>("clahe_clip_limit", 2.0);
  clahe_tile_grid_size_ = declare_parameter<int>("clahe_tile_grid_size", 8);

  w_sobel_ = declare_parameter<double>("w_sobel", 0.35);
  w_laplacian_ = declare_parameter<double>("w_laplacian", 0.20);
  w_variance_ = declare_parameter<double>("w_variance", 0.20);
  w_temporal_ = declare_parameter<double>("w_temporal", 0.20);
  w_top_ = declare_parameter<double>("w_top", 0.15);

  top_penalty_ratio_ = declare_parameter<double>("top_penalty_ratio", 0.30);
  min_intensity_weight_ = declare_parameter<double>("min_intensity_weight", 0.35);
  mask_power_ = declare_parameter<double>("mask_power", 1.0);
  reliability_blur_kernel_size_ = declare_parameter<int>("reliability_blur_kernel_size", 3);
  local_variance_kernel_size_ = declare_parameter<int>("local_variance_kernel_size", 9);

  grid_rows_ = declare_parameter<int>("grid_rows", 6);
  grid_cols_ = declare_parameter<int>("grid_cols", 8);
  max_keypoints_per_cell_ = declare_parameter<int>("max_keypoints_per_cell", 30);
  orb_nfeatures_ = declare_parameter<int>("orb_nfeatures", 1500);
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
  log_interval_ = std::max(log_interval_, 1);
  top_penalty_ratio_ = std::clamp(top_penalty_ratio_, 0.0, 1.0);
  min_intensity_weight_ = std::clamp(min_intensity_weight_, 0.0, 1.0);
  mask_power_ = std::max(mask_power_, 0.05);

  gamma_lut_ = cv::Mat(1, 256, CV_8UC1);
  auto * lut = gamma_lut_.ptr<uchar>(0);
  for (int i = 0; i < 256; ++i) {
    const double normalized = static_cast<double>(i) / 255.0;
    lut[i] = cv::saturate_cast<uchar>(std::pow(normalized, gamma_) * 255.0);
  }

  clahe_ = cv::createCLAHE(clahe_clip_limit_, cv::Size(clahe_tile_grid_size_, clahe_tile_grid_size_));
  orb_ = cv::ORB::create(orb_nfeatures_);
}

void OrchardVslamPreprocessNode::stereoCallback(
  const ImageMsg::ConstSharedPtr & left_msg, const ImageMsg::ConstSharedPtr & right_msg)
{
  const auto start = std::chrono::steady_clock::now();

  try {
    ProcessResult left = processImage(left_msg, previous_left_, "left");
    ProcessResult right = processImage(right_msg, previous_right_, "right");

    auto left_out_msg = cv_bridge::CvImage(left_msg->header, sensor_msgs::image_encodings::MONO8,
      makeImageMessageMat(left.output_mono8)).toImageMsg();
    auto right_out_msg = cv_bridge::CvImage(right_msg->header, sensor_msgs::image_encodings::MONO8,
      makeImageMessageMat(right.output_mono8)).toImageMsg();
    auto left_rel_msg = cv_bridge::CvImage(left_msg->header, sensor_msgs::image_encodings::MONO8,
      makeImageMessageMat(left.reliability_mono8)).toImageMsg();
    auto right_rel_msg = cv_bridge::CvImage(right_msg->header, sensor_msgs::image_encodings::MONO8,
      makeImageMessageMat(right.reliability_mono8)).toImageMsg();
    auto left_dbg_msg = cv_bridge::CvImage(left_msg->header, sensor_msgs::image_encodings::BGR8,
      makeImageMessageMat(left.debug_bgr8)).toImageMsg();
    auto right_dbg_msg = cv_bridge::CvImage(right_msg->header, sensor_msgs::image_encodings::BGR8,
      makeImageMessageMat(right.debug_bgr8)).toImageMsg();

    left_image_pub_->publish(*left_out_msg);
    right_image_pub_->publish(*right_out_msg);
    left_reliability_pub_->publish(*left_rel_msg);
    right_reliability_pub_->publish(*right_rel_msg);
    left_debug_pub_->publish(*left_dbg_msg);
    right_debug_pub_->publish(*right_dbg_msg);

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

void OrchardVslamPreprocessNode::leftCameraInfoCallback(const CameraInfoMsg::SharedPtr msg) const
{
  left_camera_info_pub_->publish(*msg);
}

void OrchardVslamPreprocessNode::rightCameraInfoCallback(const CameraInfoMsg::SharedPtr msg) const
{
  right_camera_info_pub_->publish(*msg);
}

OrchardVslamPreprocessNode::ProcessResult OrchardVslamPreprocessNode::processImage(
  const ImageMsg::ConstSharedPtr & msg, cv::Mat & previous_frame, const std::string & side)
{
  (void)side;
  ProcessResult result;
  cv::Mat mono8 = toMono8(msg);

  cv::Mat enhanced = use_gamma_ ? applyGamma(mono8) : mono8;
  if (use_clahe_) {
    cv::Mat clahe_out;
    clahe_->apply(enhanced, clahe_out);
    enhanced = clahe_out;
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
  laplacian_raw = cv::abs(laplacian_raw);
  cv::Mat laplacian = computePercentileNormalized(laplacian_raw);

  cv::Mat variance = computeLocalVariance(enhanced);
  cv::Mat temporal = computeTemporalDifference(enhanced, previous_frame);
  result.mean_temporal_difference = cv::mean(temporal)[0];

  cv::Mat top_penalty = computeTopPenalty(enhanced.rows, enhanced.cols);
  cv::Mat reliability = computeReliabilityMap(gradient, laplacian, variance, temporal, top_penalty);

  result.output_mono8 = applyReliabilityWeight(enhanced, reliability);
  reliability.convertTo(result.reliability_mono8, CV_8U, 255.0);
  result.debug_bgr8 = drawDebugKeypoints(enhanced, reliability, result);
  result.mean_reliability_score = cv::mean(reliability)[0];

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

  cv::Mat converted;
  cv_bridge::CvImageConstPtr mono_ptr =
    cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  mono_ptr->image.copyTo(converted);
  return converted;
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

  const int hist_size = 512;
  const float range[] = {0.0F, static_cast<float>(max_value)};
  const float * hist_range = range;
  cv::Mat hist;
  cv::calcHist(&src32f, 1, 0, cv::Mat(), hist, 1, &hist_size, &hist_range, true, false);

  const float target = static_cast<float>(src32f.total()) * std::clamp(percentile, 0.01F, 0.999F);
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

cv::Mat OrchardVslamPreprocessNode::computeTemporalDifference(const cv::Mat & mono8, cv::Mat & previous_frame) const
{
  cv::Mat temporal;
  if (previous_frame.empty() || previous_frame.size() != mono8.size()) {
    temporal = cv::Mat::zeros(mono8.size(), CV_32F);
  } else {
    cv::Mat diff8;
    cv::absdiff(mono8, previous_frame, diff8);
    cv::Mat diff32;
    diff8.convertTo(diff32, CV_32F, 1.0 / 255.0);
    temporal = computePercentileNormalized(diff32);
  }
  previous_frame = mono8.clone();
  return temporal;
}

cv::Mat OrchardVslamPreprocessNode::computeTopPenalty(int rows, int cols) const
{
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

cv::Mat OrchardVslamPreprocessNode::applyReliabilityWeight(const cv::Mat & mono8, const cv::Mat & reliability) const
{
  cv::Mat mono32;
  mono8.convertTo(mono32, CV_32F);

  cv::Mat powered;
  cv::pow(reliability, mask_power_, powered);
  cv::Mat mask = min_intensity_weight_ + (1.0 - min_intensity_weight_) * powered;
  cv::Mat weighted = mono32.mul(mask);

  cv::Mat output;
  weighted.convertTo(output, CV_8U);
  return output;
}

cv::Mat OrchardVslamPreprocessNode::drawDebugKeypoints(
  const cv::Mat & mono8, const cv::Mat & reliability, ProcessResult & result) const
{
  std::vector<cv::KeyPoint> keypoints;
  orb_->detect(mono8, keypoints);
  result.total_keypoints = static_cast<int>(keypoints.size());
  result.total_grid_cells = grid_rows_ * grid_cols_;

  cv::Mat debug;
  cv::cvtColor(mono8, debug, cv::COLOR_GRAY2BGR);
  if (keypoints.empty()) {
    return debug;
  }

  float max_response = 1.0e-6F;
  for (const auto & keypoint : keypoints) {
    max_response = std::max(max_response, keypoint.response);
  }

  struct ScoredKeypoint
  {
    cv::KeyPoint keypoint;
    float score;
    int cell_index;
    size_t original_index;
  };

  std::vector<std::vector<ScoredKeypoint>> cells(result.total_grid_cells);
  const double cell_width = static_cast<double>(mono8.cols) / static_cast<double>(grid_cols_);
  const double cell_height = static_cast<double>(mono8.rows) / static_cast<double>(grid_rows_);

  for (size_t i = 0; i < keypoints.size(); ++i) {
    const auto & keypoint = keypoints[i];
    const int x = std::clamp(static_cast<int>(std::round(keypoint.pt.x)), 0, reliability.cols - 1);
    const int y = std::clamp(static_cast<int>(std::round(keypoint.pt.y)), 0, reliability.rows - 1);
    const int col = std::clamp(static_cast<int>(keypoint.pt.x / cell_width), 0, grid_cols_ - 1);
    const int row = std::clamp(static_cast<int>(keypoint.pt.y / cell_height), 0, grid_rows_ - 1);
    const int cell_index = row * grid_cols_ + col;
    const float normalized_response = std::clamp(keypoint.response / max_response, 0.0F, 1.0F);
    const float reliability_score = reliability.at<float>(y, x);
    const float score = 0.5F * normalized_response + 0.5F * reliability_score;
    cells[cell_index].push_back({keypoint, score, cell_index, i});
  }

  std::vector<int> selected_counts(result.total_grid_cells, 0);
  std::vector<ScoredKeypoint> selected;
  for (auto & cell : cells) {
    std::sort(cell.begin(), cell.end(), [](const ScoredKeypoint & a, const ScoredKeypoint & b) {
      return a.score > b.score;
    });
    const int count = std::min(max_keypoints_per_cell_, static_cast<int>(cell.size()));
    for (int i = 0; i < count; ++i) {
      selected.push_back(cell[i]);
      selected_counts[cell[i].cell_index]++;
    }
  }

  std::vector<uchar> selected_mask(keypoints.size(), 0);
  for (const auto & scored : selected) {
    selected_mask[scored.original_index] = 1;
  }

  for (size_t i = 0; i < keypoints.size(); ++i) {
    const cv::Scalar color = selected_mask[i] ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 0, 220);
    cv::circle(debug, keypoints[i].pt, 2, color, 1, cv::LINE_AA);
  }

  result.selected_keypoints = static_cast<int>(selected.size());
  result.occupied_grid_cells =
    static_cast<int>(std::count_if(selected_counts.begin(), selected_counts.end(), [](int v) {return v > 0;}));

  if (!selected.empty()) {
    double entropy = 0.0;
    for (const int count : selected_counts) {
      if (count <= 0) {
        continue;
      }
      const double p = static_cast<double>(count) / static_cast<double>(selected.size());
      entropy -= p * std::log(p);
    }
    const double normalizer = std::log(static_cast<double>(result.total_grid_cells));
    result.grid_entropy = normalizer > 0.0 ? entropy / normalizer : 0.0;
  }

  return debug;
}

void OrchardVslamPreprocessNode::logMetrics(
  const ProcessResult & left, const ProcessResult & right, double processing_time_ms,
  const rclcpp::Time & stamp)
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
    "stamp=%.3f proc=%.2f ms | L kp=%d sel=%d ratio=%.2f grid=%.2f entropy=%.2f rel=%.3f temp=%.3f | "
    "R kp=%d sel=%d ratio=%.2f grid=%.2f entropy=%.2f rel=%.3f temp=%.3f",
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
