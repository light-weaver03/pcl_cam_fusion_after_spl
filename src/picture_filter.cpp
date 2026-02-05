#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <deque>
#include <vector>
#include <limits>
#include <cmath>
#include <string>
#include <algorithm>
#include <numeric>

class PictureFilterNode : public rclcpp::Node {
public:
  PictureFilterNode() : Node("picture_filter_node") {
    // params
    declare_parameter<std::string>("image_in", "/image_raw");
    declare_parameter<std::string>("cloud_in", "/cloud_registered");
    declare_parameter<std::string>("image_out", "/image_nearest");
    declare_parameter<double>("max_dt", 0.3);
    declare_parameter<int>("buffer_size", 200);

    // diagnostics
    declare_parameter<int>("diag_every_n", 50);                 // 每N帧输出一次统计（image侧）
    declare_parameter<int>("offset_window", 80);               // 统计最近N个 cloud 匹配的 signed_dt
    declare_parameter<double>("warn_gap_image", 0.08);          // image stamp gap 超过该值认为“断流/跳变”
    declare_parameter<double>("warn_gap_cloud", 0.20);          // cloud stamp gap 超过该值认为异常
    declare_parameter<bool>("store_stamp_sorted", true);        // 是否按 stamp 排序插入（默认true）

    image_in_  = get_parameter("image_in").as_string();
    cloud_in_  = get_parameter("cloud_in").as_string();
    image_out_ = get_parameter("image_out").as_string();
    max_dt_    = get_parameter("max_dt").as_double();
    buf_max_   = static_cast<size_t>(get_parameter("buffer_size").as_int());

    diag_every_n_   = get_parameter("diag_every_n").as_int();
    offset_window_  = get_parameter("offset_window").as_int();
    warn_gap_img_   = get_parameter("warn_gap_image").as_double();
    warn_gap_cloud_ = get_parameter("warn_gap_cloud").as_double();
    store_stamp_sorted_ = get_parameter("store_stamp_sorted").as_bool();

    // pubs/subs
    pub_img_ = create_publisher<sensor_msgs::msg::Image>(image_out_, rclcpp::SensorDataQoS());

    sub_img_ = create_subscription<sensor_msgs::msg::Image>(
      image_in_, rclcpp::SensorDataQoS(),
      std::bind(&PictureFilterNode::onImage, this, std::placeholders::_1));

    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_in_, rclcpp::SensorDataQoS(),
      std::bind(&PictureFilterNode::onCloud, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "image_in=%s cloud_in=%s image_out=%s max_dt=%.3f buf=%zu diag_every_n=%d offset_window=%d",
      image_in_.c_str(), cloud_in_.c_str(), image_out_.c_str(),
      max_dt_, buf_max_, diag_every_n_, offset_window_);
  }

private:
  struct ImgItem {
    rclcpp::Time stamp; // 这里刻意使用 header.stamp（不是 now），用于判断时间戳连续性与对齐
    sensor_msgs::msg::Image::SharedPtr msg;
  };

  // 图像缓存（按 stamp 升序维护）
  std::deque<ImgItem> buf_;
  size_t buf_max_{200};
  double max_dt_{0.3};

  std::string image_in_, cloud_in_, image_out_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;

  // diagnostics params
  int diag_every_n_{50};
  int offset_window_{80};
  double warn_gap_img_{0.08};
  double warn_gap_cloud_{0.20};
  bool store_stamp_sorted_{true};

  // ---------- image diagnostics state ----------
  rclcpp::Time last_img_stamp_{0, 0, RCL_ROS_TIME};
  int img_cnt_{0};
  int img_backjump_cnt_{0};
  double img_gap_min_{std::numeric_limits<double>::infinity()};
  double img_gap_max_{0.0};
  std::vector<double> img_gaps_; // 保存最近一段 gap 用于 median

  // ---------- cloud diagnostics state ----------
  rclcpp::Time last_cloud_stamp_{0, 0, RCL_ROS_TIME};
  int cloud_cnt_{0};
  int cloud_backjump_cnt_{0};
  double cloud_gap_min_{std::numeric_limits<double>::infinity()};
  double cloud_gap_max_{0.0};
  std::vector<double> cloud_gaps_;

  // ---------- cross-sensor diagnostics ----------
  std::deque<double> signed_dt_window_; // img_stamp - cloud_stamp（匹配成功的）
  std::deque<double> abs_dt_window_;

  static double median_of(std::vector<double> v) {
    if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::nth_element(v.begin(), v.begin() + v.size()/2, v.end());
    double m = v[v.size()/2];
    if (v.size() % 2 == 0) {
      auto it = std::max_element(v.begin(), v.begin() + v.size()/2);
      m = (m + *it) * 0.5;
    }
    return m;
  }

  static double median_of_deque(const std::deque<double>& d) {
    if (d.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::vector<double> v(d.begin(), d.end());
    return median_of(std::move(v));
  }

  void push_window(std::deque<double>& win, double x, int max_n) {
    win.push_back(x);
    while ((int)win.size() > max_n) win.pop_front();
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    const auto t_arr = this->now();
    const auto t_stp = rclcpp::Time(msg->header.stamp);

    // 1) 记录 arrival-stamp 延迟（用于判断传输/队列）
    const double delay_ms = (t_arr - t_stp).seconds() * 1000.0;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "IMG: stamp=%.6f arrival=%.6f delay=%.1f ms",
      t_stp.seconds(), t_arr.seconds(), delay_ms);

    // 2) stamp 连续性诊断（gap/回跳）
    if (last_img_stamp_.nanoseconds() != 0) {
      const double gap = (t_stp - last_img_stamp_).seconds();
      if (gap < 0) img_backjump_cnt_++;
      else {
        img_gap_min_ = std::min(img_gap_min_, gap);
        img_gap_max_ = std::max(img_gap_max_, gap);
        img_gaps_.push_back(gap);
      }
      if (gap > warn_gap_img_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "IMG stamp gap unusually large: %.4f s (possible drop/jitter).", gap);
      }
    }
    last_img_stamp_ = t_stp;
    img_cnt_++;

    // 3) 缓存（用 header stamp，不用 now）
    ImgItem item{t_stp, msg};
    if (store_stamp_sorted_) {
      auto it = std::upper_bound(
        buf_.begin(), buf_.end(), item.stamp,
        [](const rclcpp::Time& t, const ImgItem& x){ return t < x.stamp; });
      buf_.insert(it, std::move(item));
    } else {
      buf_.push_back(std::move(item));
    }
    while (buf_.size() > buf_max_) buf_.pop_front();

    // 4) 每隔 N 帧输出一次 image stamp 统计
    if (img_cnt_ % diag_every_n_ == 0) {
      double med = median_of(img_gaps_);
      double mean = (img_gaps_.empty()) ? std::numeric_limits<double>::quiet_NaN()
        : std::accumulate(img_gaps_.begin(), img_gaps_.end(), 0.0) / img_gaps_.size();

      RCLCPP_INFO(get_logger(),
        "IMG stamp stats (last %zu gaps): min=%.4f med=%.4f mean=%.4f max=%.4f backjump=%d",
        img_gaps_.size(),
        std::isfinite(img_gap_min_) ? img_gap_min_ : 0.0,
        std::isfinite(med) ? med : 0.0,
        std::isfinite(mean) ? mean : 0.0,
        img_gap_max_,
        img_backjump_cnt_);

      // reset window stats
      img_gaps_.clear();
      img_gap_min_ = std::numeric_limits<double>::infinity();
      img_gap_max_ = 0.0;
      img_backjump_cnt_ = 0;
    }
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    if (buf_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Image buffer empty.");
      return;
    }

    const auto c_arr = this->now();
    const auto c_stp = rclcpp::Time(cloud_msg->header.stamp);

    // 1) cloud arrival-stamp 延迟
    const double c_delay_ms = (c_arr - c_stp).seconds() * 1000.0;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "CLOUD: stamp=%.6f arrival=%.6f delay=%.1f ms frame=%s",
      c_stp.seconds(), c_arr.seconds(), c_delay_ms, cloud_msg->header.frame_id.c_str());

    // 2) cloud stamp 连续性
    if (last_cloud_stamp_.nanoseconds() != 0) {
      const double gap = (c_stp - last_cloud_stamp_).seconds();
      if (gap < 0) cloud_backjump_cnt_++;
      else {
        cloud_gap_min_ = std::min(cloud_gap_min_, gap);
        cloud_gap_max_ = std::max(cloud_gap_max_, gap);
        cloud_gaps_.push_back(gap);
      }
      if (gap > warn_gap_cloud_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "CLOUD stamp gap unusually large: %.4f s (possible drop/jitter).", gap);
      }
    }
    last_cloud_stamp_ = c_stp;
    cloud_cnt_++;

    // 每隔一段输出 cloud stamp 统计（用同样的 N）
    if (cloud_cnt_ % std::max(5, diag_every_n_/5) == 0) { // cloud更低频，别等太久
      double med = median_of(cloud_gaps_);
      double mean = (cloud_gaps_.empty()) ? std::numeric_limits<double>::quiet_NaN()
        : std::accumulate(cloud_gaps_.begin(), cloud_gaps_.end(), 0.0) / cloud_gaps_.size();

      RCLCPP_INFO(get_logger(),
        "CLOUD stamp stats (last %zu gaps): min=%.4f med=%.4f mean=%.4f max=%.4f backjump=%d",
        cloud_gaps_.size(),
        std::isfinite(cloud_gap_min_) ? cloud_gap_min_ : 0.0,
        std::isfinite(med) ? med : 0.0,
        std::isfinite(mean) ? mean : 0.0,
        cloud_gap_max_,
        cloud_backjump_cnt_);

      cloud_gaps_.clear();
      cloud_gap_min_ = std::numeric_limits<double>::infinity();
      cloud_gap_max_ = 0.0;
      cloud_backjump_cnt_ = 0;
    }

    // 3) 在图像缓存中找 stamp 最近的那张（img_stamp - cloud_stamp）
    const rclcpp::Time t_cloud = c_stp;

    size_t best = 0;
    double best_abs = std::numeric_limits<double>::infinity();
    double best_signed = 0.0;

    // 如果 buf_ 已经按 stamp 排序，可以二分加速；这里为了稳，先线扫
    for (size_t i = 0; i < buf_.size(); ++i) {
      const double dt_signed = (buf_[i].stamp - t_cloud).seconds();
      const double dt_abs = std::abs(dt_signed);
      if (dt_abs < best_abs) {
        best_abs = dt_abs;
        best_signed = dt_signed;
        best = i;
      }
    }

    // 维护窗口统计，用于判断“固定偏移/漂移”
    push_window(signed_dt_window_, best_signed, offset_window_);
    push_window(abs_dt_window_, best_abs, offset_window_);

    // 每秒左右输出一次相对偏移统计
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "SYNC window(%zu): abs_dt med=%.4f max=%.4f | signed_dt med=%.4f mean=%.4f",
      signed_dt_window_.size(),
      median_of_deque(abs_dt_window_),
      abs_dt_window_.empty() ? 0.0 : *std::max_element(abs_dt_window_.begin(), abs_dt_window_.end()),
      median_of_deque(signed_dt_window_),
      signed_dt_window_.empty() ? 0.0 :
        (std::accumulate(signed_dt_window_.begin(), signed_dt_window_.end(), 0.0) / signed_dt_window_.size())
    );

    if (best_abs > max_dt_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "No image within max_dt. best_abs=%.4f, best_signed=%.4f (img_stamp - cloud_stamp)",
        best_abs, best_signed);
      return;
    }

    // 4) 通过则发布：输出 stamp 强行设为 cloud stamp（给下游对齐）
    sensor_msgs::msg::Image out = *(buf_[best].msg);
    out.header.stamp = cloud_msg->header.stamp;
    pub_img_->publish(out);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PictureFilterNode>());
  rclcpp::shutdown();
  return 0;
}
