#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <deque>
#include <limits>
#include <cmath>
#include <string>

class PictureFilterNode : public rclcpp::Node {
public:
  PictureFilterNode() : Node("picture_filter_node") {
    declare_parameter<std::string>("image_in", "/image_raw");
    declare_parameter<std::string>("cloud_in", "/cloud_registered");
    declare_parameter<std::string>("image_out", "/image_nearest");

    // 允许的最大时间差（超过就不发，避免错配）
    declare_parameter<double>("max_dt", 0.05);

    // 图像缓存长度
    declare_parameter<int>("buffer_size", 200); // 100Hz * 2s

    image_in_  = get_parameter("image_in").as_string();
    cloud_in_  = get_parameter("cloud_in").as_string();
    image_out_ = get_parameter("image_out").as_string();
    max_dt_    = get_parameter("max_dt").as_double();
    buf_max_   = (size_t)get_parameter("buffer_size").as_int();

    sub_img_ = create_subscription<sensor_msgs::msg::Image>(
      image_in_, rclcpp::SensorDataQoS(),
      std::bind(&PictureFilterNode::onImage, this, std::placeholders::_1));

    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_in_, rclcpp::SensorDataQoS(),
      std::bind(&PictureFilterNode::onCloud, this, std::placeholders::_1));

    pub_img_ = create_publisher<sensor_msgs::msg::Image>(image_out_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "image_in=%s cloud_in=%s image_out=%s max_dt=%.3f buf=%zu",
                image_in_.c_str(), cloud_in_.c_str(), image_out_.c_str(), max_dt_, buf_max_);
  }

private:
  struct ImgItem {
    rclcpp::Time stamp;
    sensor_msgs::msg::Image::SharedPtr msg;
  };

  std::deque<ImgItem> buf_;
  size_t buf_max_{200};
  double max_dt_{0.05};

  std::string image_in_, cloud_in_, image_out_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_;

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    buf_.push_back({rclcpp::Time(msg->header.stamp), msg});
    while (buf_.size() > buf_max_) buf_.pop_front();
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    if (buf_.empty()) return;

    const rclcpp::Time t_cloud(cloud->header.stamp);

    size_t best = 0;
    double best_dt = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < buf_.size(); ++i) {
      double dt = std::abs((buf_[i].stamp - t_cloud).seconds());
      if (dt < best_dt) { best_dt = dt; best = i; }
    }

    if (best_dt > max_dt_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "No image within max_dt. best_dt=%.4f", best_dt);
      return;
    }

    // 发布“最近图”
    auto out = *(buf_[best].msg);               // 拷贝一份消息头和数据
    out.header.stamp = cloud->header.stamp;     // 建议改成 cloud stamp，方便下游严格对齐
    pub_img_->publish(out);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PictureFilterNode>());
  rclcpp::shutdown();
  return 0;
}
