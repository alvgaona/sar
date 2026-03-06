#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <rerun.hpp>

class RerunBridge : public rclcpp::Node
{
public:
  RerunBridge()
  : Node("rerun_bridge")
  {
    declare_parameter("image_topic", "/oak/rgb/color");
    declare_parameter("scan_topic", "/scan_filtered");
    declare_parameter("map_topic", "/map");

    auto image_topic = get_parameter("image_topic").as_string();
    auto scan_topic = get_parameter("scan_topic").as_string();
    auto map_topic = get_parameter("map_topic").as_string();

    declare_parameter("rerun_url", "rerun+http://127.0.0.1:9876/proxy");
    auto rerun_url = get_parameter("rerun_url").as_string();

    rec_ = std::make_unique<rerun::RecordingStream>("sar");
    rec_->connect_grpc(rerun_url).exit_on_failure();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { on_image(msg); });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 10,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { on_scan(msg); });

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, rclcpp::QoS(1).transient_local().reliable(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { on_map(msg); });

    pose_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { on_pose_timer(); });
  }

private:
  void on_image(const sensor_msgs::msg::Image::SharedPtr & msg)
  {
    auto cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
    auto & img = cv_ptr->image;

    auto bytes = rerun::Collection<uint8_t>::borrow(
      img.data, img.total() * img.elemSize());
    rec_->log("camera/rgb",
      rerun::Image::from_rgb24(bytes,
        {static_cast<uint32_t>(img.cols), static_cast<uint32_t>(img.rows)}));
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr & msg)
  {
    std::vector<rerun::Position3D> points;
    float angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float r = msg->ranges[i];
      if (r >= msg->range_min && r <= msg->range_max) {
        points.push_back({r * std::cos(angle), r * std::sin(angle), 0.0f});
      }
      angle += msg->angle_increment;
    }

    rec_->log("scan", rerun::Points3D(points).with_radii({0.02f}));
  }

  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr & msg)
  {
    auto w = msg->info.width;
    auto h = msg->info.height;
    std::vector<uint8_t> pixels(w * h);

    for (size_t i = 0; i < msg->data.size(); ++i) {
      auto val = msg->data[i];
      if (val < 0) {
        pixels[i] = 128;
      } else {
        pixels[i] = static_cast<uint8_t>(255 - val * 255 / 100);
      }
    }

    rec_->log("map",
      rerun::Image::from_grayscale8(std::move(pixels),
        {static_cast<uint32_t>(w), static_cast<uint32_t>(h)}));
  }

  void on_pose_timer()
  {
    try {
      auto t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      auto & pos = t.transform.translation;
      rec_->log("robot/pose",
        rerun::Points3D({{
          static_cast<float>(pos.x),
          static_cast<float>(pos.y),
          static_cast<float>(pos.z)
        }}).with_radii({0.1f}));
    } catch (const tf2::TransformException &) {
    }
  }

  std::unique_ptr<rerun::RecordingStream> rec_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RerunBridge>());
  rclcpp::shutdown();
  return 0;
}
