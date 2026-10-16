#include <array>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuCovarianceRelay : public rclcpp::Node
{
public:
  ImuCovarianceRelay()
  : Node("imu_covariance_relay")
  {
    const auto orientation_variance = declare_parameter<double>("orientation_variance", 1e-2);
    const auto angular_velocity_variance =
      declare_parameter<double>("angular_velocity_variance", 1e-4);
    const auto linear_acceleration_variance =
      declare_parameter<double>("linear_acceleration_variance", 1e-2);

    orientation_covariance_ = makeDiagonalCovariance(orientation_variance);
    angular_velocity_covariance_ = makeDiagonalCovariance(angular_velocity_variance);
    linear_acceleration_covariance_ = makeDiagonalCovariance(linear_acceleration_variance);

    auto sub_qos = rclcpp::SensorDataQoS();
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(2000));
    pub_qos.reliable();
    pub_qos.durability_volatile();

    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data_raw_raw", sub_qos,
      std::bind(&ImuCovarianceRelay::handleMessage, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", pub_qos);
  }

private:
  static std::array<double, 9> makeDiagonalCovariance(double variance)
  {
    return {variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, variance};
  }

  void handleMessage(const sensor_msgs::msg::Imu::SharedPtr msg) const
  {
    auto out = *msg;
    out.orientation_covariance = orientation_covariance_;
    out.angular_velocity_covariance = angular_velocity_covariance_;
    out.linear_acceleration_covariance = linear_acceleration_covariance_;
    pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  std::array<double, 9> orientation_covariance_{};
  std::array<double, 9> angular_velocity_covariance_{};
  std::array<double, 9> linear_acceleration_covariance_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuCovarianceRelay>());
  rclcpp::shutdown();
  return 0;
}
