#pragma once
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <vector>

#include "tercom_rviz_plugins/sparkline_widget.hpp"

namespace tercom_rviz_plugins {

class FilterStatusPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit FilterStatusPanel(QWidget* parent = nullptr);
  void onInitialize() override;

Q_SIGNALS:
  void stateReceived(const QString& state);
  void healthReceived(const std::vector<float>& data);
  void nisHistoryReceived(const std::vector<float>& data);
  void biasAccelReceived(float x, float y, float z);
  void biasGyroReceived(float x, float y, float z);
  void positionReceived(double lat, double lon, float alt, int status);

private Q_SLOTS:
  void onStateReceived(const QString& state);
  void onHealthReceived(const std::vector<float>& data);
  void onNisHistoryReceived(const std::vector<float>& data);
  void onBiasAccelReceived(float x, float y, float z);
  void onBiasGyroReceived(float x, float y, float z);
  void onPositionReceived(double lat, double lon, float alt, int status);
  void onResetClicked();

private:
  static QString badgeStyleSheet(const QString& bg, const QString& fg);
  static QString progressBarStyleSheet(float value, float limit);
  static QString biasLabelColor(float magnitude);
  static void applyBiasColor(QLabel* lbl, float val);
  void updateHealthLed();

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_health_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_nis_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_bias_accel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_bias_gyro_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_global_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;

  // Cached state for combined health assessment
  QString current_state_   {"UNKNOWN"};
  float   current_sigma_   {0.f};
  float   current_avg_nis_ {0.f};
  float   current_innov_   {0.f};
  float   current_soft_reset_count_ {0.f};
  float   current_hard_reset_count_ {0.f};

  QLabel*          health_led_{nullptr};
  QLabel*          state_badge_{nullptr};
  SparklineWidget* nis_sparkline_{nullptr};
  QProgressBar*    sigma_bar_{nullptr};
  QLabel*          sigma_lbl_{nullptr};
  QProgressBar*    innov_bar_{nullptr};
  QLabel*          innov_lbl_{nullptr};
  QLabel*          soft_resets_lbl_{nullptr};
  QLabel*          hard_resets_lbl_{nullptr};
  QLabel*          ax_{nullptr};
  QLabel*          ay_{nullptr};
  QLabel*          az_{nullptr};
  QLabel*          gx_{nullptr};
  QLabel*          gy_{nullptr};
  QLabel*          gz_{nullptr};

  // Position display
  QLabel*          lat_lbl_{nullptr};
  QLabel*          lon_lbl_{nullptr};
  QLabel*          alt_lbl_{nullptr};

  // Reset filter button
  QPushButton*     reset_btn_{nullptr};
};

}  // namespace tercom_rviz_plugins
