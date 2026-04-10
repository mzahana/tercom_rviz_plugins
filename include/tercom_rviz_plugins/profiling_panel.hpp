#pragma once

#include <QLabel>
#include <QTableWidget>
#include <QTimer>
#include <QString>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace tercom_rviz_plugins {

/**
 * ProfilingPanel — RViz2 panel that displays execution time and call frequency
 * for all timed components across tercom_node, eskf_node, and diagnostics_node.
 *
 * Subscribes to /tercom/diagnostics_node/profiling (Float32MultiArray, 16 floats):
 *   [exec_ms, hz] × 8 components:
 *     [0-1]  tercom  : cb_synced
 *     [2-3]  tercom  : run_matching
 *     [4-5]  eskf    : cb_imu
 *     [6-7]  eskf    : cb_tercom_fix
 *     [8-9]  eskf    : cb_altitude
 *     [10-11] diag   : timer_error
 *     [12-13] diag   : timer_paths
 *     [14-15] diag   : timer_stats
 */
class ProfilingPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ProfilingPanel(QWidget* parent = nullptr);
  ~ProfilingPanel() override = default;

  void onInitialize() override;

private Q_SLOTS:
  void onWatchdog();

private:
  // Color thresholds per row (exec_ms): [green_limit, yellow_limit]
  // Above yellow_limit → red. Values tuned to expected computation per component.
  struct RowThresholds { float green_ms; float yellow_ms; };
  static constexpr RowThresholds THRESHOLDS[8] = {
    {5.f,   20.f},   // tercom: cb_synced
    {100.f, 500.f},  // tercom: run_matching
    {1.f,   5.f},    // eskf: cb_imu
    {2.f,   10.f},   // eskf: cb_tercom_fix
    {1.f,   5.f},    // eskf: cb_altitude
    {10.f,  50.f},   // diag: timer_error
    {20.f,  100.f},  // diag: timer_paths
    {5.f,   20.f},   // diag: timer_stats
  };

  void updateTable(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  static QString execColor(float exec_ms, float green_ms, float yellow_ms);

  // UI
  QLabel*       status_label_{nullptr};
  QTableWidget* table_{nullptr};
  QTimer*       watchdog_timer_{nullptr};

  // ROS
  rclcpp::Node::SharedPtr             node_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;

  bool data_received_{false};
};

}  // namespace tercom_rviz_plugins
