#pragma once
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <QLabel>
#include <QLineEdit>
#include <QString>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <vector>

// Qt5 requires this namespace macro
QT_CHARTS_USE_NAMESPACE

namespace tercom_rviz_plugins {

class ErrorHistoryPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit ErrorHistoryPanel(QWidget* parent = nullptr);
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config& config) override;

Q_SIGNALS:
  void chartDataReceived(const std::vector<float>& data);
  void statsDataReceived(const std::vector<float>& data);

private Q_SLOTS:
  void onChartDataReceived(const std::vector<float>& data);
  void onStatsDataReceived(const std::vector<float>& data);
  void onTopicEdited();

private:
  static QString valueColor(float value);
  static void applyValueColor(QLabel* lbl, float value, const QString& suffix = " m");
  void resubscribe();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_chart_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_stats_;

  // Topic configuration
  // Chart topic published by diagnostics_node at ~/error_history_chart
  //   → /tercom/diagnostics_node/error_history_chart
  // Stats topic published by diagnostics_node at ~/error_stats
  //   → /tercom/diagnostics_node/error_stats
  QString     topic_chart_ {"/tercom/diagnostics_node/error_history_chart"};
  QString     topic_stats_ {"/tercom/diagnostics_node/error_stats"};
  QLineEdit*  topic_chart_edit_{nullptr};
  QLineEdit*  topic_stats_edit_{nullptr};

  // Horizontal error chart (current / RMS / max rolling)
  QChartView*  h_chart_view_{nullptr};
  QLineSeries* h_series_current_{nullptr};
  QLineSeries* h_series_rms_{nullptr};
  QLineSeries* h_series_max_{nullptr};
  QLineSeries* h_threshold_{nullptr};
  QValueAxis*  h_axis_x_{nullptr};
  QValueAxis*  h_axis_y_{nullptr};

  // Vertical error chart
  QChartView*  v_chart_view_{nullptr};
  QLineSeries* v_series_{nullptr};
  QLineSeries* v_threshold_{nullptr};
  QValueAxis*  v_axis_x_{nullptr};
  QValueAxis*  v_axis_y_{nullptr};

  // Summary labels
  QLabel* now_lbl_{nullptr};
  QLabel* rms_lbl_{nullptr};
  QLabel* max_lbl_{nullptr};
  QLabel* mean_lbl_{nullptr};
};

}  // namespace tercom_rviz_plugins
