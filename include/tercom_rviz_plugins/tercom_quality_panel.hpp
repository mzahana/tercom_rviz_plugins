#pragma once
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QString>
#include <vector>

namespace tercom_rviz_plugins {

class TercomQualityPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit TercomQualityPanel(QWidget* parent = nullptr);
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config& config) override;

Q_SIGNALS:
  void statusReceived(const QString& status);
  void qualityReceived(const std::vector<float>& data);
  void rejectionReceived(const QString& reason);

private Q_SLOTS:
  void onStatusReceived(const QString& status);
  void onQualityReceived(const std::vector<float>& data);
  void onRejectionReceived(const QString& reason);
  void onTopicEdited();

private:
  static QString progressBarStyle(const QString& color);
  void resubscribe();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_quality_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_rejection_;

  // Topic configuration
  QString     topic_status_    {"/tercom/tercom_node/status"};
  QString     topic_quality_   {"/tercom/tercom_node/match_quality"};
  QString     topic_rejection_ {"/tercom/tercom_node/rejection_reason"};
  QLineEdit*  topic_status_edit_{nullptr};
  QLineEdit*  topic_quality_edit_{nullptr};
  QLineEdit*  topic_rejection_edit_{nullptr};

  // Widgets
  QLabel*        status_lbl_{nullptr};
  QProgressBar*  collect_bar_{nullptr};
  QProgressBar*  mad_bar_{nullptr};
  QLabel*        mad_lbl_{nullptr};
  QProgressBar*  disc_bar_{nullptr};
  QLabel*        disc_lbl_{nullptr};
  QProgressBar*  rough_bar_{nullptr};
  QLabel*        rough_lbl_{nullptr};
  QProgressBar*  noise_bar_{nullptr};
  QLabel*        noise_lbl_{nullptr};
  QLabel*        decision_badge_{nullptr};
  QLabel*        session_lbl_{nullptr};
  QLabel*        reasons_lbl_{nullptr};

  int accepted_count_{0};
  int rejected_count_{0};
};

}  // namespace tercom_rviz_plugins
