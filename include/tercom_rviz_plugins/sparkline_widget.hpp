#pragma once
#include <QWidget>
#include <QString>
#include <vector>

namespace tercom_rviz_plugins {

/// Compact line chart rendered with QPainter — no QtCharts dependency.
/// Draws the last N float samples with an optional horizontal threshold line.
/// Color: blue when all samples <= threshold; red when any sample > threshold.
class SparklineWidget : public QWidget {
  Q_OBJECT
public:
  explicit SparklineWidget(QWidget* parent = nullptr);

  /// Replace data. Call from Qt main thread only.
  void setData(const std::vector<float>& samples, float threshold = -1.f);

  /// Optional label shown in top-left corner (e.g. "NIS").
  void setLabel(const QString& label);

  QSize sizeHint() const override { return {220, 70}; }

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  std::vector<float> data_;
  float threshold_{-1.f};
  QString label_;
};

}  // namespace tercom_rviz_plugins
