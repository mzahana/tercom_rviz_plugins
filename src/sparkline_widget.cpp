#include "tercom_rviz_plugins/sparkline_widget.hpp"
#include <QPainter>
#include <QPen>
#include <QFont>
#include <algorithm>
#include <cmath>

namespace tercom_rviz_plugins {

SparklineWidget::SparklineWidget(QWidget* p)
    : QWidget(p) {
  setMinimumSize(200, 60);
  setAttribute(Qt::WA_OpaquePaintEvent);
}

void SparklineWidget::setData(const std::vector<float>& samples, float threshold) {
  data_ = samples;
  threshold_ = threshold;
  update();
}

void SparklineWidget::setLabel(const QString& label) {
  label_ = label;
  update();
}

void SparklineWidget::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.fillRect(rect(), QColor(30, 30, 30));

  if (data_.empty()) return;

  const int W = width();
  const int H = height();
  const int pad = 4;
  const int inner_w = W - 2 * pad;
  const int inner_h = H - 2 * pad;

  float y_min = *std::min_element(data_.begin(), data_.end());
  float y_max = *std::max_element(data_.begin(), data_.end());
  if (threshold_ > 0.f) y_max = std::max(y_max, threshold_ * 1.1f);
  float y_range = y_max - y_min;
  if (y_range < 1e-6f) y_range = 1.f;

  auto to_screen = [&](int i, float v) -> QPointF {
    float x = pad + (static_cast<float>(i) / static_cast<float>(data_.size() - 1)) * inner_w;
    float y = pad + inner_h - ((v - y_min) / y_range) * inner_h;
    return {x, y};
  };

  bool over_threshold = false;
  if (threshold_ > 0.f) {
    for (float v : data_) {
      if (v > threshold_) { over_threshold = true; break; }
    }
  }
  QColor line_color = over_threshold ? QColor(220, 60, 60) : QColor(80, 160, 255);

  // Draw threshold line
  if (threshold_ > 0.f) {
    float ty = pad + inner_h - ((threshold_ - y_min) / y_range) * inner_h;
    p.setPen(QPen(QColor(200, 200, 80, 180), 1, Qt::DashLine));
    p.drawLine(QPointF(pad, ty), QPointF(W - pad, ty));
  }

  // Draw data polyline
  p.setPen(QPen(line_color, 1.5));
  QPolygonF poly;
  for (int i = 0; i < static_cast<int>(data_.size()); ++i) {
    poly << to_screen(i, data_[i]);
  }
  p.drawPolyline(poly);

  // Label (top-left)
  if (!label_.isEmpty()) {
    p.setPen(QColor(180, 180, 180));
    p.setFont(QFont("monospace", 7));
    p.drawText(pad + 2, pad + 10, label_);
  }

  // Current value (top-right)
  p.setPen(line_color);
  p.setFont(QFont("monospace", 7, QFont::Bold));
  QString val = QString::number(static_cast<double>(data_.back()), 'f', 2);
  p.drawText(rect().adjusted(0, 2, -pad - 2, 0), Qt::AlignRight | Qt::AlignTop, val);
}

}  // namespace tercom_rviz_plugins
