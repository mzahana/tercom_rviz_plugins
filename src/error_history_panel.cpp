#include "tercom_rviz_plugins/error_history_panel.hpp"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QFrame>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <algorithm>

QT_CHARTS_USE_NAMESPACE

namespace tercom_rviz_plugins {

// ── colour helpers ────────────────────────────────────────────────────────────

QString ErrorHistoryPanel::valueColor(float value)
{
  if (value < 5.f)  return "#2ecc71";   // green
  if (value < 15.f) return "#e8a020";   // amber
  return "#e74c3c";                      // red
}

void ErrorHistoryPanel::applyValueColor(QLabel* lbl, float value, const QString& suffix)
{
  lbl->setText(QString("%1%2").arg(static_cast<double>(value), 0, 'f', 2).arg(suffix));
  lbl->setStyleSheet(
    QString("font-weight:bold; color:%1; font-size:12px;").arg(valueColor(value)));
}

// ── chart factory ─────────────────────────────────────────────────────────────

static QChart* make_chart(const QString& title)
{
  auto* chart = new QChart();
  chart->setTitle(title);
  chart->setBackgroundBrush(QColor(30, 30, 30));
  chart->setTitleBrush(QColor(180, 180, 180));
  chart->legend()->setLabelColor(QColor(180, 180, 180));
  chart->legend()->setBackgroundVisible(false);
  chart->setMargins(QMargins(4, 4, 4, 4));
  return chart;
}

// ── constructor ───────────────────────────────────────────────────────────────

ErrorHistoryPanel::ErrorHistoryPanel(QWidget* parent)
  : rviz_common::Panel(parent)
{
  setStyleSheet("background-color: #1e1e1e; color: #cccccc;");

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(6, 6, 6, 6);
  root->setSpacing(6);

  // ── Topic configuration ──────────────────────────────────────────────────
  auto* topics_box = new QGroupBox("Topics", this);
  topics_box->setStyleSheet(
    "QGroupBox { color: #aaa; border: 1px solid #444; margin-top: 6px; font-size: 10px; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 8px; }");
  auto* topics_grid = new QGridLayout(topics_box);
  topics_grid->setSpacing(3);
  topics_grid->setContentsMargins(4, 8, 4, 4);

  auto topic_style = QString(
    "background:#252525; color:#ccc; border:1px solid #555;"
    " border-radius:2px; font-size:10px; padding:1px 3px;");

  topics_grid->addWidget(new QLabel("Chart:", this), 0, 0);
  topic_chart_edit_ = new QLineEdit(topic_chart_, this);
  topic_chart_edit_->setStyleSheet(topic_style);
  topics_grid->addWidget(topic_chart_edit_, 0, 1);

  topics_grid->addWidget(new QLabel("Stats:", this), 1, 0);
  topic_stats_edit_ = new QLineEdit(topic_stats_, this);
  topic_stats_edit_->setStyleSheet(topic_style);
  topics_grid->addWidget(topic_stats_edit_, 1, 1);

  root->addWidget(topics_box);

  connect(topic_chart_edit_, &QLineEdit::editingFinished,
          this, &ErrorHistoryPanel::onTopicEdited);
  connect(topic_stats_edit_, &QLineEdit::editingFinished,
          this, &ErrorHistoryPanel::onTopicEdited);

  // ── Horizontal error chart ─────────────────────────────────────────────
  {
    QChart* chart = make_chart("Horizontal Error");

    h_series_current_ = new QLineSeries();
    h_series_current_->setName("now");
    h_series_current_->setColor(QColor(80, 160, 255));

    h_series_rms_ = new QLineSeries();
    h_series_rms_->setName("RMS");
    h_series_rms_->setColor(QColor(80, 200, 120));

    h_series_max_ = new QLineSeries();
    h_series_max_->setName("max");
    h_series_max_->setColor(QColor(220, 80, 60));

    h_threshold_ = new QLineSeries();
    h_threshold_->setName("thresh");
    QPen tpen(QColor(220, 200, 60, 200));
    tpen.setStyle(Qt::DashLine);
    tpen.setWidth(1);
    h_threshold_->setPen(tpen);

    chart->addSeries(h_series_current_);
    chart->addSeries(h_series_rms_);
    chart->addSeries(h_series_max_);
    chart->addSeries(h_threshold_);

    h_axis_x_ = new QValueAxis();
    h_axis_x_->setTitleText("sample");
    h_axis_x_->setLabelFormat("%d");
    h_axis_x_->setLabelsColor(QColor(160, 160, 160));
    h_axis_x_->setTitleBrush(QColor(160, 160, 160));
    h_axis_x_->setGridLineColor(QColor(60, 60, 60));
    chart->addAxis(h_axis_x_, Qt::AlignBottom);

    h_axis_y_ = new QValueAxis();
    h_axis_y_->setTitleText("m");
    h_axis_y_->setLabelsColor(QColor(160, 160, 160));
    h_axis_y_->setTitleBrush(QColor(160, 160, 160));
    h_axis_y_->setGridLineColor(QColor(60, 60, 60));
    chart->addAxis(h_axis_y_, Qt::AlignLeft);

    for (auto* s : {h_series_current_, h_series_rms_, h_series_max_, h_threshold_}) {
      s->attachAxis(h_axis_x_);
      s->attachAxis(h_axis_y_);
    }

    h_chart_view_ = new QChartView(chart, this);
    h_chart_view_->setRenderHint(QPainter::Antialiasing);
    h_chart_view_->setFixedHeight(170);
    h_chart_view_->setStyleSheet("background: #1e1e1e;");
    root->addWidget(h_chart_view_);
  }

  // ── Vertical error chart ───────────────────────────────────────────────
  {
    QChart* chart = make_chart("Vertical Error");

    v_series_ = new QLineSeries();
    v_series_->setName("vertical");
    v_series_->setColor(QColor(160, 100, 255));

    v_threshold_ = new QLineSeries();
    v_threshold_->setName("thresh");
    QPen tpen(QColor(220, 200, 60, 200));
    tpen.setStyle(Qt::DashLine);
    tpen.setWidth(1);
    v_threshold_->setPen(tpen);

    chart->addSeries(v_series_);
    chart->addSeries(v_threshold_);

    v_axis_x_ = new QValueAxis();
    v_axis_x_->setTitleText("sample");
    v_axis_x_->setLabelFormat("%d");
    v_axis_x_->setLabelsColor(QColor(160, 160, 160));
    v_axis_x_->setGridLineColor(QColor(60, 60, 60));
    chart->addAxis(v_axis_x_, Qt::AlignBottom);

    v_axis_y_ = new QValueAxis();
    v_axis_y_->setTitleText("m");
    v_axis_y_->setLabelsColor(QColor(160, 160, 160));
    v_axis_y_->setGridLineColor(QColor(60, 60, 60));
    chart->addAxis(v_axis_y_, Qt::AlignLeft);

    v_series_->attachAxis(v_axis_x_);
    v_series_->attachAxis(v_axis_y_);
    v_threshold_->attachAxis(v_axis_x_);
    v_threshold_->attachAxis(v_axis_y_);

    v_chart_view_ = new QChartView(chart, this);
    v_chart_view_->setRenderHint(QPainter::Antialiasing);
    v_chart_view_->setFixedHeight(140);
    v_chart_view_->setStyleSheet("background: #1e1e1e;");
    root->addWidget(v_chart_view_);
  }

  // ── Summary labels ─────────────────────────────────────────────────────
  {
    auto* stats_box = new QGroupBox("Statistics", this);
    stats_box->setStyleSheet(
      "QGroupBox { color: #aaa; border: 1px solid #444; margin-top: 6px; }"
      "QGroupBox::title { subcontrol-origin: margin; left: 8px; }");
    auto* grid = new QGridLayout(stats_box);
    grid->setSpacing(4);

    auto make_stat = [&](int row, const QString& key) -> QLabel* {
      grid->addWidget(new QLabel(key, this), row, 0);
      auto* v = new QLabel("—", this);
      v->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
      grid->addWidget(v, row, 1);
      return v;
    };

    now_lbl_  = make_stat(0, "Now:");
    rms_lbl_  = make_stat(1, "RMS:");
    max_lbl_  = make_stat(2, "Max:");
    mean_lbl_ = make_stat(3, "Mean:");

    root->addWidget(stats_box);
  }

  root->addStretch();

  // ── Cross-thread signals ───────────────────────────────────────────────
  connect(this, &ErrorHistoryPanel::chartDataReceived,
          this, &ErrorHistoryPanel::onChartDataReceived,
          Qt::QueuedConnection);
  connect(this, &ErrorHistoryPanel::statsDataReceived,
          this, &ErrorHistoryPanel::onStatsDataReceived,
          Qt::QueuedConnection);
}

// ── onInitialize ──────────────────────────────────────────────────────────────

void ErrorHistoryPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  resubscribe();
}

// ── save / load ───────────────────────────────────────────────────────────────

void ErrorHistoryPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("TopicChart", topic_chart_);
  config.mapSetValue("TopicStats", topic_stats_);
}

void ErrorHistoryPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
  QString val;
  if (config.mapGetString("TopicChart", &val)) {
    topic_chart_ = val;
    topic_chart_edit_->setText(val);
  }
  if (config.mapGetString("TopicStats", &val)) {
    topic_stats_ = val;
    topic_stats_edit_->setText(val);
  }
}

// ── topic management ──────────────────────────────────────────────────────────

void ErrorHistoryPanel::resubscribe()
{
  if (!node_) return;

  sub_chart_.reset();
  sub_stats_.reset();

  // Chart topic: diagnostics_node publishes interleaved 4-tuples
  //   flat = [h_err, rms, max, v_err, h_err, rms, max, v_err, ...]
  //   one record per accumulated error sample
  sub_chart_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    topic_chart_.toStdString(), rclcpp::QoS(10),
    [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
      Q_EMIT chartDataReceived(std::vector<float>(msg->data.begin(), msg->data.end()));
    });

  // Stats topic: diagnostics_node publishes [rms_h, max_h, mean_h, v_err, now_h, v_err2]
  sub_stats_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    topic_stats_.toStdString(), rclcpp::QoS(10),
    [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
      Q_EMIT statsDataReceived(std::vector<float>(msg->data.begin(), msg->data.end()));
    });
}

void ErrorHistoryPanel::onTopicEdited()
{
  topic_chart_ = topic_chart_edit_->text().trimmed();
  topic_stats_ = topic_stats_edit_->text().trimmed();
  resubscribe();
}

// ── slots ─────────────────────────────────────────────────────────────────────

void ErrorHistoryPanel::onChartDataReceived(const std::vector<float>& data)
{
  // diagnostics_node packs error history as interleaved 4-tuples:
  //   data = [h_err_0, rms_0, max_0, v_err_0,  h_err_1, rms_1, max_1, v_err_1, ...]
  // N = data.size() / 4  (number of accumulated samples)
  if (data.size() < 4) return;
  const int N = static_cast<int>(data.size()) / 4;

  constexpr float THRESH_H = 50.f;   // horizontal warning threshold (m)
  constexpr float THRESH_V = 20.f;   // vertical warning threshold (m)

  float max_h = 0.f, max_v = 0.f;
  QVector<QPointF> pts_h, pts_rms, pts_max, pts_v;
  pts_h.reserve(N); pts_rms.reserve(N); pts_max.reserve(N); pts_v.reserve(N);

  for (int i = 0; i < N; ++i) {
    const float h   = data[static_cast<size_t>(4 * i + 0)];
    const float rms = data[static_cast<size_t>(4 * i + 1)];
    const float mx  = data[static_cast<size_t>(4 * i + 2)];
    const float v   = data[static_cast<size_t>(4 * i + 3)];
    max_h = std::max(max_h, h);
    max_v = std::max(max_v, std::abs(v));
    pts_h   << QPointF(i, static_cast<double>(h));
    pts_rms << QPointF(i, static_cast<double>(rms));
    pts_max << QPointF(i, static_cast<double>(mx));
    pts_v   << QPointF(i, static_cast<double>(v));
  }

  h_series_current_->replace(pts_h);
  h_series_rms_->replace(pts_rms);
  h_series_max_->replace(pts_max);
  v_series_->replace(pts_v);

  // Threshold lines
  if (N >= 2) {
    QVector<QPointF> th, tv;
    th << QPointF(0, THRESH_H) << QPointF(N - 1, THRESH_H);
    tv << QPointF(0, THRESH_V) << QPointF(N - 1, THRESH_V);
    h_threshold_->replace(th);
    v_threshold_->replace(tv);
  }

  double h_ymax = static_cast<double>(std::max(max_h, THRESH_H) * 1.15f);
  h_axis_x_->setRange(0, N - 1);
  h_axis_y_->setRange(0, h_ymax);

  double v_extent = static_cast<double>(std::max(max_v, THRESH_V) * 1.15f);
  v_axis_x_->setRange(0, N - 1);
  v_axis_y_->setRange(-v_extent, v_extent);
}

void ErrorHistoryPanel::onStatsDataReceived(const std::vector<float>& data)
{
  // diagnostics_node publishes: [rms_h, max_h, mean_h, v_err, now_h, v_err2]
  // Panel layout: Now / RMS / Max / Mean
  if (data.size() < 1) return;
  const float rms_h  = data.size() > 0 ? data[0] : 0.f;
  const float max_h  = data.size() > 1 ? data[1] : 0.f;
  const float mean_h = data.size() > 2 ? data[2] : 0.f;
  const float now_h  = data.size() > 4 ? data[4] : rms_h;   // data[4] = h_errors[-1]

  applyValueColor(now_lbl_,  now_h);
  applyValueColor(rms_lbl_,  rms_h);
  applyValueColor(max_lbl_,  max_h);
  applyValueColor(mean_lbl_, mean_h);
}

}  // namespace tercom_rviz_plugins

PLUGINLIB_EXPORT_CLASS(tercom_rviz_plugins::ErrorHistoryPanel, rviz_common::Panel)
