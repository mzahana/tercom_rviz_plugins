#include "tercom_rviz_plugins/tercom_quality_panel.hpp"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QFrame>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <algorithm>

namespace tercom_rviz_plugins {

// ── helpers ──────────────────────────────────────────────────────────────────

QString TercomQualityPanel::progressBarStyle(const QString& color)
{
  return QString(
    "QProgressBar {"
    "  border: 1px solid #555;"
    "  border-radius: 3px;"
    "  background: #2a2a2a;"
    "  text-align: center;"
    "  color: #ccc;"
    "}"
    "QProgressBar::chunk {"
    "  background-color: %1;"
    "  border-radius: 2px;"
    "}").arg(color);
}

// ── constructor ───────────────────────────────────────────────────────────────

TercomQualityPanel::TercomQualityPanel(QWidget* parent)
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

  topics_grid->addWidget(new QLabel("Status:", this),   0, 0);
  topic_status_edit_ = new QLineEdit(topic_status_, this);
  topic_status_edit_->setStyleSheet(topic_style);
  topics_grid->addWidget(topic_status_edit_, 0, 1);

  topics_grid->addWidget(new QLabel("Quality:", this),  1, 0);
  topic_quality_edit_ = new QLineEdit(topic_quality_, this);
  topic_quality_edit_->setStyleSheet(topic_style);
  topics_grid->addWidget(topic_quality_edit_, 1, 1);

  topics_grid->addWidget(new QLabel("Rejected:", this), 2, 0);
  topic_rejection_edit_ = new QLineEdit(topic_rejection_, this);
  topic_rejection_edit_->setStyleSheet(topic_style);
  topics_grid->addWidget(topic_rejection_edit_, 2, 1);

  root->addWidget(topics_box);

  connect(topic_status_edit_,    &QLineEdit::editingFinished,
          this, &TercomQualityPanel::onTopicEdited);
  connect(topic_quality_edit_,   &QLineEdit::editingFinished,
          this, &TercomQualityPanel::onTopicEdited);
  connect(topic_rejection_edit_, &QLineEdit::editingFinished,
          this, &TercomQualityPanel::onTopicEdited);

  // ── Pipeline status ──────────────────────────────────────────────────────
  auto* pipeline_box = new QGroupBox("Pipeline", this);
  pipeline_box->setStyleSheet(
    "QGroupBox { color: #aaa; border: 1px solid #444; margin-top: 6px; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 8px; }");
  auto* pipeline_lay = new QVBoxLayout(pipeline_box);

  status_lbl_ = new QLabel("—", this);
  status_lbl_->setAlignment(Qt::AlignCenter);
  status_lbl_->setStyleSheet("font-weight: bold; font-size: 13px; padding: 4px;");
  pipeline_lay->addWidget(status_lbl_);

  collect_bar_ = new QProgressBar(this);
  collect_bar_->setRange(0, 0);   // indeterminate until collecting progress is known
  collect_bar_->setTextVisible(false);
  collect_bar_->setFixedHeight(8);
  collect_bar_->setStyleSheet(progressBarStyle("#4a90d9"));
  collect_bar_->hide();
  pipeline_lay->addWidget(collect_bar_);

  root->addWidget(pipeline_box);

  // ── Last-match metrics ────────────────────────────────────────────────────
  auto* metrics_box = new QGroupBox("Last Match", this);
  metrics_box->setStyleSheet(pipeline_box->styleSheet());
  auto* metrics_grid = new QGridLayout(metrics_box);
  metrics_grid->setSpacing(4);

  auto make_metric_row = [&](int row, const QString& label,
                              QProgressBar*& bar, QLabel*& val_lbl,
                              const QString& bar_color)
  {
    metrics_grid->addWidget(new QLabel(label, this), row, 0);
    bar = new QProgressBar(this);
    bar->setRange(0, 100);
    bar->setValue(0);
    bar->setTextVisible(false);
    bar->setFixedHeight(14);
    bar->setStyleSheet(progressBarStyle(bar_color));
    metrics_grid->addWidget(bar, row, 1);
    val_lbl = new QLabel("—", this);
    val_lbl->setFixedWidth(60);
    val_lbl->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    metrics_grid->addWidget(val_lbl, row, 2);
  };

  make_metric_row(0, "MAD:",   mad_bar_,   mad_lbl_,   "#4a90d9");
  make_metric_row(1, "Disc:",  disc_bar_,  disc_lbl_,  "#50c878");
  make_metric_row(2, "Rough:", rough_bar_, rough_lbl_, "#e8a020");
  make_metric_row(3, "Noise:", noise_bar_, noise_lbl_, "#a070d0");

  root->addWidget(metrics_box);

  // ── Decision badge ────────────────────────────────────────────────────────
  decision_badge_ = new QLabel("WAITING", this);
  decision_badge_->setAlignment(Qt::AlignCenter);
  decision_badge_->setStyleSheet(
    "font-weight: bold; font-size: 14px; padding: 6px; border-radius: 4px;"
    "background: #333; color: #888;");
  root->addWidget(decision_badge_);

  // ── Session stats ─────────────────────────────────────────────────────────
  auto* session_box = new QGroupBox("Session", this);
  session_box->setStyleSheet(pipeline_box->styleSheet());
  auto* session_lay = new QGridLayout(session_box);
  session_lay->setSpacing(3);

  session_lbl_ = new QLabel("Accepted: 0  Rejected: 0", this);
  session_lbl_->setAlignment(Qt::AlignCenter);
  session_lay->addWidget(session_lbl_, 0, 0, 1, 2);

  reasons_lbl_ = new QLabel("", this);
  reasons_lbl_->setAlignment(Qt::AlignCenter);
  reasons_lbl_->setStyleSheet("color: #999; font-size: 10px;");
  session_lay->addWidget(reasons_lbl_, 1, 0, 1, 2);

  root->addWidget(session_box);
  root->addStretch();

  // ── Cross-thread signals ──────────────────────────────────────────────────
  connect(this, &TercomQualityPanel::statusReceived,
          this, &TercomQualityPanel::onStatusReceived,
          Qt::QueuedConnection);
  connect(this, &TercomQualityPanel::qualityReceived,
          this, &TercomQualityPanel::onQualityReceived,
          Qt::QueuedConnection);
  connect(this, &TercomQualityPanel::rejectionReceived,
          this, &TercomQualityPanel::onRejectionReceived,
          Qt::QueuedConnection);
}

// ── onInitialize ──────────────────────────────────────────────────────────────

void TercomQualityPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  resubscribe();
}

// ── save / load ───────────────────────────────────────────────────────────────

void TercomQualityPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("TopicStatus",    topic_status_);
  config.mapSetValue("TopicQuality",   topic_quality_);
  config.mapSetValue("TopicRejection", topic_rejection_);
}

void TercomQualityPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
  QString val;
  if (config.mapGetString("TopicStatus", &val)) {
    topic_status_ = val;
    topic_status_edit_->setText(val);
  }
  if (config.mapGetString("TopicQuality", &val)) {
    topic_quality_ = val;
    topic_quality_edit_->setText(val);
  }
  if (config.mapGetString("TopicRejection", &val)) {
    topic_rejection_ = val;
    topic_rejection_edit_->setText(val);
  }
}

// ── topic management ──────────────────────────────────────────────────────────

void TercomQualityPanel::resubscribe()
{
  if (!node_) return;

  sub_status_.reset();
  sub_quality_.reset();
  sub_rejection_.reset();

  sub_status_ = node_->create_subscription<std_msgs::msg::String>(
    topic_status_.toStdString(), rclcpp::QoS(10),
    [this](std_msgs::msg::String::ConstSharedPtr msg) {
      Q_EMIT statusReceived(QString::fromStdString(msg->data));
    });

  // Published format: [MAD(m), discrimination, roughness(m), noise_scale]
  // Only published on accepted fixes — quality msg implies match was accepted.
  sub_quality_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    topic_quality_.toStdString(), rclcpp::QoS(10),
    [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
      Q_EMIT qualityReceived(std::vector<float>(msg->data.begin(), msg->data.end()));
    });

  // Published as a String (e.g. "MAD>30m", "disc<1.02", "rough<5m") on every rejected fix.
  sub_rejection_ = node_->create_subscription<std_msgs::msg::String>(
    topic_rejection_.toStdString(), rclcpp::QoS(10),
    [this](std_msgs::msg::String::ConstSharedPtr msg) {
      Q_EMIT rejectionReceived(QString::fromStdString(msg->data));
    });
}

void TercomQualityPanel::onTopicEdited()
{
  topic_status_    = topic_status_edit_->text().trimmed();
  topic_quality_   = topic_quality_edit_->text().trimmed();
  topic_rejection_ = topic_rejection_edit_->text().trimmed();
  resubscribe();
}

// ── slots ─────────────────────────────────────────────────────────────────────

void TercomQualityPanel::onStatusReceived(const QString& status)
{
  status_lbl_->setText(status);

  // Colour-code background by state
  struct { const char* key; const char* bg; const char* fg; } palette[] = {
    {"COLLECTING",      "#1a3a5c", "#4a90d9"},
    {"MATCHING",        "#1a3a1a", "#50c878"},
    {"WAITING_SENSORS", "#3d2a00", "#e8a020"},
    {"ERROR",           "#3d1a0d", "#e67e22"},
    {nullptr,           "#333",    "#888"},
  };
  const char* bg = "#333";
  const char* fg = "#888";
  for (int i = 0; palette[i].key; ++i) {
    if (status.toUpper().contains(palette[i].key)) {
      bg = palette[i].bg; fg = palette[i].fg; break;
    }
  }
  status_lbl_->setStyleSheet(
    QString("font-weight: bold; font-size: 13px; padding: 4px;"
            "background: %1; color: %2; border-radius: 3px;").arg(bg).arg(fg));

  if (status.toUpper().startsWith("COLLECTING")) {
    collect_bar_->show();
    // Parse "COLLECTING N/M" to show determinate progress
    QStringList parts = status.split(' ');
    if (parts.size() >= 2) {
      QStringList nm = parts[1].split('/');
      if (nm.size() == 2) {
        int n = nm[0].toInt();
        int m = nm[1].toInt();
        if (m > 0) {
          collect_bar_->setRange(0, m);
          collect_bar_->setValue(n);
          collect_bar_->setTextVisible(true);
          collect_bar_->setFormat(QString("%1 / %2 pts").arg(n).arg(m));
        }
      }
    }
  } else {
    collect_bar_->hide();
  }
}

void TercomQualityPanel::onQualityReceived(const std::vector<float>& data)
{
  // Actual published layout: [MAD(m), discrimination, roughness(m), noise_scale]
  // This message is only published when a fix is accepted by tercom_node.
  if (data.size() < 4) return;

  const float mad            = data[0];
  const float discrimination = data[1];
  const float roughness      = data[2];
  const float noise          = data[3];

  // MAD bar: 0–30 m → 0–100 %; green when ≤ 15, yellow ≤ 25, red > 25
  {
    int pct = static_cast<int>(std::min(mad / 30.f, 1.f) * 100.f);
    mad_bar_->setValue(pct);
    QString color = (mad > 25.f) ? "#e74c3c" : (mad > 15.f) ? "#e8a020" : "#50c878";
    mad_bar_->setStyleSheet(progressBarStyle(color));
    mad_lbl_->setText(QString("%1 m").arg(static_cast<double>(mad), 0, 'f', 1));
    mad_lbl_->setStyleSheet(QString("color:%1;").arg(color));
  }

  // Discrimination bar: 0–2.0 range; higher is better
  // Good: > 1.5, marginal 1.02–1.5, poor < 1.02
  {
    int pct = static_cast<int>(std::min(discrimination / 2.f, 1.f) * 100.f);
    disc_bar_->setValue(pct);
    QString color = (discrimination < 1.02f) ? "#e74c3c"
                  : (discrimination < 1.5f)  ? "#e8a020"
                  : "#50c878";
    disc_bar_->setStyleSheet(progressBarStyle(color));
    disc_lbl_->setText(QString("%1").arg(static_cast<double>(discrimination), 0, 'f', 3));
    disc_lbl_->setStyleSheet(QString("color:%1;").arg(color));
  }

  // Roughness bar: 0–30 m; red when < 5 m (flat terrain), green when > 10 m
  {
    int pct = static_cast<int>(std::min(roughness / 30.f, 1.f) * 100.f);
    rough_bar_->setValue(pct);
    QString color = (roughness < 5.f)  ? "#e74c3c"
                  : (roughness < 10.f) ? "#e8a020"
                  : "#50c878";
    rough_bar_->setStyleSheet(progressBarStyle(color));
    rough_lbl_->setText(QString("%1 m").arg(static_cast<double>(roughness), 0, 'f', 1));
    rough_lbl_->setStyleSheet(QString("color:%1;").arg(color));
  }

  // Noise scale bar: 0–5 scale; green ≤ 1.0, yellow ≤ 2.0, red > 2.0
  {
    int pct = static_cast<int>(std::min(noise / 5.f, 1.f) * 100.f);
    noise_bar_->setValue(pct);
    QString color = (noise > 2.f) ? "#e74c3c" : (noise > 1.f) ? "#e8a020" : "#50c878";
    noise_bar_->setStyleSheet(progressBarStyle(color));
    noise_lbl_->setText(QString("×%1").arg(static_cast<double>(noise), 0, 'f', 2));
    noise_lbl_->setStyleSheet(QString("color:%1;").arg(color));
  }

  // Decision badge — quality_msg is only published on accepted fixes.
  // "good" thresholds aligned with classify_terrain_quality() in terrain_quality.py:
  //   good_disc  = disc   > discrimination_min(1.02) * 1.5 → > 1.53
  //   good_rough = rough  > roughness_min(5m) * 2          → > 10 m
  //   good_mad   = mad    < mad_threshold(30m) * 0.5       → < 15 m
  ++accepted_count_;
  const bool marginal_mad   = mad          > 15.f;
  const bool marginal_disc  = discrimination < 1.5f;
  const bool marginal_rough = roughness    < 10.f;
  const bool marginal       = marginal_mad || marginal_disc || marginal_rough;

  if (marginal) {
    QStringList why;
    if (marginal_mad)   why << QString("MAD %1m").arg(static_cast<double>(mad), 0, 'f', 1);
    if (marginal_disc)  why << QString("disc %1").arg(static_cast<double>(discrimination), 0, 'f', 2);
    if (marginal_rough) why << QString("rough %1m").arg(static_cast<double>(roughness), 0, 'f', 1);
    decision_badge_->setText("ACCEPTED (marginal)");
    decision_badge_->setStyleSheet(
      "font-weight:bold; font-size:13px; padding:6px; border-radius:4px;"
      "background:#3d3300; color:#f1c40f;");
    reasons_lbl_->setText("↓ " + why.join("  "));
    reasons_lbl_->setStyleSheet("color:#f1c40f; font-size:10px;");
  } else {
    decision_badge_->setText("ACCEPTED");
    decision_badge_->setStyleSheet(
      "font-weight:bold; font-size:14px; padding:6px; border-radius:4px;"
      "background:#0d3d1a; color:#2ecc71;");
    reasons_lbl_->clear();
  }

  session_lbl_->setText(
    QString("Accepted: %1   Rejected: %2").arg(accepted_count_).arg(rejected_count_));
}

void TercomQualityPanel::onRejectionReceived(const QString& reason)
{
  // rejection_reason is published by tercom_node on every rejected fix.
  ++rejected_count_;

  // Flash the badge to show the rejection (the next quality message will override it)
  decision_badge_->setText(reason.isEmpty() ? "REJECTED" : "REJECTED");
  decision_badge_->setStyleSheet(
    "font-weight:bold; font-size:14px; padding:6px; border-radius:4px;"
    "background:#3d0d0d; color:#e74c3c;");

  // Show rejection reason underneath
  reasons_lbl_->setText("✗ " + reason);
  reasons_lbl_->setStyleSheet("color:#e74c3c; font-size:10px;");

  session_lbl_->setText(
    QString("Accepted: %1   Rejected: %2").arg(accepted_count_).arg(rejected_count_));
}

}  // namespace tercom_rviz_plugins

PLUGINLIB_EXPORT_CLASS(tercom_rviz_plugins::TercomQualityPanel, rviz_common::Panel)
