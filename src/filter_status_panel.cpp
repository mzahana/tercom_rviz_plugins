#include "tercom_rviz_plugins/filter_status_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QString>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <cmath>

namespace tercom_rviz_plugins {

// ── Helpers ───────────────────────────────────────────────────────────────────

QString FilterStatusPanel::badgeStyleSheet(const QString& bg, const QString& fg) {
  return QString(
    "background:%1; color:%2; padding:4px; border-radius:4px; font-weight:bold;")
    .arg(bg, fg);
}

QString FilterStatusPanel::progressBarStyleSheet(float value, float limit) {
  float ratio = (limit > 0.f) ? (value / limit) : 0.f;
  QString color;
  if (ratio < 0.60f)      color = "#2ecc71";
  else if (ratio < 0.85f) color = "#f1c40f";
  else                    color = "#e74c3c";
  return QString("QProgressBar::chunk { background: %1; }").arg(color);
}

QString FilterStatusPanel::biasLabelColor(float magnitude) {
  if (magnitude > 0.05f) return "#e74c3c";
  if (magnitude > 0.02f) return "#ff9800";
  return "";
}

void FilterStatusPanel::applyBiasColor(QLabel* lbl, float val) {
  QString color = biasLabelColor(std::fabs(val));
  if (color.isEmpty())
    lbl->setStyleSheet("");
  else
    lbl->setStyleSheet(QString("color:%1;").arg(color));
  lbl->setText(QString::number(static_cast<double>(val), 'f', 4));
}

// ── Constructor ───────────────────────────────────────────────────────────────

FilterStatusPanel::FilterStatusPanel(QWidget* parent)
    : rviz_common::Panel(parent)
{
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(6, 6, 6, 6);
  root->setSpacing(4);

  // ── Health LED ────────────────────────────────────────────────────────────
  // Full-width banner: easy-to-spot at a glance.
  // Levels: OFFLINE (grey) · INITIALIZING (blue) · HEALTHY (green) ·
  //         DEGRADED (amber) · CRITICAL (red)
  health_led_ = new QLabel("⬤  OFFLINE", this);
  health_led_->setAlignment(Qt::AlignCenter);
  health_led_->setMinimumHeight(34);
  health_led_->setStyleSheet(
    "font-size:15px; font-weight:bold; padding:5px 8px; border-radius:5px;"
    "background:#2a2a2a; color:#666;");
  root->addWidget(health_led_);

  // ── State badge ───────────────────────────────────────────────────────────
  state_badge_ = new QLabel("UNKNOWN", this);
  state_badge_->setAlignment(Qt::AlignCenter);
  state_badge_->setStyleSheet(badgeStyleSheet("#607d8b", "#fff"));
  root->addWidget(state_badge_);

  // NIS sparkline group
  auto* nis_group = new QGroupBox("NIS — last 60 s", this);
  auto* nis_lay   = new QVBoxLayout(nis_group);
  nis_sparkline_  = new SparklineWidget(nis_group);
  nis_sparkline_->setLabel("NIS");
  nis_lay->addWidget(nis_sparkline_);
  root->addWidget(nis_group);

  // Health group
  auto* health_group = new QGroupBox("Health", this);
  auto* health_lay   = new QVBoxLayout(health_group);

  auto* sigma_row = new QHBoxLayout();
  sigma_row->addWidget(new QLabel("Max σ_pos", health_group));
  sigma_bar_ = new QProgressBar(health_group);
  sigma_bar_->setRange(0, 100);
  sigma_bar_->setTextVisible(false);
  sigma_row->addWidget(sigma_bar_);
  sigma_lbl_ = new QLabel("0.0 m", health_group);
  sigma_lbl_->setMinimumWidth(55);
  sigma_row->addWidget(sigma_lbl_);
  health_lay->addLayout(sigma_row);

  auto* innov_row = new QHBoxLayout();
  innov_row->addWidget(new QLabel("Innov norm", health_group));
  innov_bar_ = new QProgressBar(health_group);
  innov_bar_->setRange(0, 100);
  innov_bar_->setTextVisible(false);
  innov_row->addWidget(innov_bar_);
  innov_lbl_ = new QLabel("0.00", health_group);
  innov_lbl_->setMinimumWidth(55);
  innov_row->addWidget(innov_lbl_);
  health_lay->addLayout(innov_row);

  root->addWidget(health_group);

  // IMU Bias group
  auto* bias_group = new QGroupBox("IMU Bias", this);
  auto* bias_grid  = new QGridLayout(bias_group);

  bias_grid->addWidget(new QLabel("Accel", bias_group), 0, 0);
  ax_ = new QLabel("0.0000", bias_group);
  ay_ = new QLabel("0.0000", bias_group);
  az_ = new QLabel("0.0000", bias_group);
  bias_grid->addWidget(ax_, 0, 1);
  bias_grid->addWidget(ay_, 0, 2);
  bias_grid->addWidget(az_, 0, 3);
  bias_grid->addWidget(new QLabel("m/s²", bias_group), 0, 4);

  bias_grid->addWidget(new QLabel("Gyro", bias_group), 1, 0);
  gx_ = new QLabel("0.0000", bias_group);
  gy_ = new QLabel("0.0000", bias_group);
  gz_ = new QLabel("0.0000", bias_group);
  bias_grid->addWidget(gx_, 1, 1);
  bias_grid->addWidget(gy_, 1, 2);
  bias_grid->addWidget(gz_, 1, 3);
  bias_grid->addWidget(new QLabel("rad/s", bias_group), 1, 4);

  root->addWidget(bias_group);
  root->addStretch();

  // Connect signals → slots (cross-thread safe)
  connect(this, &FilterStatusPanel::stateReceived,
          this, &FilterStatusPanel::onStateReceived,
          Qt::QueuedConnection);
  connect(this, &FilterStatusPanel::healthReceived,
          this, &FilterStatusPanel::onHealthReceived,
          Qt::QueuedConnection);
  connect(this, &FilterStatusPanel::nisHistoryReceived,
          this, &FilterStatusPanel::onNisHistoryReceived,
          Qt::QueuedConnection);
  connect(this, &FilterStatusPanel::biasAccelReceived,
          this, &FilterStatusPanel::onBiasAccelReceived,
          Qt::QueuedConnection);
  connect(this, &FilterStatusPanel::biasGyroReceived,
          this, &FilterStatusPanel::onBiasGyroReceived,
          Qt::QueuedConnection);
}

// ── onInitialize ──────────────────────────────────────────────────────────────

void FilterStatusPanel::onInitialize() {
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  sub_state_ = node_->create_subscription<std_msgs::msg::String>(
    "/tercom/eskf_node/state", rclcpp::QoS(10),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      Q_EMIT stateReceived(QString::fromStdString(msg->data));
    });

  sub_health_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/tercom/eskf_node/health", rclcpp::QoS(10),
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
      Q_EMIT healthReceived(std::vector<float>(msg->data.begin(), msg->data.end()));
    });

  sub_nis_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/tercom/diagnostics_node/nis_history", rclcpp::QoS(10),
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
      Q_EMIT nisHistoryReceived(std::vector<float>(msg->data.begin(), msg->data.end()));
    });

  sub_bias_accel_ = node_->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "/tercom/eskf_node/bias_accel", rclcpp::QoS(10),
    [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
      Q_EMIT biasAccelReceived(
        static_cast<float>(msg->vector.x),
        static_cast<float>(msg->vector.y),
        static_cast<float>(msg->vector.z));
    });

  sub_bias_gyro_ = node_->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "/tercom/eskf_node/bias_gyro", rclcpp::QoS(10),
    [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
      Q_EMIT biasGyroReceived(
        static_cast<float>(msg->vector.x),
        static_cast<float>(msg->vector.y),
        static_cast<float>(msg->vector.z));
    });
}

// ── Slots ─────────────────────────────────────────────────────────────────────

void FilterStatusPanel::onStateReceived(const QString& state) {
  current_state_ = state;
  state_badge_->setText(state);
  if (state == "RUNNING")
    state_badge_->setStyleSheet(badgeStyleSheet("#2ecc71", "#000"));
  else if (state == "INITIALIZING")
    state_badge_->setStyleSheet(badgeStyleSheet("#00bcd4", "#000"));
  else if (state == "WAITING_GPS")
    state_badge_->setStyleSheet(badgeStyleSheet("#9e9e9e", "#fff"));
  else if (state == "DIVERGED")
    state_badge_->setStyleSheet(badgeStyleSheet("#ff9800", "#000"));
  else if (state == "RESETTING")
    state_badge_->setStyleSheet(badgeStyleSheet("#e74c3c", "#fff"));
  else
    state_badge_->setStyleSheet(badgeStyleSheet("#607d8b", "#fff"));
  updateHealthLed();
}

void FilterStatusPanel::onHealthReceived(const std::vector<float>& data) {
  if (data.size() < 3) return;
  current_avg_nis_ = data[0];
  current_sigma_   = data[1];
  current_innov_   = data[2];

  // sigma_limit: 200 m covers the initialization phase (GPS init + early filter).
  // Green (<120 m) = converging/converged, yellow (120–170 m) = marginal,
  // red (>170 m) = high uncertainty (e.g., no TERCOM fixes yet).
  const float sigma_limit = 200.f;
  // innov_limit: TERCOM innovation norm; 50 m corresponds to half the initial
  // search radius. Values above this suggest poor DEM alignment or large drift.
  const float innov_limit = 50.f;

  sigma_bar_->setValue(static_cast<int>(std::min(current_sigma_ / sigma_limit, 1.f) * 100));
  sigma_bar_->setStyleSheet(progressBarStyleSheet(current_sigma_, sigma_limit));
  sigma_lbl_->setText(QString::number(static_cast<double>(current_sigma_), 'f', 1) + " m");

  innov_bar_->setValue(static_cast<int>(std::min(current_innov_ / innov_limit, 1.f) * 100));
  innov_bar_->setStyleSheet(progressBarStyleSheet(current_innov_, innov_limit));
  innov_lbl_->setText(QString::number(static_cast<double>(current_innov_), 'f', 1) + " m");

  updateHealthLed();
}

void FilterStatusPanel::updateHealthLed()
{
  // Combine state + metrics into a single easy-to-read indicator.
  //
  // CRITICAL (red)     : DIVERGED / RESETTING, or sigma > 170 m while RUNNING
  // DEGRADED (amber)   : RUNNING but sigma 120–170 m, or NIS > 5.99
  // HEALTHY  (green)   : RUNNING, sigma < 120 m, NIS ≤ 5.99
  // INITIALIZING (blue): WAITING_GPS / INITIALIZING
  // OFFLINE  (grey)    : unknown / no data

  struct Level { const char* icon; const char* text; const char* bg; const char* fg; };
  Level level;

  if (current_state_ == "RUNNING") {
    const bool nis_bad    = current_avg_nis_ > 5.99f;
    const bool sigma_crit = current_sigma_   > 170.f;
    const bool sigma_warn = current_sigma_   > 120.f;

    if (sigma_crit) {
      level = {"⬤", "CRITICAL — σ too large", "#4a0a0a", "#ff5555"};
    } else if (sigma_warn || nis_bad) {
      level = {"⬤", "DEGRADED", "#4a3800", "#ffc107"};
    } else {
      level = {"⬤", "HEALTHY", "#0a3d1a", "#4cfc6c"};
    }
  } else if (current_state_ == "DIVERGED" || current_state_ == "RESETTING") {
    level = {"⬤", "CRITICAL — filter diverged", "#4a0a0a", "#ff5555"};
  } else if (current_state_ == "INITIALIZING") {
    level = {"⬤", "INITIALIZING", "#0a2a4a", "#4dc8ff"};
  } else if (current_state_ == "WAITING_GPS") {
    level = {"⬤", "WAITING FOR GPS", "#1a1a2a", "#7090cc"};
  } else {
    level = {"⬤", "OFFLINE", "#1e1e1e", "#555"};
  }

  health_led_->setText(QString("%1  %2").arg(level.icon).arg(level.text));
  health_led_->setStyleSheet(
    QString("font-size:15px; font-weight:bold; padding:5px 8px; border-radius:5px;"
            "background:%1; color:%2;").arg(level.bg).arg(level.fg));
}

void FilterStatusPanel::onNisHistoryReceived(const std::vector<float>& data) {
  nis_sparkline_->setData(data, 5.99f);
}

void FilterStatusPanel::onBiasAccelReceived(float x, float y, float z) {
  applyBiasColor(ax_, x);
  applyBiasColor(ay_, y);
  applyBiasColor(az_, z);
}

void FilterStatusPanel::onBiasGyroReceived(float x, float y, float z) {
  applyBiasColor(gx_, x);
  applyBiasColor(gy_, y);
  applyBiasColor(gz_, z);
}

}  // namespace tercom_rviz_plugins

PLUGINLIB_EXPORT_CLASS(tercom_rviz_plugins::FilterStatusPanel, rviz_common::Panel)
