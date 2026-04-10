#include "tercom_rviz_plugins/profiling_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QFont>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace tercom_rviz_plugins {

// ── Row metadata (node name, component name) ──────────────────────────────────

static const char* const ROW_NODE[8] = {
  "tercom_node", "tercom_node",
  "eskf_node",   "eskf_node",   "eskf_node",
  "diagnostics", "diagnostics", "diagnostics",
};

static const char* const ROW_COMPONENT[8] = {
  "cb_synced",    "run_matching",
  "cb_imu",       "cb_tercom_fix", "cb_altitude",
  "timer_error",  "timer_paths",   "timer_stats",
};

// ── constexpr definition (required for ODR) ───────────────────────────────────
constexpr ProfilingPanel::RowThresholds ProfilingPanel::THRESHOLDS[8];

// ── Constructor ───────────────────────────────────────────────────────────────

ProfilingPanel::ProfilingPanel(QWidget* parent)
    : rviz_common::Panel(parent)
{
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(6, 6, 6, 6);
  root->setSpacing(4);

  // ── Status label ──────────────────────────────────────────────────────────
  status_label_ = new QLabel("⬤  WAITING FOR DATA", this);
  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setMinimumHeight(28);
  status_label_->setStyleSheet(
    "font-size:13px; font-weight:bold; padding:4px 8px; border-radius:4px;"
    "background:#2a2a2a; color:#888;");
  root->addWidget(status_label_);

  // ── Profiling table ───────────────────────────────────────────────────────
  auto* grp = new QGroupBox("Component Timing", this);
  auto* grp_lay = new QVBoxLayout(grp);

  table_ = new QTableWidget(8, 4, grp);
  table_->setHorizontalHeaderLabels({"Node", "Component", "Exec (ms)", "Rate (Hz)"});
  table_->verticalHeader()->setVisible(false);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->setSelectionMode(QAbstractItemView::NoSelection);
  table_->setAlternatingRowColors(true);
  table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
  table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);

  QFont mono("Monospace");
  mono.setStyleHint(QFont::Monospace);
  mono.setPointSize(9);

  for (int r = 0; r < 8; ++r) {
    auto* node_item = new QTableWidgetItem(ROW_NODE[r]);
    auto* comp_item = new QTableWidgetItem(ROW_COMPONENT[r]);
    auto* exec_item = new QTableWidgetItem("—");
    auto* hz_item   = new QTableWidgetItem("—");

    exec_item->setFont(mono);
    hz_item->setFont(mono);
    exec_item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
    hz_item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);

    table_->setItem(r, 0, node_item);
    table_->setItem(r, 1, comp_item);
    table_->setItem(r, 2, exec_item);
    table_->setItem(r, 3, hz_item);
  }

  grp_lay->addWidget(table_);
  root->addWidget(grp);

  // ── Watchdog: dim table when upstream is silent ───────────────────────────
  watchdog_timer_ = new QTimer(this);
  watchdog_timer_->setInterval(3000);  // 3 s
  connect(watchdog_timer_, &QTimer::timeout, this, &ProfilingPanel::onWatchdog);
  watchdog_timer_->start();
}

// ── RViz initialization ───────────────────────────────────────────────────────

void ProfilingPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/tercom/diagnostics_node/profiling", 10,
    [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
      updateTable(msg);
    });
}

// ── Slot: watchdog fires when no data received in 3 s ────────────────────────

void ProfilingPanel::onWatchdog()
{
  if (!data_received_) {
    status_label_->setStyleSheet(
      "font-size:13px; font-weight:bold; padding:4px 8px; border-radius:4px;"
      "background:#2a2a2a; color:#888;");
    status_label_->setText("⬤  WAITING FOR DATA");
  }
  data_received_ = false;  // reset for next interval
}

// ── Update table from incoming profiling message ──────────────────────────────

void ProfilingPanel::updateTable(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (static_cast<int>(msg->data.size()) < 16)
    return;

  data_received_ = true;

  status_label_->setStyleSheet(
    "font-size:13px; font-weight:bold; padding:4px 8px; border-radius:4px;"
    "background:#1a472a; color:#2ecc71;");
  status_label_->setText("⬤  LIVE");

  for (int r = 0; r < 8; ++r) {
    float exec_ms = msg->data[r * 2];
    float hz      = msg->data[r * 2 + 1];

    auto* exec_item = table_->item(r, 2);
    auto* hz_item   = table_->item(r, 3);

    exec_item->setText(QString::number(static_cast<double>(exec_ms), 'f', 2));
    hz_item->setText(hz > 0.f ? QString::number(static_cast<double>(hz), 'f', 1) : "—");

    // Color-code exec time cell
    QString color = execColor(exec_ms,
                               THRESHOLDS[r].green_ms,
                               THRESHOLDS[r].yellow_ms);
    exec_item->setBackground(QColor(color));
    // Ensure text is always readable against colored backgrounds
    exec_item->setForeground(QColor("#000000"));
  }
}

// ── Helper: map exec time to background color ─────────────────────────────────

QString ProfilingPanel::execColor(float exec_ms, float green_ms, float yellow_ms)
{
  if (exec_ms <= 0.f)            return "#3a3a3a";  // no data — dark grey
  if (exec_ms <= green_ms)       return "#27ae60";  // green
  if (exec_ms <= yellow_ms)      return "#f39c12";  // amber
  return "#e74c3c";                                  // red
}

}  // namespace tercom_rviz_plugins

PLUGINLIB_EXPORT_CLASS(tercom_rviz_plugins::ProfilingPanel, rviz_common::Panel)
