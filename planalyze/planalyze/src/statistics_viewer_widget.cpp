#include "statistics_viewer_widget.h"

StatisticsViewerWidget::StatisticsViewerWidget(QWidget * parent)
  : QPlainTextEdit(parent)
{
  setReadOnly(true);
  setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
}

StatisticsViewerWidget::~StatisticsViewerWidget(void)
{
}
