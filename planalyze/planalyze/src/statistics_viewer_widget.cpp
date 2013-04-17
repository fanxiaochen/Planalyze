#include "statistics.h"

#include "statistics_viewer_widget.h"

StatisticsViewerWidget::StatisticsViewerWidget(QWidget * parent)
  : QwtPlot(parent),
  statistics_(new Statistics(this))
{
  setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
}

StatisticsViewerWidget::~StatisticsViewerWidget(void)
{
  delete statistics_;
}
