#include "statistics.h"
#include "main_window.h"
#include "statistics_viewer_widget.h"

#include "point_cloud.h"

void PointCloud::updateStatistics(void)
{
  Statistics* statistics = MainWindow::getInstance()->getStatisticsViewerWidget()->getStatistics();

  statistics->addOtherSample("Points", getFrame(), size());

  return;
}