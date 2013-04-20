#include "statistics.h"
#include "main_window.h"
#include "statistics_viewer_widget.h"

#include "point_cloud.h"

void PointCloud::updateStatistics(void)
{
  Statistics* statistics = MainWindow::getInstance()->getStatisticsViewerWidget()->getStatistics();

  int frame = getFrame();
  for (size_t i = 0, i_end = leaves_.size(); i < i_end; ++ i)
    statistics->addLeafSample(i, frame, leaves_[i].computeArea());

  for (size_t i = 0, i_end = stems_.size(); i < i_end; ++ i)
    statistics->addStemSample(i, frame, stems_[i].computeArea());

  return;
}