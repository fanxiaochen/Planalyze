#pragma once
#ifndef STATISTICS_VIEWER_WIDGET_H
#define STATISTICS_VIEWER_WIDGET_H

#include <qwt_plot.h>

class Statistics;

class StatisticsViewerWidget : public QwtPlot
{
  Q_OBJECT

public:
  StatisticsViewerWidget(QWidget * parent = 0);
  virtual ~StatisticsViewerWidget(void);

  virtual QSize sizeHint() const {return QSize(320, 256);}

  Statistics* getStatistics(void) {return statistics_;}

private:
  Statistics*     statistics_;
};

#endif /*STATISTICS_VIEWER_WIDGET_H*/