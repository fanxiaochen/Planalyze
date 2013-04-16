#pragma once
#ifndef STATISTICS_VIEWER_WIDGET_H
#define STATISTICS_VIEWER_WIDGET_H

#include <QPlainTextEdit>

class StatisticsViewerWidget : public QPlainTextEdit
{
  Q_OBJECT

public:
  StatisticsViewerWidget(QWidget * parent = 0);
  virtual ~StatisticsViewerWidget(void);

  virtual QSize
    sizeHint() const {return QSize(320, 256);}
};

#endif /*STATISTICS_VIEWER_WIDGET_H*/