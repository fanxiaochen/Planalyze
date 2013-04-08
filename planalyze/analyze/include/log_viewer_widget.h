#pragma once
#ifndef LOG_VIEWER_WIDGET_H
#define LOG_VIEWER_WIDGET_H

#include <QPlainTextEdit>

class LogViewerWidget : public QPlainTextEdit
{
  Q_OBJECT

public:
  LogViewerWidget(QWidget * parent = 0);
  virtual ~LogViewerWidget(void);

  virtual QSize
    sizeHint() const {return QSize(320, 256);}
};

#endif /*LOG_VIEWER_WIDGET_H*/