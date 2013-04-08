#pragma once
#ifndef PLAIN_TEXT_VIEWER_H
#define PLAIN_TEXT_VIEWER_H

#include <QPlainTextEdit>

class PlainTextViewer : public QPlainTextEdit
{
  Q_OBJECT

public:
  PlainTextViewer(QWidget * parent = 0);
  ~PlainTextViewer(void);

  virtual QSize
    sizeHint() const {return QSize(width_, height_);}

private:
  int width_;
  int height_;
};

#endif /*PLAIN_TEXT_VIEWER_H*/