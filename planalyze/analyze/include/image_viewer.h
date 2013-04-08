#pragma once
#ifndef IMAGE_VIEWER_H
#define IMAGE_VIEWER_H

#include <QLabel>

class ImageViewer : public QLabel
{
public:
  ImageViewer(QWidget * parent = 0, Qt::WindowFlags f = 0);
  ~ImageViewer(void);

  virtual QSize sizeHint() const {return QSize(1280, 960);}
};

#endif /*IMAGE_VIEWER_H*/