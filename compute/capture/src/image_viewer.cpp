#include "image_viewer.h"

ImageViewer::ImageViewer(QWidget * parent, Qt::WindowFlags f)
  : QLabel(parent, f)
{
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

ImageViewer::~ImageViewer(void)
{
}
