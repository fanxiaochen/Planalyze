#include "log_viewer_widget.h"

LogViewerWidget::LogViewerWidget(QWidget * parent)
  : QPlainTextEdit(parent)
{
  setReadOnly(true);
  setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
}

LogViewerWidget::~LogViewerWidget(void)
{
}
