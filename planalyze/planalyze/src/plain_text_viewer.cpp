#include "plain_text_viewer.h"

PlainTextViewer::PlainTextViewer(QWidget * parent)
  : QPlainTextEdit(parent)
{
  setReadOnly(true);
  setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
}

PlainTextViewer::~PlainTextViewer(void)
{
}
