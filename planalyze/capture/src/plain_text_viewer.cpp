#include "plain_text_viewer.h"

PlainTextViewer::PlainTextViewer(QWidget * parent)
  : QPlainTextEdit(parent),
  width_(640),
  height_(256)
{
  setReadOnly(true);
  setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
}

PlainTextViewer::~PlainTextViewer(void)
{
}
