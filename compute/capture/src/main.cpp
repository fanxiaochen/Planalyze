#include <QtGui/QApplication>

#include "main_window.h"

int main(int argc, char *argv[])
{
  QApplication application(argc, argv);
  MainWindow main_window;
  main_window.showMaximized();

  return application.exec();
}
