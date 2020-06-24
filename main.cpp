#include "pclviewer.h"
#include <QApplication>
#include <QMainWindow>

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);
  pclviewer w(argc, argv);
  w.showMaximized();
  return a.exec ();
}