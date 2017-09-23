#include "g_main_window.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    G_MAIN_WINDOW w;
    w.show();

    return a.exec();
}
