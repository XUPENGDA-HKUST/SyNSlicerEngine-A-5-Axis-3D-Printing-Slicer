#include <qapplication.h>
#include "SyNSlicerGUI/Qt/main_window.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    SyNSlicerGUI::MainWindow main_window;
    main_window.move(300, 50);
    main_window.show();
    QApplication::processEvents();
    return a.exec();
}