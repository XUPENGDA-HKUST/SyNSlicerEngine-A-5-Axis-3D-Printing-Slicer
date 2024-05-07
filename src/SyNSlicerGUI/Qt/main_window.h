#ifndef SYNSLICERGUI_MAINWINDOW_H_
#define SYNSLICERGUI_MAINWINDOW_H_

#include <qmainwindow.h>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

#include "central_widget.h"
#include "menu_bar.h"
#include "path_planning_dock_widget.h"

namespace SyNSlicerGUI {

    class MainWindow : public QMainWindow
    {
        Q_OBJECT

    public:
        MainWindow();
        ~MainWindow();

    protected slots:

    protected:
        CentralWidget m_central_widget;
        MenuBar m_menu_bar;
        PathPlanningDockWidget m_path_planning_dock_widget;
    };
}

#endif  // SYNSLICERGUI_MAINWINDOW_H_
