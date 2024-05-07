#include "main_window.h"

using SyNSlicerGUI::MainWindow;

MainWindow::MainWindow()
    : m_central_widget(CentralWidget(this))
    , m_menu_bar(MenuBar(this))
    , m_path_planning_dock_widget(m_central_widget.getVTKRenderer())
{
    auto my_logger = spdlog::basic_logger_mt("basic_logger", "logs/basic.txt", true);
    spdlog::get("basic_logger")->info("Start program.");

    this->resize(1280,720);
    this->setWindowTitle("SyNSlicer");
    this->setCentralWidget(&m_central_widget);
    this->setMenuBar(&m_menu_bar);

    m_path_planning_dock_widget.setWindowTitle(tr("Path Planning"));
    m_path_planning_dock_widget.setAllowedAreas(Qt::RightDockWidgetArea);
    this->addDockWidget(Qt::RightDockWidgetArea, &m_path_planning_dock_widget);

    spdlog::get("basic_logger")->info("Construct MainWindow");
}

MainWindow::~MainWindow()
{
    spdlog::get("basic_logger")->info("End program.");
    spdlog::get("basic_logger")->flush();
}