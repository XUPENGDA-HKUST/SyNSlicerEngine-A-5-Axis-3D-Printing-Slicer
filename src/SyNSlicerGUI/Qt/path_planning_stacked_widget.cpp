#include "path_planning_stacked_widget.h"

using SyNSlicerGUI::PathPlanningStackedWidget;

PathPlanningStackedWidget::PathPlanningStackedWidget(QWidget *parent)
	: QStackedWidget(parent)
{
	this->addWidget(&m_mesh_loading_widget);
	this->addWidget(&m_model_partition_widget);
	this->addWidget(&m_toolpath_generating_widget);
	this->addWidget(&m_preview_widget);
}

PathPlanningStackedWidget::~PathPlanningStackedWidget()
{

}
