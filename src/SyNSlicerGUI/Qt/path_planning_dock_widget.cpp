#include "path_planning_dock_widget.h"

using SyNSlicerGUI::PathPlanningDockWidget;

PathPlanningDockWidget::PathPlanningDockWidget(vtkRenderer *input_renderer, QWidget *parent)
	: QDockWidget(parent)
	, mp_renderer(input_renderer)
	, mp_render_window(mp_renderer->GetRenderWindow())
	, mp_render_window_interactor(mp_render_window->GetInteractor())
	, mp_default_interactor_style(vtkInteractorStyleTrackballCamera::New())
	, m_drawer(input_renderer)
	, m_widget(nullptr)
	, m_grid_layout(nullptr)
	, m_tree_widget(nullptr)
	, m_reset_button(tr("Reset All"), nullptr)
	, m_partitions()
{
	mp_render_window = mp_renderer->GetRenderWindow();
	mp_render_window_interactor = mp_render_window->GetInteractor();
	mp_default_interactor_style = vtkInteractorStyleTrackballCamera::New();

	this->setWidget(&m_widget);
	m_widget.setLayout(&m_grid_layout);

	m_grid_layout.addWidget(&m_tree_widget);
	m_grid_layout.addWidget(&m_reset_button);

	QObject::connect(&m_tree_widget, SIGNAL(changePartitionVisibility(int, bool)), this, SLOT(changeShowOrHidePartition(int, bool)));

	model_name = std::string("partition_list.txt");
	m_partitions.load(model_name);

	this->updateTreeWidget();
}

PathPlanningDockWidget::~PathPlanningDockWidget()
{
	mp_default_interactor_style->Delete();
}

void PathPlanningDockWidget::changeShowOrHidePartition(int partition_index, bool visible)
{
	std::string name = std::string("Partition") + std::to_string(partition_index);
	m_drawer.setVisible(name, visible);
}

void PathPlanningDockWidget::updateTreeWidget()
{
	m_drawer.removeAllObjectsDrawn();

	for (int i = 0; i < m_partitions.numberOfPartitions(); i++)
	{
		std::string name = std::string("Partition") + std::to_string(i);
		m_drawer.drawMesh(m_partitions[i].getEPICKMesh(), name);
		m_drawer.setColor(name,
			255.0 / m_partitions.numberOfPartitions() * i,
			255.0 / m_partitions.numberOfPartitions() * i,
			255.0 / m_partitions.numberOfPartitions() * i);
	}

	// Tree widget only has one top item.
	if (m_tree_widget.topLevelItemCount() > 0)
	{
		m_tree_widget.takeTopLevelItem(0);
	}

	m_tree_widget.addTopLevelItem(model_name);
	m_tree_widget.addChildItems(0, m_partitions.numberOfPartitions());
}