#include "path_planning_dock_widget.h"

using SyNSlicerGUI::PathPlanningDockWidget;

PathPlanningDockWidget::PathPlanningDockWidget(vtkRenderer *input_renderer, QWidget *parent)
	: QDockWidget(parent)
	, mp_renderer(input_renderer)
	, mp_render_window(mp_renderer->GetRenderWindow())
	, mp_render_window_interactor(mp_render_window->GetInteractor())
	, mp_default_interactor_style(vtkInteractorStyleTrackballCamera::New())
	, m_drawer(input_renderer)
	, m_drawer_for_preview(input_renderer)
	, m_widget(nullptr)
	, m_grid_layout(nullptr)
	, m_tree_widget(nullptr)
	, m_stacked_widget(nullptr)
	, m_reset_button(tr("Reset All"), nullptr)
	, m_operating_partition_index(0)
	, m_mesh_clipping_interactor_style(mp_renderer)
	, m_partition_selecting_interactor_style(mp_renderer)
	, m_mesh_clipper(mp_renderer)
{
	this->setWidget(&m_widget);
	m_widget.setLayout(&m_grid_layout);

	m_grid_layout.addWidget(&m_tree_widget);
	m_grid_layout.addWidget(&m_stacked_widget);
	m_grid_layout.addWidget(&m_reset_button);
	/*
	model_name = std::string("partition_list.txt");
	m_partitions.load("partition_list.txt");
	this->updateTreeWidget();
	*/
	QObject::connect(
		&m_reset_button, SIGNAL(clicked()),
		this, SLOT(reset()));

	QObject::connect(
		&m_stacked_widget.m_mesh_loading_widget.m_open_mesh_button, SIGNAL(clicked()),
		this, SLOT(openMesh()));

	QObject::connect(
		&m_stacked_widget.m_mesh_loading_widget.m_open_default_mesh_button, SIGNAL(clicked()),
		this, SLOT(openDefaultMesh()));

	QObject::connect(
		&m_tree_widget, SIGNAL(changePartitionVisibility(int, bool)), 
		this, SLOT(changeShowOrHidePartition(int, bool)));

	QObject::connect(
		&m_tree_widget, SIGNAL(clipPartition(int)),
		this, SLOT(setClipPartition(int)));

	QObject::connect(
		&m_stacked_widget.m_model_partition_widget.m_partition_mode_combo_box, SIGNAL(currentIndexChanged(int)),
		this, SLOT(changePartitionMode(int)));

	QObject::connect(
		&m_stacked_widget.m_model_partition_widget.m_auto_partition_button, SIGNAL(clicked()),
		this, SLOT(runAutoPartition()));

	QObject::connect(
		&m_stacked_widget.m_model_partition_widget.m_confirm_result_button, SIGNAL(clicked()),
		this, SLOT(confirmPartition()));

	QObject::connect(
		&m_stacked_widget.m_model_partition_widget.m_reset_button, SIGNAL(clicked()),
		this, SLOT(resetPartitionResult()));

	QObject::connect(
		&m_stacked_widget.m_model_partition_widget.m_exit_button, SIGNAL(clicked()),
		this, SLOT(exitPartition()));

	QObject::connect(
		&m_stacked_widget.m_model_partition_widget.m_infinite_plane_clipping_confirm_line_button, SIGNAL(clicked()),
		this, SLOT(clipPartition()));

	QObject::connect(
		&m_stacked_widget.m_model_partition_widget.m_finite_plane_clipping_confirm_line_button, SIGNAL(clicked()),
		this, SLOT(clipPartitionWithFinitePlane()));

	QObject::connect(
		&m_stacked_widget.m_model_partition_widget.m_confirm_paritition_selection, SIGNAL(clicked()),
		this, SLOT(confirmPartitionSelection()));

	QObject::connect(
		&m_stacked_widget.m_toolpath_generating_widget, SIGNAL(emitToolpathSetting(std::tuple<int, int, int, int, int, int>)),
		this, SLOT(generateToolPath(std::tuple<int, int, int, int, int, int>)));

	QObject::connect(
		&m_stacked_widget.m_preview_widget.m_slider_for_selecting_printing_layer, SIGNAL(valueChanged(int)),
		this, SLOT(changePreviewLayer(int)));

	QObject::connect(
		&m_stacked_widget.m_preview_widget.m_prev_page, SIGNAL(clicked()),
		this, SLOT(returnToToolpathGenerationWidget()));

	QObject::connect(
		&m_stacked_widget.m_preview_widget.m_generate_gcode_button, SIGNAL(clicked()),
		this, SLOT(generateGcode()));
}

PathPlanningDockWidget::~PathPlanningDockWidget()
{
	mp_default_interactor_style->Delete();
}

void PathPlanningDockWidget::reset()
{
	m_drawer.removeAllObjectsDrawn();
	m_drawer_for_preview.removeAllObjectsDrawn();
	m_partitions.clear();
	model_name = std::string();
	m_tree_widget.reset();
	mp_renderer->ResetCamera();
	mp_render_window->Render();
	mp_render_window_interactor->SetInteractorStyle(mp_default_interactor_style);
	m_stacked_widget.setCurrentIndex(0);
}

void PathPlanningDockWidget::openMesh()
{
	QString file_path = QFileDialog::getOpenFileName(this, tr("open STL file"), "../data", tr("STL files(*.stl)"));

	m_partitions.clear();
	m_partitions.addPartition(SO::Partition<CgalMesh_EPICK>(file_path.toStdString()));

	std::size_t found = file_path.toStdString().find_last_of("/\\");
	model_name = file_path.toStdString().substr(found + 1);

	this->updateTreeWidget();
	m_stacked_widget.setCurrentIndex(2);
}

void PathPlanningDockWidget::openDefaultMesh()
{
	QString file_path("../data/firebird_1mm_zero.stl");

	m_partitions.clear();
	m_partitions.addPartition(SO::Partition<CgalMesh_EPICK>(file_path.toStdString()));

	std::size_t found = file_path.toStdString().find_last_of("/\\");
	model_name = file_path.toStdString().substr(found + 1);

	this->updateTreeWidget();
	m_stacked_widget.setCurrentIndex(2);
}

void PathPlanningDockWidget::changeShowOrHidePartition(int partition_index, bool visible)
{
	std::string name = std::string("Partition") + std::to_string(partition_index);
	m_drawer.setVisible(name, visible);
	name = std::string("Contours") + std::to_string(partition_index);
	m_drawer.setVisible(name, visible);
	name = std::string("Support_Contours") + std::to_string(partition_index);
	m_drawer.setVisible(name, visible);
}

void PathPlanningDockWidget::changePartitionMode(int mode)
{
	switch (mode)
	{
	case 0:
		mp_render_window_interactor->SetInteractorStyle(mp_default_interactor_style);
	case 1:
		mp_render_window_interactor->SetInteractorStyle(mp_default_interactor_style);
	case 2:
		mp_render_window_interactor->SetInteractorStyle(&m_mesh_clipping_interactor_style);
		m_stacked_widget.m_model_partition_widget.m_infinite_plane_clipping_confirm_line_button.setEnabled(true);
	case 3:
		mp_render_window_interactor->SetInteractorStyle(&m_mesh_clipping_interactor_style);
		m_stacked_widget.m_model_partition_widget.m_finite_plane_clipping_confirm_line_button.setEnabled(true);
		m_stacked_widget.m_model_partition_widget.m_confirm_paritition_selection.setEnabled(false);
		m_stacked_widget.m_model_partition_widget.m_confirm_result_button.setEnabled(false);
		m_stacked_widget.m_model_partition_widget.m_reset_button.setEnabled(false);
	default:
		break;
	}
}

void PathPlanningDockWidget::setClipPartition(int partition_index)
{
	m_tree_widget.setButtonsEnabled(false);

	for (int i = 0; i < m_partitions.numberOfPartitions(); i++)
	{
		std::string name = std::string("Partition") + std::to_string(i);
		m_drawer.setVisible(name, false);
		name = std::string("Contours") + std::to_string(i);
		m_drawer.setVisible(name, false);
		name = std::string("Support_Contours") + std::to_string(i);
		m_drawer.setVisible(name, false);
		m_tree_widget.topLevelItem(0)->child(i)->setCheckState(0, Qt::Unchecked);
	}

	std::string name = std::string("Partition") + std::to_string(partition_index);
	m_drawer.setVisible(name, true);
	name = std::string("Contours") + std::to_string(partition_index);
	m_drawer.setVisible(name, true);
	name = std::string("Support_Contours") + std::to_string(partition_index);
	m_drawer.setVisible(name, true);
	m_tree_widget.topLevelItem(0)->child(partition_index)->setCheckState(0, Qt::Checked);

	mp_renderer->GetActiveCamera()->ParallelProjectionOn();
	mp_renderer->ResetCamera();
	mp_render_window->Render();

	mp_on_clipping_partition = &m_partitions[partition_index];
	m_operating_partition_index = partition_index;

	m_stacked_widget.setCurrentIndex(1);
	m_stacked_widget.m_model_partition_widget.m_partition_mode_combo_box.setCurrentIndex(0);
}

void PathPlanningDockWidget::clipPartition()
{
	std::tuple<SO::Line, Eigen::Vector3d, SO::Plane> clipping_parameter;
	m_mesh_clipper.setPartition(&m_partitions[m_operating_partition_index]);
	if (m_mesh_clipping_interactor_style.getLine(clipping_parameter))
	{
		m_mesh_clipper.clipWithInfinitePlane(std::get<2>(clipping_parameter), m_partitions_in_partitioning);

		std::string name = std::string("Partition") + std::to_string(m_operating_partition_index);
		m_drawer.setVisible(name, false);
		name = std::string("Contours") + std::to_string(m_operating_partition_index);
		m_drawer.setVisible(name, false);
		name = std::string("Support_Contours") + std::to_string(m_operating_partition_index);
		m_drawer.setVisible(name, false);

		this->drawPartitionsInPartitioning();

		m_mesh_clipping_interactor_style.deleteLine();
	}

	m_stacked_widget.m_model_partition_widget.m_partition_mode_combo_box.setEnabled(false);
	m_stacked_widget.m_model_partition_widget.m_infinite_plane_clipping_confirm_line_button.setEnabled(false);
	m_stacked_widget.m_model_partition_widget.m_confirm_result_button.setEnabled(true);
	m_stacked_widget.m_model_partition_widget.m_reset_button.setEnabled(true);
	m_stacked_widget.m_model_partition_widget.m_exit_button.setEnabled(false);
}

void PathPlanningDockWidget::clipPartitionWithFinitePlane()
{
	std::tuple<SO::Line, Eigen::Vector3d, SO::Plane> clipping_parameter;
	m_mesh_clipper.setPartition(&m_partitions[m_operating_partition_index]);
	if (m_mesh_clipping_interactor_style.getLine(clipping_parameter))
	{
		m_mesh_clipper.clipWithFinitePlane(std::get<0>(clipping_parameter),
			std::get<1>(clipping_parameter), std::get<2>(clipping_parameter), m_partitions_in_partitioning);

		std::string name = std::string("Partition") + std::to_string(m_operating_partition_index);
		m_drawer.setVisible(name, false);
		name = std::string("Contours") + std::to_string(m_operating_partition_index);
		m_drawer.setVisible(name, false);
		name = std::string("Support_Contours") + std::to_string(m_operating_partition_index);
		m_drawer.setVisible(name, false);

		m_mesh_clipping_interactor_style.deleteLine();
	}

	m_partition_selecting_interactor_style.setPartitions(m_partitions_in_partitioning, mp_on_clipping_partition->getBasePlane());
	mp_render_window_interactor->SetInteractorStyle(&m_partition_selecting_interactor_style);

	m_stacked_widget.m_model_partition_widget.m_partition_mode_combo_box.setEnabled(false);
	m_stacked_widget.m_model_partition_widget.m_finite_plane_clipping_confirm_line_button.setEnabled(false);
	m_stacked_widget.m_model_partition_widget.m_confirm_paritition_selection.setEnabled(true);
	m_stacked_widget.m_model_partition_widget.m_confirm_result_button.setEnabled(false);
	m_stacked_widget.m_model_partition_widget.m_reset_button.setEnabled(false);
	m_stacked_widget.m_model_partition_widget.m_exit_button.setEnabled(false);

	mp_render_window->Render();
}

void PathPlanningDockWidget::confirmPartitionSelection()
{
	m_partition_selecting_interactor_style.confirmSelection(m_partitions_in_partitioning);
	this->drawPartitionsInPartitioning();
	mp_render_window_interactor->SetInteractorStyle(mp_default_interactor_style);
	m_stacked_widget.m_model_partition_widget.m_confirm_paritition_selection.setEnabled(false);
	m_stacked_widget.m_model_partition_widget.m_confirm_result_button.setEnabled(true);
	m_stacked_widget.m_model_partition_widget.m_reset_button.setEnabled(true);
}

void PathPlanningDockWidget::runAutoPartition()
{	
	AutoPartitionerGUI auto_partitioner(*mp_on_clipping_partition, m_nozzle, mp_renderer);
	auto_partitioner.partition();
	m_partitions_in_partitioning = auto_partitioner.getResultEPICK();

	std::string name = std::string("Partition") + std::to_string(m_operating_partition_index);
	m_drawer.setVisible(name, false);
	name = std::string("Contours") + std::to_string(m_operating_partition_index);
	m_drawer.setVisible(name, false);
	name = std::string("Support_Contours") + std::to_string(m_operating_partition_index);
	m_drawer.setVisible(name, false);

	this->drawPartitionsInPartitioning();

	m_stacked_widget.m_model_partition_widget.m_auto_partition_button.setEnabled(false);
	m_stacked_widget.m_model_partition_widget.m_confirm_result_button.setEnabled(true);
	m_stacked_widget.m_model_partition_widget.m_reset_button.setEnabled(true);
	m_stacked_widget.m_model_partition_widget.m_exit_button.setEnabled(false);
}

void PathPlanningDockWidget::confirmPartition()
{
	m_drawer.removeAllObjectsDrawn();
	m_partitions.erase(m_operating_partition_index);

	for (int i = 0; i < m_partitions_in_partitioning.numberOfPartitions(); i++)
	{
		m_partitions.addPartition(m_partitions_in_partitioning[i]);
	}

	PrintingSequenceDeterminatorGUI printing_sequence_determinator(m_partitions, m_nozzle, mp_renderer);
	printing_sequence_determinator.determinePrintingSequence();

	this->updateTreeWidget();
	m_stacked_widget.setCurrentIndex(2);
	m_mesh_clipping_interactor_style.deleteLine();
	mp_render_window_interactor->SetInteractorStyle(mp_default_interactor_style);
}

void PathPlanningDockWidget::resetPartitionResult()
{
	for (int i = 0; i < m_partitions_in_partitioning.numberOfPartitions(); i++)
	{
		std::string name = std::string("Temp_Partition") + std::to_string(i);
		m_drawer.removeObjectDrawn(name);
		name = std::string("Temp_Contours") + std::to_string(i);
		m_drawer.removeObjectDrawn(name);
		name = std::string("Temp_Support_Contours") + std::to_string(i);
		m_drawer.removeObjectDrawn(name);
	}
	m_partitions_in_partitioning.clear();

	std::string name = std::string("Partition") + std::to_string(m_operating_partition_index);
	m_drawer.setVisible(name, true);
	name = std::string("Contours") + std::to_string(m_operating_partition_index);
	m_drawer.setVisible(name, true);
	name = std::string("Support_Contours") + std::to_string(m_operating_partition_index);
	m_drawer.setVisible(name, true);

	this->changePartitionMode(m_stacked_widget.m_model_partition_widget.m_partition_mode_combo_box.currentIndex());

	m_stacked_widget.m_model_partition_widget.m_partition_mode_combo_box.setEnabled(true);
	m_stacked_widget.m_model_partition_widget.m_auto_partition_button.setEnabled(true);
	m_stacked_widget.m_model_partition_widget.m_infinite_plane_clipping_confirm_line_button.setEnabled(true);
	m_stacked_widget.m_model_partition_widget.m_finite_plane_clipping_confirm_line_button.setEnabled(true);
	m_stacked_widget.m_model_partition_widget.m_confirm_result_button.setEnabled(false);
	m_stacked_widget.m_model_partition_widget.m_reset_button.setEnabled(false);
	m_stacked_widget.m_model_partition_widget.m_exit_button.setEnabled(true);
}

void PathPlanningDockWidget::exitPartition()
{
	m_tree_widget.setButtonsEnabled(true);
	m_stacked_widget.setCurrentIndex(2);
}

void PathPlanningDockWidget::generateToolPath(std::tuple<int, int, int, int, int, int> toolpath_setting)
{
	spdlog::info("Generate Support: start.");
	SupportGeneratorGUI support_generator(m_partitions, mp_renderer);
	support_generator.generateSupportStructure();
	spdlog::info("Generate Support: End.");

	spdlog::info("Generate ToolPath: start.");
	for (int i = 0; i < m_partitions.numberOfPartitions(); i++)
	{
		ToolpathGeneratorGUI toolpath_generator(m_partitions[i], mp_renderer, true);
		toolpath_generator.setPathPropertyForModel(std::get<0>(toolpath_setting), 3, 3, 
			std::get<1>(toolpath_setting), std::get<2>(toolpath_setting), 0.4);
		toolpath_generator.setPathPropertyForSupport(std::get<3>(toolpath_setting), 3, 3,
			std::get<4>(toolpath_setting), std::get<5>(toolpath_setting), 0.4);
		toolpath_generator.generatePath();
	}
	spdlog::info("Generate ToolPath: End.");
	this->updatePrintingPath();
	m_stacked_widget.setCurrentIndex(3);

	int slider_size = 0;
	for (int i = 0; i < m_partitions.numberOfPartitions(); i++)
	{
		std::string name = std::string("Partition") + std::to_string(i);
		m_drawer.setOpacity(name, 0.2);
		slider_size += m_partitions[i].getPrintingLayers().size();
	}

	m_tree_widget.setButtonsEnabled(false);
	m_stacked_widget.m_preview_widget.m_slider_for_selecting_printing_layer.setRange(0, slider_size - 1);
	m_stacked_widget.m_preview_widget.m_slider_for_selecting_printing_layer.setValue(0);
}

void PathPlanningDockWidget::changePreviewLayer(int value)
{
	if (m_drawer_for_preview.numberOfObjectsDrawn() > 1)
	{
		m_drawer_for_preview.removeAllObjectsDrawn();
	}

	int difference_0 = value;
	int difference_1 = value;
	int partition_index = 0;
	bool done = false;

	while (done == false)
	{
		difference_1 = difference_1 - m_partitions[partition_index].getPrintingLayers().size();
		if (difference_1 < 0)
		{
			m_stacked_widget.m_preview_widget.m_partition_ID.setValue(partition_index);
			m_stacked_widget.m_preview_widget.m_layer_ID.setValue(difference_0);
			done = true;
		}
		else
		{
			difference_0 = difference_1;
			++partition_index;
		}
	}

	SO::PrintingLayerCollection &printing_layers = m_partitions[partition_index].getPrintingLayers();
	SO::PrintingLayer &printing_layer = printing_layers[difference_0];

	std::string name = "S" + std::to_string(value);
	m_drawer_for_preview.drawPolygons(printing_layer.getPrintingPaths().getSurface(), name);
	m_drawer_for_preview.setColor(name, 1, 0, 0);

	for (size_t i = 0; i < printing_layer.getPrintingPaths().getWall().size(); i++)
	{
		name = "W" + std::to_string(i) + std::to_string(value);
		m_drawer_for_preview.drawPolygons(printing_layer.getPrintingPaths().getWall()[i], name);
		m_drawer_for_preview.setColor(name, 0, 1, 0);
	}

	name = "BT" + std::to_string(value);
	m_drawer_for_preview.drawPolygons(printing_layer.getPrintingPaths().getBottomTopUnion(), name);
	m_drawer_for_preview.setColor(name, 1, 1, 0);

	name = "I" + std::to_string(value);
	m_drawer_for_preview.drawPolygons(printing_layer.getPrintingPaths().getInfill(), name);
	m_drawer_for_preview.setColor(name, (double)255 / 255, (double)165 / 255, 0);

	name = "SS" + std::to_string(value);
	m_drawer_for_preview.drawPolygons(printing_layer.getPrintingPathsForSupport().getSurface(), name);
	m_drawer_for_preview.setColor(name, 1, 0, 0);

	for (size_t i = 0; i < printing_layer.getPrintingPathsForSupport().getWall().size(); i++)
	{
		name = "WW" + std::to_string(i) + std::to_string(value);
		m_drawer_for_preview.drawPolygons(printing_layer.getPrintingPathsForSupport().getWall()[i], name);
		m_drawer_for_preview.setColor(name, 0, 1, 0);
	}

	name = "BTBT" + std::to_string(value);
	m_drawer_for_preview.drawPolygons(printing_layer.getPrintingPathsForSupport().getBottomTopUnion(), name);
	m_drawer_for_preview.setColor(name, 1, 1, 0);

	name = "II" + std::to_string(value);
	m_drawer_for_preview.drawPolygons(printing_layer.getPrintingPathsForSupport().getInfill(), name);
	m_drawer_for_preview.setColor(name, (double)255 / 255, (double)165 / 255, 0);
}

void PathPlanningDockWidget::returnToToolpathGenerationWidget()
{
	m_drawer_for_preview.removeAllObjectsDrawn();
	m_stacked_widget.setCurrentIndex(2);
}

void PathPlanningDockWidget::generateGcode()
{
	QString file_path = QFileDialog::getSaveFileName(this, tr("Save G-code File"), "../", tr("G-code(*.txt)"));

	GcodeGeneratorGUI gcode_generator(m_partitions, mp_renderer);
	gcode_generator.generateToolpathForEachLayer();
	gcode_generator.generateCompletedToolpath();
	gcode_generator.writeGcode(file_path.toStdString());
	
	spdlog::info("Done");
}

void PathPlanningDockWidget::updateTreeWidget()
{
	m_drawer.removeAllObjectsDrawn();

	if (m_partitions.numberOfPartitions() == 1)
	{
		std::string name = std::string("Partition") + std::to_string(0);
		m_drawer.drawMesh(m_partitions[0].getEPICKMesh(), name);
		m_drawer.setColor(name, 0.7, 0.7, 0.7);

		SyNSlicerGUI::AutoSlicerGUI auto_slicer(m_partitions[0], mp_renderer);
		auto_slicer.slice();
		SO::PrintingLayerCollection &printing_layers = m_partitions[0].getPrintingLayers();
		printing_layers.update();
		name = std::string("Contours") + std::to_string(0);
		m_drawer.drawPolylines(printing_layers.getContours(), name);
		m_drawer.setColor(name, 1, 0, 0);
		name = std::string("Support_Contours") + std::to_string(0);
		m_drawer.drawPolylines(printing_layers.getSupportContours(), name);
		m_drawer.setColor(name, 1, 1, 0);
	}
	else
	{
		this->drawPartitions();
	}

	mp_renderer->GetActiveCamera()->ParallelProjectionOn();
	mp_renderer->ResetCamera();
	mp_render_window->Render();

	// Tree widget only has one top item.
	if (m_tree_widget.topLevelItemCount() > 0)
	{
		m_tree_widget.takeTopLevelItem(0);
	}

	m_tree_widget.addTopLevelItem(model_name);
	m_tree_widget.addChildItems(0, m_partitions.numberOfPartitions());
}

void PathPlanningDockWidget::drawPartitions()
{
	for (int i = 0; i < m_partitions.numberOfPartitions(); i++)
	{
		std::string name = std::string("Partition") + std::to_string(i);
		m_drawer.drawMesh(m_partitions[i].getEPICKMesh(), name);
		m_drawer.setColor(name,
			double(50 + 150 / m_partitions.numberOfPartitions() * i) / double(255),
			double(50 + 150 / m_partitions.numberOfPartitions() * i) / double(255),
			double(50 + 150 / m_partitions.numberOfPartitions() * i) / double(255));

		SyNSlicerGUI::AutoSlicerGUI auto_slicer(m_partitions[i], mp_renderer);
		auto_slicer.slice();
		SO::PrintingLayerCollection &printing_layers = m_partitions[i].getPrintingLayers();
		printing_layers.update();
		name = std::string("Contours") + std::to_string(i);
		m_drawer.drawPolylines(printing_layers.getContours(), name);
		m_drawer.setColor(name, 1, 0, 0);
		name = std::string("Support_Contours") + std::to_string(i);
		m_drawer.drawPolylines(printing_layers.getSupportContours(), name);
		m_drawer.setColor(name, 1, 1, 0);
	}
}

void PathPlanningDockWidget::drawPartitionsInPartitioning()
{
	for (int i = 0; i < m_partitions_in_partitioning.numberOfPartitions(); i++)
	{
		std::string name = std::string("Temp_Partition") + std::to_string(i);
		m_drawer.drawMesh(m_partitions_in_partitioning[i].getEPICKMesh(), name);
		m_drawer.setColor(name,
			double(50 + 150 / m_partitions_in_partitioning.numberOfPartitions() * i) / double(255),
			double(50 + 150 / m_partitions_in_partitioning.numberOfPartitions() * i) / double(255),
			double(50 + 150 / m_partitions_in_partitioning.numberOfPartitions() * i) / double(255));

		SyNSlicerGUI::AutoSlicerGUI auto_slicer(m_partitions_in_partitioning[i], mp_renderer);
		auto_slicer.slice();
		SO::PrintingLayerCollection &printing_layers = m_partitions_in_partitioning[i].getPrintingLayers();
		printing_layers.update();
		name = std::string("Temp_Contours") + std::to_string(i);
		m_drawer.drawPolylines(printing_layers.getContours(), name);
		m_drawer.setColor(name, 1, 0, 0);
		name = std::string("Temp_Support_Contours") + std::to_string(i);
		m_drawer.drawPolylines(printing_layers.getSupportContours(), name);
		m_drawer.setColor(name, 1, 1, 0);
	}
}

void PathPlanningDockWidget::updatePrintingPath()
{
	for (int i = 0; i < m_partitions.numberOfPartitions(); i++)
	{
		SO::PrintingLayerCollection &printing_layers = m_partitions[i].getPrintingLayers();
		printing_layers.update();
		std::string name = std::string("Contours") + std::to_string(i);
		m_drawer.removeObjectDrawn(name);
		//m_drawer.drawPolylines(printing_layers.getContours(), name);
		//m_drawer.setColor(name, 1, 0, 0);
		name = std::string("Support_Contours") + std::to_string(i);
		m_drawer.removeObjectDrawn(name);
		//m_drawer.drawPolylines(printing_layers.getSupportContours(), name);
		//m_drawer.setColor(name, 1, 1, 0);
	}
}
