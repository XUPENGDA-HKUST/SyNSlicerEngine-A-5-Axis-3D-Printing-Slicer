#include "auto_slicer_gui.h"

using SyNSlicerGUI::AutoSlicerGUI;

AutoSlicerGUI::AutoSlicerGUI(SO::Partition<CgalMesh_EPICK> &p_partition, vtkRenderer *p_renderer, double target_layer_thickness, double side_step, double min_layer_thickness, double max_layer_thickness)
	: AutoSlicer(p_partition, target_layer_thickness, side_step, min_layer_thickness, max_layer_thickness)
	, m_drawer(new SyNSlicerGUI::ObjectDrawer(p_renderer))
	, m_should_drawer_delete_in_destructer(true)
{

}

AutoSlicerGUI::AutoSlicerGUI(SO::Partition<CgalMesh_EPICK> &p_partition, SyNSlicerGUI::ObjectDrawer *p_drawer, double target_layer_thickness, double side_step, double min_layer_thickness, double max_layer_thickness)
	: AutoSlicer(p_partition, target_layer_thickness, side_step, min_layer_thickness, max_layer_thickness)
	, m_drawer(p_drawer)
	, m_should_drawer_delete_in_destructer(false)
{

}

AutoSlicerGUI::~AutoSlicerGUI()
{
	if (m_should_drawer_delete_in_destructer == true)
	{
		delete m_drawer;
	}
}
