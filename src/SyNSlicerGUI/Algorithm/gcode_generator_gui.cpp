#include "gcode_generator_gui.h"

using SyNSlicerGUI::GcodeGeneratorGUI;

GcodeGeneratorGUI::GcodeGeneratorGUI(SO::PartitionCollection<CgalMesh_EPICK> partitions, vtkRenderer *p_renderer, double side_step)
	: GcodeGenerator(partitions, side_step)
	, m_drawer(new SyNSlicerGUI::ObjectDrawer(p_renderer))
	, m_should_drawer_delete_in_destructer(true)
{

}

GcodeGeneratorGUI::GcodeGeneratorGUI(SO::PartitionCollection<CgalMesh_EPICK> partitions, SyNSlicerGUI::ObjectDrawer *p_drawer, double side_step)
	: GcodeGenerator(partitions, side_step)
	, m_drawer(p_drawer)
	, m_should_drawer_delete_in_destructer(false)
{

}

GcodeGeneratorGUI::~GcodeGeneratorGUI()
{
	if (m_should_drawer_delete_in_destructer == true)
	{
		delete m_drawer;
	}
}

