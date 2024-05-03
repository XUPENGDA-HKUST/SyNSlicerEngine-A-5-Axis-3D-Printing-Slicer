#include "support_generator_gui.h"

using SyNSlicerGUI::SupportGeneratorGUI;

SupportGeneratorGUI::SupportGeneratorGUI(SO::PartitionCollection<CgalMesh_EPICK> &input_paritions, vtkRenderer *p_renderer)
	: SupportGenerator(input_paritions)
	, m_drawer(new SyNSlicerGUI::ObjectDrawer(p_renderer))
	, m_should_drawer_delete_in_destructer(true)
{

}

SupportGeneratorGUI::SupportGeneratorGUI(SO::PartitionCollection<CgalMesh_EPICK> &input_paritions, SyNSlicerGUI::ObjectDrawer *p_drawer)
	: SupportGenerator(input_paritions)
	, m_drawer(p_drawer)
	, m_should_drawer_delete_in_destructer(false)
{

}

SupportGeneratorGUI::~SupportGeneratorGUI()
{
	if (m_should_drawer_delete_in_destructer == true)
	{
		delete m_drawer;
	}
}

