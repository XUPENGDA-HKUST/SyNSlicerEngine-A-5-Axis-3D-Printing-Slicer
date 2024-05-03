#include "slicer_gui.h"

using SyNSlicerGUI::SlicerGUI;

SlicerGUI::SlicerGUI(SO::Partition<CgalMesh_EPICK> &p_partition, vtkRenderer *p_renderer)
	: Slicer(p_partition)
	, m_drawer(new SyNSlicerGUI::ObjectDrawer(p_renderer))
	, m_should_drawer_delete_in_destructer(true)
{

}

SlicerGUI::SlicerGUI(SO::Partition<CgalMesh_EPICK> &p_partition, SyNSlicerGUI::ObjectDrawer *p_drawer)
	: Slicer(p_partition)
	, m_drawer(p_drawer)
	, m_should_drawer_delete_in_destructer(false)
{

}

SlicerGUI::~SlicerGUI()
{
	if (m_should_drawer_delete_in_destructer == true)
	{
		delete m_drawer;
	}
}
