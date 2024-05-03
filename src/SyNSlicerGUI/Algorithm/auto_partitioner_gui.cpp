#include "auto_partitioner_gui.h"

using SyNSlicerGUI::AutoPartitionerGUI;

AutoPartitionerGUI::AutoPartitionerGUI(const SO::Partition<CgalMesh_EPICK> &partition, const SO::Nozzle &nozzle, vtkRenderer *p_renderer, double overhanging_angle, double area_threshold_coefficient)
	: AutoPartitioner(partition, nozzle, overhanging_angle, area_threshold_coefficient)
	, m_drawer(new SyNSlicerGUI::ObjectDrawer(p_renderer))
	, m_should_drawer_delete_in_destructer(true)
{

}

AutoPartitionerGUI::AutoPartitionerGUI(const SO::Partition<CgalMesh_EPICK> &partition, const SO::Nozzle &nozzle, SyNSlicerGUI::ObjectDrawer *p_drawer, double overhanging_angle, double area_threshold_coefficient)
	: AutoPartitioner(partition, nozzle, overhanging_angle, area_threshold_coefficient)
	, m_drawer(p_drawer)
	, m_should_drawer_delete_in_destructer(false)
{

}

AutoPartitionerGUI::~AutoPartitionerGUI()
{
	if (m_should_drawer_delete_in_destructer == true)
	{
		delete m_drawer;
	}
}