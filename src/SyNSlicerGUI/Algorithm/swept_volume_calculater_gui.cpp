#include "swept_volume_calculater_gui.h"

using SyNSlicerGUI::SweptVolumwCalculatorGUI;

SweptVolumwCalculatorGUI::SweptVolumwCalculatorGUI(const SO::Partition<CgalMesh_EPICK> &partition, SO::Nozzle nozzle, vtkRenderer *p_renderer)
	: SweptVolumwCalculator(partition, nozzle)
	, m_drawer(new SyNSlicerGUI::ObjectDrawer(p_renderer))
	, m_should_drawer_delete_in_destructer(true)
{

}

SweptVolumwCalculatorGUI::SweptVolumwCalculatorGUI(const SO::Partition<CgalMesh_EPICK> &partition, SO::Nozzle nozzle, SyNSlicerGUI::ObjectDrawer *p_drawer)
	: SweptVolumwCalculator(partition, nozzle)
	, m_drawer(p_drawer)
	, m_should_drawer_delete_in_destructer(false)
{

}

SweptVolumwCalculatorGUI::~SweptVolumwCalculatorGUI()
{
	if (m_should_drawer_delete_in_destructer == true)
	{
		delete m_drawer;
	}
}
