#include "printing_sequence_determinator_gui.h"

using SyNSlicerGUI::PrintingSequenceDeterminatorGUI;

PrintingSequenceDeterminatorGUI::PrintingSequenceDeterminatorGUI(SO::PartitionCollection<CgalMesh_EPICK> &partition_list, SO::Nozzle nozzle, vtkRenderer *p_renderer)
	: PrintingSequenceDeterminator(partition_list, nozzle)
	, m_drawer(new SyNSlicerGUI::ObjectDrawer(p_renderer))
	, m_should_drawer_delete_in_destructer(true)
{

}

PrintingSequenceDeterminatorGUI::PrintingSequenceDeterminatorGUI(SO::PartitionCollection<CgalMesh_EPICK> &partition_list, SO::Nozzle nozzle, SyNSlicerGUI::ObjectDrawer *p_drawer)
	: PrintingSequenceDeterminator(partition_list, nozzle)
	, m_drawer(p_drawer)
	, m_should_drawer_delete_in_destructer(false)
{

}

PrintingSequenceDeterminatorGUI::~PrintingSequenceDeterminatorGUI()
{
	if (m_should_drawer_delete_in_destructer == true)
	{
		delete m_drawer;
	}
}

void PrintingSequenceDeterminatorGUI::findSweptVolumeOfNozzleForAllPartition()
{
	for (size_t i = 0; i < m_printing_sequence.numberOfPartitions(); i++)
	{
		if (m_should_drawer_delete_in_destructer == true)
		{
			SweptVolumwCalculatorGUI swept_volume_calculator(m_printing_sequence[i], m_nozzle, m_drawer->getRenderer());
			swept_volume_calculator.calculateSweptVolume();
			m_swept_volume_list.push_back(swept_volume_calculator.getSweptVolume()[0]);
		}
		else
		{
			SweptVolumwCalculatorGUI swept_volume_calculator(m_printing_sequence[i], m_nozzle, m_drawer);
			swept_volume_calculator.calculateSweptVolume();
			m_swept_volume_list.push_back(swept_volume_calculator.getSweptVolume()[0]);
		}
	}
}
