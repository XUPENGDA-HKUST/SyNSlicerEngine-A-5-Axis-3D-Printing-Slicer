#ifndef SYNSLICERENGINE_GUI_PRINTINGSEQUENCEDETERMINATOR_H_
#define SYNSLICERENGINE_GUI_PRINTINGSEQUENCEDETERMINATOR_H_
 
#include "SyNSlicerEngine/Algorithm/printing_sequence_determinator.h"
#include "SyNSlicerGUI/Algorithm/swept_volume_calculater_gui.h"
#include "object_drawer.h"

namespace SO = SyNSlicerEngine::Object;
namespace SA = SyNSlicerEngine::Algorithm;

namespace SyNSlicerGUI
{
	//!  This class is used to determine the printing sequence of all the printing layers
	class PrintingSequenceDeterminatorGUI : public SA::PrintingSequenceDeterminator
	{
	public:
		//!  Default constructor is not allowed.
		PrintingSequenceDeterminatorGUI() = delete;

		//!  Constructor.
		/*!
			\param[in,out] partition_list All the partitions.
			\param[in] nozzle The printer nozzle.
		*/
		PrintingSequenceDeterminatorGUI(SO::PartitionCollection<CgalMesh_EPICK> &partition_list, SO::Nozzle nozzle, vtkRenderer *p_renderer);

		//!  Constructor.
		/*!
			\param[in,out] partition_list All the partitions.
			\param[in] nozzle The printer nozzle.
		*/
		PrintingSequenceDeterminatorGUI(SO::PartitionCollection<CgalMesh_EPICK> &partition_list, SO::Nozzle nozzle, SyNSlicerGUI::ObjectDrawer *p_drawer);

		//!  Destructor.
		~PrintingSequenceDeterminatorGUI();

	protected:

		virtual void findSweptVolumeOfNozzleForAllPartition() override;

		bool m_should_drawer_delete_in_destructer;

		SyNSlicerGUI::ObjectDrawer *m_drawer;
	};
}

#endif  // SYNSLICERENGINE_GUI_PRINTINGSEQUENCEDETERMINATOR_H_