#ifndef SYNSLICERENGINE_GUI_PRINTINGSEQUENCEDETERMINATORDEBUG_H_
#define SYNSLICERENGINE_GUI_PRINTINGSEQUENCEDETERMINATORDEBUG_H_

#include "Algorithm/printing_sequence_determinator.h"
#include "object_drawer.h"

namespace SO = SyNSlicerEngine::Object;
namespace SA = SyNSlicerEngine::Algorithm;

namespace SyNSlicerEngine::GUI
{
	//!  This class is used to determine the printing sequence of all the printing layers
	/*!
		Input:	Partition list. \n \n
		Output:	(PrintingLayerCollection). \n \n
		Description: None. \n
	*/
	class PrintingSequenceDeterminatorDebug : public SA::PrintingSequenceDeterminator
	{
	public:
		PrintingSequenceDeterminatorDebug(SO::PartitionCollection<CgalMesh_EPICK> &partition_list, SO::Nozzle nozzle, vtkRenderer *renderer);
		~PrintingSequenceDeterminatorDebug();

	protected:
		virtual bool isSweptVolumeIntersectBuildPlate() override;
		ObjectDrawer m_drawer;
	};
}

#endif  // SYNSLICERENGINE_GUI_PRINTINGSEQUENCEDETERMINATORDEBUG_H_