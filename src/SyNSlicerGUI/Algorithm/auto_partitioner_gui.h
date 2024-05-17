#ifndef SYNSLICERENGINE_GUI_AUTOPARTITIONER_H_
#define SYNSLICERENGINE_GUI_AUTOPARTITIONER_H_

#include "SyNSlicerEngine/Algorithm/auto_partitioner.h"
#include "object_drawer.h"

namespace SO = SyNSlicerEngine::Object;
namespace SA = SyNSlicerEngine::Algorithm;

//!  The frontend namespace
namespace SyNSlicerGUI
{
	//!  This class is used to partition a 3D model automatically.
	class AutoPartitionerGUI : public SA::AutoPartitioner
	{
	public:
		//! Defualt constructer is not allowed.
		AutoPartitionerGUI() = delete;

		AutoPartitionerGUI(
			const SO::Partition<CgalMesh_EPICK> &partition, 
			const SO::Nozzle &nozzle, 
			vtkRenderer *p_renderer,
			double overhanging_angle = 56, 
			double area_threshold_coefficient = 1.0);

		AutoPartitionerGUI(
			const SO::Partition<CgalMesh_EPICK> &partition,
			const SO::Nozzle &nozzle,
			SyNSlicerGUI::ObjectDrawer *p_drawer,
			double overhanging_angle = 56,
			double area_threshold_coefficient = 1.0);

		//! Destructer
		~AutoPartitionerGUI();

	protected:
		bool m_should_drawer_delete_in_destructer;

		SyNSlicerGUI::ObjectDrawer *m_drawer;
	};
}

#endif //SYNSLICERENGINE_GUI_AUTOPARTITIONER_H_