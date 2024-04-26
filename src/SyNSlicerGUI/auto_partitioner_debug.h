#ifndef SYNSLICERENGINE_GUI_AUTOPARTITIONERDEBUG_H_
#define SYNSLICERENGINE_GUI_AUTOPARTITIONERDEBUG_H_

#include "Algorithm/auto_partitioner.h"

#include "object_drawer.h"

namespace SO = SyNSlicerEngine::Object;
namespace SA = SyNSlicerEngine::Algorithm;

namespace SyNSlicerEngine::GUI
{
	//! AutoPartitionerDebug is used to debug AutoPartitioner
	/*!
		For those methods have problem, copy them to this class and then 
		override it. You can use ObjectDrawer to draw the geometry you 
		want to see.
	*/
	class AutoPartitionerDebug : public SA::AutoPartitioner
	{
	public:
		AutoPartitionerDebug() = delete;
		AutoPartitionerDebug(const SO::Partition<CgalMesh_EPICK> &partition, const SO::Nozzle &nozzle, vtkRenderer *renderer, double overhanging_angle = 56, double area_threshold_coefficient = 1.0);
		~AutoPartitionerDebug();

	protected:
		void partitionMesh(SO::Partition<CgalMesh_EPECK> &partition, SO::PartitionCollection<CgalMesh_EPECK> &partition_list, std::vector<SO::PointCloud> &vertices_to_ignore_list) override;
		AutoPartitioner::ResultOfDetermineClippingPlane determineClippingPlane(SO::Partition<CgalMesh_EPICK> &partition, SO::Plane &clipping_plane, std::vector<SO::PointCloud> &vertices_to_ignore_list) override;
		ObjectDrawer m_drawer;
	};
}

#endif //SYNSLICERENGINE_GUI_AUTOPARTITIONERDEBUG_H_