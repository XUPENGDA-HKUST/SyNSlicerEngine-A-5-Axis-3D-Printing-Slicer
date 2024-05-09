#ifndef SYNSLICERENGINE_ALGORITHM_MESHCLIPPER_H_
#define SYNSLICERENGINE_ALGORITHM_MESHCLIPPER_H_

#include <vector>

#include <clipper2/clipper.h>

#include <CGAL_CORE_CLASS>

#include "Object/polygon.h"
#include "Object/partition.h"
#include "Object/partition_collection.h"
#include "Object/printing_layer_collection.h"
#include "Object/plane.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Algorithm
{
	//! This class is used to slice a 3D model with already calculated planes.
	/*!
	
	*/
	class MeshClipper
	{
	public:
		//! Default constructor.
		MeshClipper();

		//! Destructor.
		~MeshClipper();

		void setPartition(SO::Partition<CgalMesh_EPICK> *p_partition);

		//! Call to perform slicing.
		virtual bool clipWithInfinitePlane(
			const SO::Plane &plane, 
			SO::PartitionCollection<CgalMesh_EPICK> &result);

		virtual bool clipWithFinitePlane(
			const SO::Line &line,
			const Eigen::Vector3d &camera_position,
			const SO::Plane &clipping_plane,
			SO::PartitionCollection<CgalMesh_EPICK> &result);

	protected:
		int clipping_time;
		SO::Partition<CgalMesh_EPICK> *mp_partition;
	};
}

#endif  // SYNSLICERENGINE_ALGORITHM_MESHCLIPPER_H_