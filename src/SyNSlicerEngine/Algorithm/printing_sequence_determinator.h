#ifndef SYNSLICERENGINE_ALGORITHM_PRINTINGSEQUENCEDETERMINATOR_H_
#define SYNSLICERENGINE_ALGORITHM_PRINTINGSEQUENCEDETERMINATOR_H_

#include <vector>
#include <iostream>
#include <utility>

#include <CGAL/Polygon_mesh_processing/corefinement.h>

#include "Object/nozzle.h"
#include "Object/partition_collection.h"
#include "Object/printing_layer_collection.h"
#include "swept_volume_calculater.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Algorithm
{
	//!  This class is used to determine the printing sequence of all the printing layers
	/*!

	*/
	class PrintingSequenceDeterminator
	{
	public:
		enum PrintingSequenceStatus
		{
			CollisionOccurAndCannotBeFixed = 0,
			CollisionOccurAndCanBeFixed = 1,
			CollisionFree = 2,
			CollisionFreeButNotSelfSupportive = 3,
			CollisionFreeAndSelfSupportive = 4
		};

		//!  Default constructor is not allowed.
		PrintingSequenceDeterminator() = delete;

		//!  Constructor.
		/*!
			\param[in,out] partition_list All the partitions.
			\param[in] nozzle The printer nozzle.
		*/
		PrintingSequenceDeterminator(SO::PartitionCollection<CgalMesh_EPICK> &partition_list, SO::Nozzle nozzle);

		//!  Destructor.
		~PrintingSequenceDeterminator();

		//! Call to start determine printing sequence.
		virtual PrintingSequenceStatus determinePrintingSequence();

	protected:
		//! Find Swept volume of nozzle.
		virtual void findSweptVolumeOfNozzleForAllPartition();

		//! Check if swept volume intersect the build plate.
		/*!
			\return \b Ture \b if intersected. \b False \b if not intersected.
		*/
		virtual bool isSweptVolumeIntersectBuildPlate();

		//! Check if the printing process is collision free.
		/*!
			\param[in] printing_sequence Current printing sequence.
			\param[out] collided_pair Collided partitions.
			\return PrintingSequenceStatus
		*/
		virtual PrintingSequenceStatus isPrintingSequenceCollisionFree(SO::PartitionCollection<CgalMesh_EPICK> &printing_sequence, std::pair<int, int> &collided_pair);

		//! Check if swept volume intersect the partition printed.
		/*!
			\param swept_volume[in] Swept volume.
			\param partition[in] Printed partition.
			\return \b Ture if intersected. \b False if not intersected.
		*/
		virtual bool isSweptVolumeIntersectParition(CgalMesh_EPICK swept_volume, CgalMesh_EPICK partition);

		//! Check if the printing process is self supportive.
		/*!
			\param[in] printing_sequence Current printing sequence.
			\return \b True if self-supportive. \b False if not self-supportive.
		*/
		virtual bool isPrintingSequenceSelfSupported(SO::PartitionCollection<CgalMesh_EPICK> &printing_sequence);

		//! The contatiner to store all swept volume.
		std::vector<CgalMesh_EPICK> m_swept_volume_list;

		//! Printing sequence, from bottom to top
		SO::PartitionCollection<CgalMesh_EPICK> &m_printing_sequence;

		//! Nozzle of the printer
		SO::Nozzle m_nozzle;
	};
}

#endif  // SYNSLICERENGINE_ALGORITHM_PRINTINGSEQUENCEDETERMINATOR_H_