#ifndef SYNSLICERENGINE_ALGORITHM_PRINTINGSEQUENCEDETERMINATOR_H_
#define SYNSLICERENGINE_ALGORITHM_PRINTINGSEQUENCEDETERMINATOR_H_

#include <vector>
#include <iostream>
#include <utility>

#include <CGAL/Polygon_mesh_processing/corefinement.h>

#include "Object/partition_collection.h"
#include <Object/printing_layer_collection.h>
#include "swept_volume_calculater.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Algorithm
{
	//!  This class is used to determine the printing sequence of all the printing layers
	/*!
		Input:	Partition list. \n \n
		Output:	(PrintingLayerCollection). \n \n
		Description: None. \n
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

		PrintingSequenceDeterminator(SO::PartitionCollection<CgalMesh_EPICK> &partition_list);
		~PrintingSequenceDeterminator();

		PrintingSequenceStatus determinePrintingSequence();

	private:
		void findSweptVolumeOfNozzleForAllPartition();
		bool isSweptVolumeIntersectBuildPlate();
		bool isSweptVolumeIntersectParition(CgalMesh_EPICK swept_volume, CgalMesh_EPICK partition);
		bool isPrintingSequenceSelfSupported(SO::PartitionCollection<CgalMesh_EPICK> &printing_sequence);
		PrintingSequenceStatus isPrintingSequenceCollisionFree(SO::PartitionCollection<CgalMesh_EPICK> &printing_sequence, std::pair<int, int> &collided_pair);

		std::vector<CgalMesh_EPICK> m_swept_volume_list;
		SO::PartitionCollection<CgalMesh_EPICK> &m_printing_sequence; // From bottom to top
		std::vector<std::pair<int, int>> collided_pairs;
	};
}

#endif  // SYNSLICERENGINE_ALGORITHM_PRINTINGSEQUENCEDETERMINATOR_H_