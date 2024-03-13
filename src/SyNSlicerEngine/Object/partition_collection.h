#ifndef SYNSLICERENGINE_OBJECT_PARTITIONCOLLECTION_H_
#define SYNSLICERENGINE_OBJECT_PARTITIONCOLLECTION_H_

#include "Object/partition.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object
{
	//! This class is used describe the 3D model after partitioning.
	/*!
		The difference between this and the superclass Model is that it stores members. \n
		(1) A contour obtain during parititioning used to calculate printing sequence. \n
		(2) Partition that should be printed before this partition does. \n
		(3) All printing layers obtained after slicing this partition.
	*/
	template <class T>
	class PartitionCollection
	{
	public:
		PartitionCollection();
		PartitionCollection(const PartitionCollection &other);
		~PartitionCollection();

		void reset();
		void addPartition(const SO::Partition<T> &partition);
		SO::Partition<T> back();
		void pop_back();

		int numberOfPartitions() const;
		void determinePrintingSequence();

		PartitionCollection &operator=(const PartitionCollection &other);
		SO::Partition<T> &operator[](int index);

	private:
		std::vector<SO::Partition<T>> m_partitions;

	};
}

#endif  // SYNSLICERENGINE_OBJECT_PARTITIONCOLLECTION_H_