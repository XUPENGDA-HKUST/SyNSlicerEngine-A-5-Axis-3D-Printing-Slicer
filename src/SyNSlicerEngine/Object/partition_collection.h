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
		SO::Partition<T> &back();
		void pop_back();
		std::vector<SO::Partition<T>> &get();

		int numberOfPartitions() const;
		void determinePrintingSequence();

		PartitionCollection &operator=(const PartitionCollection &other);
		SO::Partition<T> &operator[](int index);

	private:
		std::vector<SO::Partition<T>> m_partitions;

	};

	template <class T>
	inline PartitionCollection<T>::PartitionCollection()
	{
	}

	template <class T>
	inline PartitionCollection<T>::PartitionCollection(const PartitionCollection &other)
	{
		*this = other;
	}

	template <class T>
	inline PartitionCollection<T>::~PartitionCollection()
	{
	}

	template <class T>
	inline void PartitionCollection<T>::reset()
	{
		m_partitions.clear();
	}

	template <class T>
	inline void PartitionCollection<T>::addPartition(const Partition<T> &partition)
	{
		m_partitions.emplace_back(partition);
	}

	template <class T>
	inline SO::Partition<T> &PartitionCollection<T>::back()
	{
		return m_partitions.back();
	}

	template <class T>
	inline void PartitionCollection<T>::pop_back()
	{
		m_partitions.pop_back();
	}
	template <class T>
	inline std::vector<SO::Partition<T>> &PartitionCollection<T>::get()
	{
		return m_partitions;
	}

	template <class T>
	inline int PartitionCollection<T>::numberOfPartitions() const
	{
		return m_partitions.size();
	}

	template <class T>
	inline void PartitionCollection<T>::determinePrintingSequence()
	{
	}

	template <class T>
	inline PartitionCollection<T> &PartitionCollection<T>::operator=(const PartitionCollection &other)
	{
		m_partitions = other.m_partitions;
		return *this;
	}

	template <class T>
	inline SO::Partition<T> &PartitionCollection<T>::operator[](int index)
	{
		return m_partitions[index];
	}

}

#endif  // SYNSLICERENGINE_OBJECT_PARTITIONCOLLECTION_H_