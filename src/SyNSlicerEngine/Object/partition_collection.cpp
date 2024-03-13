#include "partition_collection.h"

using SyNSlicerEngine::Object::PartitionCollection;

template <class T>
PartitionCollection<T>::PartitionCollection()
{
}

template <class T>
PartitionCollection<T>::PartitionCollection(const PartitionCollection &other)
{
	*this = other;
}

template <class T>
PartitionCollection<T>::~PartitionCollection()
{
}

template <class T>
void PartitionCollection<T>::reset()
{
	m_partitions.clear();
}

template <class T>
void PartitionCollection<T>::addPartition(const Partition<T> &partition)
{
	m_partitions.emplace_back(partition);
}

template <class T>
SO::Partition<T> PartitionCollection<T>::back()
{
	return m_partitions.back();
}

template <class T>
void PartitionCollection<T>::pop_back()
{
	m_partitions.pop_back();
}

template <class T>
int PartitionCollection<T>::numberOfPartitions() const
{
	return m_partitions.size();
}

template <class T>
void PartitionCollection<T>::determinePrintingSequence()
{
}

template <class T>
PartitionCollection<T> &PartitionCollection<T>::operator=(const PartitionCollection &other)
{
	m_partitions = other.m_partitions;
	return *this;
}

template <class T>
SO::Partition<T> &PartitionCollection<T>::operator[](int index)
{
	return m_partitions[index];
}
