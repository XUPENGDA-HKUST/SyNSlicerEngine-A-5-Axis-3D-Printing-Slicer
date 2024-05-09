#ifndef SYNSLICERENGINE_OBJECT_PARTITIONCOLLECTION_H_
#define SYNSLICERENGINE_OBJECT_PARTITIONCOLLECTION_H_

#include "Object/partition.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object
{
	//! This class is store all the 3D models obtained in partitioning.
	template <class T>
	class PartitionCollection
	{
	public:
		//! Default constructor.
		PartitionCollection();

		//! Copy constructor.
		PartitionCollection(const PartitionCollection &other);

		//! Constructor.
		/*!
			\brief	Construct by loading the save file of this class.
			\param[in]	file_name	Path of the file.
		*/
		PartitionCollection(std::string file_name);

		//! Destructor.
		~PartitionCollection();

		//! Save the Partitions.
		/*!
			\param[in]	file_name	Path of the save file.
		*/
		bool save(std::string file_name);

		//! Load the save file of this class.
		/*!
			\param[in]	file_name	Path of the save file.
		*/
		bool load(std::string file_name);

		//! Restore all the members of this class to default value.
		void reset();
		void clear();

		//! Add a partition.
		/*!
			\param[in]	partition	Partition.
		*/
		void addPartition(const SO::Partition<T> &partition);

		//! Revert the sequence of the partitions.
		void revert();

		//! Get the last partition stored in this class.
		/*!
			\return SO::Partition<T>.
		*/
		SO::Partition<T> &back();

		//! Remove the last partition stored in this class.
		void pop_back();

		void erase(int index);

		//! Get all the partitions stored in this class.
		/*!
			\return std::vector<SO::Partition<T>>.
		*/
		std::vector<SO::Partition<T>> &get();

		//! Get number of the partitions stored in this class.
		/*!
			\return Number of partitions.
		*/
		int numberOfPartitions() const;

		//! Copy assignment operators
		PartitionCollection &operator=(const PartitionCollection &other);

		//! Get one of the partition stored in this class.
		/*!
			\param[in]	index	Index.
		*/
		SO::Partition<T> &operator[](int index);

	protected:
		//! Store all the partitions.
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

	template<class T>
	inline PartitionCollection<T>::PartitionCollection(std::string file_name)
	{
		this->load(file_name);
	}

	template <class T>
	inline PartitionCollection<T>::~PartitionCollection()
	{
	}

	template<class T>
	inline bool PartitionCollection<T>::save(std::string file_name)
	{
		std::ofstream myfile(file_name);

		if (myfile.is_open()) {
			// Write some lines to the file

			for (int i = 0; i < m_partitions.size(); i++)
			{
				// Save the model.
				std::string name("Partition_" + std::to_string(i) + ".stl");
				m_partitions[i].writeMeshToSTL(name);
				myfile << name << "\n";

				// Save the base plane.
				Plane plane = m_partitions[i].getBasePlane();
				myfile << "Base" << " " << plane.getOrigin()[0] << " " << plane.getOrigin()[1] << " " << plane.getOrigin()[2] << " ";
				myfile << plane.getNormal()[0] << " " << plane.getNormal()[1] << " " << plane.getNormal()[2] << "\n";

				// Save the slicing planes.
				SO::PrintingLayerCollection &printing_layers = m_partitions[i].getPrintingLayers();
				for (int j = 0; j < printing_layers.size(); j++)
				{
					Plane plane = printing_layers[j].getSlicingPlane();
					myfile << "Slice" << " " << plane.getOrigin()[0] << " " << plane.getOrigin()[1] << " " << plane.getOrigin()[2] << " ";
					myfile << plane.getNormal()[0] << " " << plane.getNormal()[1] << " " << plane.getNormal()[2] << "\n";
				}
				myfile << "End" << "\n";
			}

			// Close the file
			myfile.close();
			return true;
		}
		else 
		{
			return false;
		}
	}

	template<class T>
	inline bool PartitionCollection<T>::load(std::string file_name)
	{
		std::ifstream inputFile(file_name); // Replace with your file path
		int status = 0; // 0: Find ".stl'. 1: Find "Base". 2: Find "Slice".

		if (inputFile.is_open()) {
			std::string line;
			SO::PrintingLayerCollection printing_layers;
			SO::Plane prev_slicing_plane;
			while (std::getline(inputFile, line)) 
			{
				if (status == 0)
				{
					std::string::size_type n;
					n = line.find(".stl");
					if (n != std::string::npos)
					{
						SO::Partition<CgalMesh_EPICK> partition(line);
						m_partitions.emplace_back(partition);
						status = 1;
					}
					else
					{
						if (this->m_partitions.size() > 0) { return true; }
						else { return false; }
					}
				}
				else if (status == 1)
				{
					std::string::size_type n;
					n = line.find("Base");
					if (n != std::string::npos)
					{
						char ss[10];
						double a, b, c, d, e, f;
						std::sscanf(line.c_str(), "%s %lf %lf %lf %lf %lf %lf", &ss, &a, &b, &c, &d, &e, &f);
						SO::Plane base_plane(Eigen::Vector3d(a, b, c), Eigen::Vector3d(d, e, f));
						m_partitions.back().setBasePlane(base_plane);
						prev_slicing_plane = base_plane;
						status = 2;
						printing_layers = m_partitions.back().getPrintingLayers();
					}
					else
					{
						spdlog::error("Saved file not valid. Load fails.");
					}
				}
				else if (status == 2)
				{
					std::string::size_type n;
					n = line.find("Slice");
					if (n != std::string::npos)
					{
						char ss[10];
						double a, b, c, d, e, f;
						std::sscanf(line.c_str(), "%s %lf %lf %lf %lf %lf %lf", &ss, &a, &b, &c, &d, &e, &f);
						SO::Plane slicing_plane(Eigen::Vector3d(a, b, c), Eigen::Vector3d(d, e, f));

						SO::PrintingLayer printing_layer(slicing_plane, prev_slicing_plane);
						printing_layers.addPrintingLayer(printing_layer);
						prev_slicing_plane = slicing_plane;
					}
					else
					{
						n = line.find("End");
						if (n != std::string::npos)
						{
							m_partitions.back().setPrintingLayers(printing_layers);
							printing_layers = SO::PrintingLayerCollection();
							status = 0;
						}
						else
						{
							spdlog::error("Saved file not valid. Load fails.");
						}
					}
				}
			}
			inputFile.close();
			return true;
		}
		else 
		{
			std::cerr << "Error opening the file." << std::endl;
			return false;
		}

	}

	template <class T>
	inline void PartitionCollection<T>::reset()
	{
		m_partitions.clear();
	}

	template <class T>
	inline void PartitionCollection<T>::clear()
	{
		m_partitions.clear();
	}

	template <class T>
	inline void PartitionCollection<T>::addPartition(const Partition<T> &partition)
	{
		m_partitions.emplace_back(partition);
	}

	template<class T>
	inline void PartitionCollection<T>::revert()
	{
		std::vector<SO::Partition<T>> reverted_partitions;
		for (int i = m_partitions.size() - 1; i >= 0; i--)
		{
			reverted_partitions.emplace_back(m_partitions[i]);
		}
		m_partitions = reverted_partitions;
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

	template<class T>
	inline void PartitionCollection<T>::erase(int index)
	{
		m_partitions.erase(m_partitions.begin() + index);
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