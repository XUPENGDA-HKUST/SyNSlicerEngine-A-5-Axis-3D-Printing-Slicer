#ifndef SYNSLICERENGINE_ALGORITHM_SWEPTVOLUMECALCULATER_H_
#define SYNSLICERENGINE_ALGORITHM_SWEPTVOLUMECALCULATER_H_

#include <vector>
#include <fstream>

#include "Object/nozzle.h"
#include "Object/printing_layer.h"
#include "Object/partition.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Algorithm
{
	//!  This class is used to calculate volume of the nozzle during printing
	/*!
		
	*/
	class SweptVolumwCalculator
	{
	public:
		//! Default constructor is not allowed.
		SweptVolumwCalculator() = delete;

		//! Constructor
		/*!
			\param[in] partition_list All the partitions.
			\param[in] nozzle The printer nozzle.
		*/
		SweptVolumwCalculator(const SO::Partition<CgalMesh_EPICK> &partition, SO::Nozzle nozzle);
		
		//! Destructor
		~SweptVolumwCalculator();
		
		//! Call to start calculation.
		virtual void calculateSweptVolume();

		//! Call to calculate the convex hull of the volume swept by the nozzle during printing this partition
		virtual std::vector<CgalMesh_EPICK> getSweptVolume();

	protected:

		//! Convert a vector to unit vector
		/*!
			\param[in,out] vector Vector to convert.
			\return Return \b True \b  if it is a unit vector. Return \b False \b if it is a zero vector.
		*/
		virtual bool makeVectorUnit(Eigen::Vector3d &vector);

		//! Calculate the Swept volume of the nozzle for one layer
		/*!
			\param[in] contour The path of the nozzle.
			\param[in] normal The normal of the nozzle.
		*/
		virtual CgalMesh_EPICK calculateConvexHull(std::vector<Eigen::Vector3d> contour, Eigen::Vector3d normal);

		//! Calculate the point cloud of the nozzle.
		/*!
			\param[in] contour The path of the nozzle.
			\param[in] normal The normal of the nozzle.
		*/
		virtual std::vector<CgalPoint_EPICK> calculatePointCloud(std::vector<Eigen::Vector3d> contour, Eigen::Vector3d normal);

		//! The swept volume of the nozzle for when printing each partition.
		std::vector<CgalMesh_EPICK> m_swept_volume_list;

		//! The partitions to be printed.
		SO::Partition<CgalMesh_EPICK> m_partition;

		//! The nozzle of the printer.
		SO::Nozzle m_nozzle;
	};
}

#endif //SYNSLICERENGINE_ALGORITHM_SWEPTVOLUMECALCULATER_H_