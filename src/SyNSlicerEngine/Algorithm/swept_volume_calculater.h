#ifndef SYNSLICERENGINE_ALGORITHM_SWEPTVOLUMECALCULATER_H_
#define SYNSLICERENGINE_ALGORITHM_SWEPTVOLUMECALCULATER_H_

#include <vector>
#include <fstream>

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
		SweptVolumwCalculator() = delete;

		//!  Calculate the convex hull of the volume swept by the nozzle during printing this partition
		/*!
			\param p_mesh The partition going to be printed.
			\param p_renderer The vtkRenderer.
		*/
		SweptVolumwCalculator(const SO::Partition<CgalMesh_EPICK> &partition);
		~SweptVolumwCalculator();

		// set the cross section of the nozzle here (Not finish yet)
		virtual void setCrossSectionOfNozzle(){}
		
		virtual std::vector<CgalMesh_EPICK> getSweptVolume();

	protected:
		virtual bool makeVectorUnit(Eigen::Vector3d &vector);
		virtual CgalMesh_EPICK calculateConvexHull(std::vector<Eigen::Vector3d> contour, Eigen::Vector3d normal);
		virtual std::vector<CgalPoint_EPICK> calculatePointCloud(std::vector<Eigen::Vector3d> contour, Eigen::Vector3d normal);

		std::vector<CgalMesh_EPICK> m_swept_volume_list;
		std::vector<SO::Polyhedron<CgalMesh_EPICK> *> m_mesh_list;

		SO::Partition<CgalMesh_EPICK> m_partition;
		SO::PrintingLayer *mp_printing_layer;
	};
}

#endif //SYNSLICERENGINE_ALGORITHM_SWEPTVOLUMECALCULATER_H_