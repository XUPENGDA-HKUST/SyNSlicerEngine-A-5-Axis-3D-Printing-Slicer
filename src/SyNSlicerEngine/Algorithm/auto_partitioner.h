#ifndef SYNSLICERENGINE_ALGORITHM_AUTOPARTITIONER_H_
#define SYNSLICERENGINE_ALGORITHM_AUTOPARTITIONER_H_

#include <cmath>
#include <deque>
#include <vector>

#include <CGAL_CORE_CLASS>
#include <CGAL/extract_mean_curvature_flow_skeleton.h>

#include "spdlog/spdlog.h"

#include "Object/point_cloud.h"
#include "Object/nozzle.h"
#include "Object/triangle.h"
#include "Object/partition.h"
#include "Object/partition_collection.h"
#include "Object/skeleton.h"

namespace SO = SyNSlicerEngine::Object;

//!  This namespace is used to hold all the algorithm.
namespace SyNSlicerEngine::Algorithm
{
	//!  This class is used to partition a 3D model automatically.
	class AutoPartitioner
	{
	public:
		AutoPartitioner() = delete;

		//! Constructer
		/*!
			\param partition The partition to be partitioned.
			\param nozzle The nozzle of the 3D printer.
			\param overhanging_angle The maximum overhanging angle of the 3D model. Default value: 56.
		*/
		AutoPartitioner(const SO::Partition<CgalMesh_EPICK> &partition, const SO::Nozzle &nozzle, double overhanging_angle = 56, double area_threshold_coefficient = 1.0);
		~AutoPartitioner();

		//! Call this method to start partition.
		virtual void partition();

		//! Call this method to obtain the result.
		SO::PartitionCollection<CgalMesh_EPICK> getResult();

	protected:
		using EigenPoint = Eigen::Vector3d;
		using EigenPoints = std::vector<Eigen::Vector3d>;
		using EigenContour = std::vector<Eigen::Vector3d>;
		using EigenContours = std::vector<EigenContour>;
		using CgalContour_EPICK = CgalPolyline_EPICK;
		using CgalContours_EPICK = std::vector<CgalContour_EPICK>;

		struct ResultOfFindNeighbour
		{
			std::vector<int> faces;
			int lowest_point;
		};

		struct OverhangingRegion
		{
			std::vector<int> faces;
			double area_projected_on_base_plane = 0.0;
		};

		struct ResultOfDetermineClippingPlane
		{
			SO::Plane clipping_plane;
			EigenPoints points_in_overhanging_triangles;
			EigenPoints points_in_base_contours;
			bool status = false;	
		};

		virtual void partitionMesh(SO::Partition<CgalMesh_EPECK> &partition, SO::PartitionCollection<CgalMesh_EPECK> &partition_list, std::vector<SO::PointCloud> &vertices_to_ignore_list);

		virtual ResultOfDetermineClippingPlane determineClippingPlane(SO::Partition<CgalMesh_EPICK> &partition,
			SO::Plane &clipping_plane, std::vector<SO::PointCloud> &vertices_to_ignore_list);

		virtual double getAreaOfOverhangingTrianglesProjectedOnBasePlane(std::vector<int> faces, const CgalMesh_EPICK &mesh, const SO::Plane &base_plane);
		virtual OverhangingRegion findLargestOverhangingRegion(std::vector<CgalMesh_EPICK::Face_index> faces_to_search,
			CgalMesh_EPICK &mesh, const SO::Plane &base_plane, std::vector<SO::PointCloud> &vertices_to_ignore_list, double area_threshold_coefficient = 1);

		virtual bool clipPartition(SO::Partition<CgalMesh_EPECK> &partition,
			ResultOfDetermineClippingPlane &clipping_plane,
			SO::Partition<CgalMesh_EPECK> &partition_low, SO::Partition<CgalMesh_EPECK> &partition_up);

		virtual bool hasPointsOnNegativeSide(const EigenPoints &points, SO::Plane &plane);
		virtual bool hasPointsOnPositiveSide(const EigenPoints &points, SO::Plane &plane);

		virtual bool adjustPlaneOriginSoPointsAreOnPostiveSide(const EigenPoints &points, SO::Plane &plane);

		virtual bool adjustPlaneOriginSoPointsAreOnNegativeSide(const EigenPoints &points, SO::Plane &plane);
		virtual bool adjustPlaneNormalSoPointsAreOnNegativeSide(const EigenPoints &points, const SO::Plane &reference_plane, const SO::Plane &base_plane, SO::Plane &clipping_plane);

		virtual bool checkClippingPlaneNormal(SO::Plane &clipping_plane, const EigenPoint &centroid_of_base_contours);

		SO::Partition<CgalMesh_EPICK> m_partition;
		SO::PartitionCollection<CgalMesh_EPECK> m_partition_list;
		SO::PartitionCollection<CgalMesh_EPICK> m_results;
		std::vector<SO::Plane> m_slicing_planes;
		CgalContours_EPICK m_prev_contours;

		SO::Nozzle m_nozzle;
		double m_overhanging_angle;
		double m_area_threshold_coefficient;

		int m_partition_time;
	};
}

#endif //SYNSLICERENGINE_ALGORITHM_AUTOPARTITIONER_H_