#ifndef SYNSLICERENGINE_ALGORITHM_AUTOPARTITIONER_H_
#define SYNSLICERENGINE_ALGORITHM_AUTOPARTITIONER_H_

#include <cmath>
#include <deque>
#include <vector>

#include <CGAL_CORE_CLASS>
#include <CGAL/extract_mean_curvature_flow_skeleton.h>

#include "spdlog/spdlog.h"

#include "Object/nozzle.h"
#include "Object/triangle.h"
#include "Object/partition.h"
#include "Object/partition_collection.h"
#include "Object/skeleton.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Algorithm
{
	class AutoPartitioner
	{
	public:
		AutoPartitioner() = delete;
		AutoPartitioner(const SO::Partition<CgalMesh_EPICK> &partition, const SO::Nozzle &nozzle);
		~AutoPartitioner();

		enum Case { ResultValid = 0, ResultInvalid = 1, TriangleTooSmall = 2 };

		void partition();

		SO::PartitionCollection<CgalMesh_EPICK> getResult();

	private:
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

		void partitionMesh(SO::Partition<CgalMesh_EPECK> &partition, SO::PartitionCollection<CgalMesh_EPECK> &partition_list, EigenPoints &vertices_to_ignore_list);

		ResultOfDetermineClippingPlane determineClippingPlane(SO::Partition<CgalMesh_EPICK> &partition,
			SO::Plane &clipping_plane, EigenPoints &vertices_to_ignore_list);

		double getAreaOfOverhangingTrianglesProjectedOnBasePlane(std::vector<int> faces, const CgalMesh_EPICK &mesh, const SO::Plane &base_plane);
		OverhangingRegion findLargestOverhangingRegion(std::vector<CgalMesh_EPICK::Face_index> faces_to_search,
			CgalMesh_EPICK &mesh, const SO::Plane &base_plane, EigenPoints &vertices_to_ignore_list, double area_threshold = 0);

		bool clipPartition(SO::Partition<CgalMesh_EPECK> &partition,
			ResultOfDetermineClippingPlane &clipping_plane,
			SO::Partition<CgalMesh_EPECK> &partition_low, SO::Partition<CgalMesh_EPECK> &partition_up);

		bool hasPointsOnNegativeSide(const EigenPoints &points, SO::Plane &plane);
		bool hasPointsOnPositiveSide(const EigenPoints &points, SO::Plane &plane);

		bool adjustPlaneOriginSoPointsAreOnPostiveSide(const EigenPoints &points, SO::Plane &plane);

		bool adjustPlaneOriginSoPointsAreOnNegativeSide(const EigenPoints &points, SO::Plane &plane);
		int adjustPlaneNormalSoPointsAreOnNegativeSide(const EigenPoints &points, const SO::Plane &reference_plane, const SO::Plane &base_plane, SO::Plane &clipping_plane);

		SO::Partition<CgalMesh_EPICK> m_partition;
		SO::PartitionCollection<CgalMesh_EPECK> m_partition_list;
		SO::PartitionCollection<CgalMesh_EPICK> m_results;
		std::vector<SO::Plane> m_slicing_planes;
		CgalContours_EPICK m_prev_contours;

		SO::Nozzle m_nozzle;
	};
}

#endif //SYNSLICERENGINE_ALGORITHM_AUTOPARTITIONER_H_