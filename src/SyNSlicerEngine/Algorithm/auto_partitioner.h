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
		//! Defualt constructer is not allowed.
		AutoPartitioner() = delete;

		//! Constructer
		/*!
			\param partition The partition to be partitioned.
			\param nozzle The nozzle of the 3D printer.
			\param overhanging_angle The maximum overhanging angle of the 3D model.
			\param area_threshold_coefficient The mimimum area of overhanging region this algorithm will handle. \n
					mimimum_area = area_threshold_coefficient * average_area_of_triangle_facets.
		*/
		AutoPartitioner(const SO::Partition<CgalMesh_EPICK> &partition, const SO::Nozzle &nozzle, double overhanging_angle = 56, double area_threshold_coefficient = 1.0);

		//! Destructer
		~AutoPartitioner();

		//! Call this method to start partition.
		virtual void partition();

		//! Call this method to obtain the result.
		SO::PartitionCollection<CgalMesh_EPICK> getResultEPICK();
		SO::PartitionCollection<CgalMesh_EPECK> getResultEPECK();

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

		//! Partition a 3D model into two parts and then store in partition_list
		/*!
			\param partition The 3D model to be partitioned.
			\param partition_list The container to store the result.
			\param vertices_to_ignore_list A list used to avoid handing the same overhanging region repeately.
		*/
		virtual void partitionMesh(SO::Partition<CgalMesh_EPECK> &partition, SO::PartitionCollection<CgalMesh_EPECK> &partition_list, std::vector<SO::PointCloud> &vertices_to_ignore_list);

		//! Determine the clipping plane used to partition a a 3D model
		/*!
			\param partition The 3D model to be partitioned.
			\param clipping_plane The plane used to partition the model.
			\param vertices_to_ignore_list A list used to avoid handing the same overhanging region repeately.
		*/
		virtual ResultOfDetermineClippingPlane determineClippingPlane(SO::Partition<CgalMesh_EPICK> &partition,
			SO::Plane &clipping_plane, std::vector<SO::PointCloud> &vertices_to_ignore_list);

		//! Calculate the area of overhanging triangles projected on plane.
		/*!
			\param faces The faces for computation.
			\param mesh The 3D model.
			\param base_plane The plane of projection.
		*/
		virtual double getAreaOfOverhangingTrianglesProjectedOnBasePlane(std::vector<int> faces, const CgalMesh_EPICK &mesh, const SO::Plane &base_plane);

		//! Find largest overhanging region of a 3D model.
		/*!
			Definition of overhanging region: If two overhanging triangles have one common vertices, they form a region.

			\param faces_to_search The faces for computation.
			\param mesh The 3D model.
			\param base_plane The plane where the 3D model located on.
			\param vertices_to_ignore_list A list used to avoid handing the same overhanging region repeately.
			\param area_threshold_coefficient The mimimum area of overhanging region this algorithm will handle. \n
					mimimum_area = area_threshold_coefficient * average_area_of_triangle_facets. \n
		*/
		virtual OverhangingRegion findLargestOverhangingRegion(std::vector<CgalMesh_EPICK::Face_index> faces_to_search,
			CgalMesh_EPICK &mesh, const SO::Plane &base_plane, std::vector<SO::PointCloud> &vertices_to_ignore_list, double area_threshold_coefficient = 1);

		//! Seperate a 3D model into two parts with a plane.
		/*!
			\param partition The 3D model.
			\param clipping_plane The plane used to clip the 3D model.
			\param partition_low The part that touchs the base plane of the partition.
			\param partition_up The other part.
		*/
		virtual bool clipPartition(SO::Partition<CgalMesh_EPECK> &partition,
			ResultOfDetermineClippingPlane &clipping_plane,
			SO::Partition<CgalMesh_EPECK> &partition_low, SO::Partition<CgalMesh_EPECK> &partition_up);

		//! Check if any point in a point cloud on negative side of the given plane.
		/*!
			\param points The point cloud.
			\param plane The given plane.
		*/
		virtual bool hasPointsOnNegativeSide(const EigenPoints &points, SO::Plane &plane);

		//! Check if any point in a point cloud on positive side of the given plane.
		/*!
			\param points The point cloud.
			\param plane The given plane.
		*/
		virtual bool hasPointsOnPositiveSide(const EigenPoints &points, SO::Plane &plane);

		//! Adjust the origin of the given plane so that all the points move to positive side of the given plane.
		/*!
			\param points The point cloud.
			\param plane The given plane.
		*/
		virtual bool adjustPlaneOriginSoPointsAreOnPostiveSide(const EigenPoints &points, SO::Plane &plane);

		//! Adjust the origin of the given plane so that all the points are moved to positive side of the given plane.
		/*!
			\param points The point cloud.
			\param plane The given plane.
		*/
		virtual bool adjustPlaneOriginSoPointsAreOnNegativeSide(const EigenPoints &points, SO::Plane &plane);

		//! Adjust the normal of the given plane so that most of the points are moved to negative side of the given plane.
		/*!
			Alert! Not all the points can be moved to negative side.
			\param points The point cloud.
			\param reference_plane The new normal of the given clipping plane must lies on reference_plane.
			\param base_plane The plane where the 3D model locate on.
			\param clipping_plane The given clipping plane.
		*/
		virtual bool adjustPlaneNormalSoPointsAreOnNegativeSide(const EigenPoints &points, const SO::Plane &reference_plane, const SO::Plane &base_plane, SO::Plane &clipping_plane);

		//! Input 3D model.
		SO::Partition<CgalMesh_EPICK> m_partition;

		//! Container of intermediate result.
		SO::PartitionCollection<CgalMesh_EPECK> m_partition_list;

		//! Output.
		SO::PartitionCollection<CgalMesh_EPICK> m_results;

		//! SO::Nozzle is used to store nozzle parameters.
		SO::Nozzle m_nozzle;

		//! Overhanging angle used to determine overhanging triangle.
		double m_overhanging_angle;

		//! A parameter used to stop partition triggered by tiny overhanging region.
		double m_area_threshold_coefficient;

		//! A parameter used for debug
		int m_partition_time;
	};
}

#endif //SYNSLICERENGINE_ALGORITHM_AUTOPARTITIONER_H_