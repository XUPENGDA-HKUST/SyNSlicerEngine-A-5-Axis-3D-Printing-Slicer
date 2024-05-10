#ifndef SYNSLICERENGINE_ALGORITHM_MESHCLIPPER_H_
#define SYNSLICERENGINE_ALGORITHM_MESHCLIPPER_H_

#include <vector>

#include <clipper2/clipper.h>

#include <CGAL_CORE_CLASS>

#include "Object/polygon.h"
#include "Object/partition.h"
#include "Object/partition_collection.h"
#include "Object/printing_layer_collection.h"
#include "Object/plane.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Algorithm
{
	//! This class is used to slice a 3D model with already calculated planes.
	/*!
	
	*/
	class MeshClipper
	{
	public:
		//! Default constructor.
		MeshClipper();

		//! Destructor.
		~MeshClipper();

		void setPartition(SO::Partition<CgalMesh_EPICK> *p_partition);

		//! Call to perform slicing.
		virtual bool clipWithInfinitePlane(
			const SO::Plane &plane, 
			SO::PartitionCollection<CgalMesh_EPICK> &result);

		virtual bool clipWithFinitePlane(
			const SO::Line &line,
			const Eigen::Vector3d &camera_position,
			const SO::Plane &clipping_plane,
			SO::PartitionCollection<CgalMesh_EPICK> &result);


	protected:
		virtual bool findTriangleContour(
			const SO::Line &line, 
			const SO::Plane &plane,
			std::vector<CgalMesh_EPICK::Face_index> &triangle_contour);

		virtual bool findAllSperatedMeshes(
			std::vector<CgalMesh_EPICK::Face_index> &triangle_contour, 
			const SO::Line &line, 
			const SO::Plane &plane);
		
		virtual bool checkHalfedgeIntersectLine(
			CgalMesh_EPICK::Halfedge_index halfedge_idx, 
			const CgalMesh_EPICK &mesh,
			const SO::Line &line, 
			const SO::Plane &plane);

		virtual bool isFaceContainedInPlane(
			CgalMesh_EPICK::halfedge_index halfedge_idx,
			const CgalMesh_EPICK &mesh,
			const SO::Plane &plane);

		virtual std::vector<std::vector<CgalMesh_EPICK::Face_index>> findNumberOfTriangleContours(
			std::vector<bool> &triangle_list, 
			const CgalMesh_EPICK &mesh);

		virtual void getNeighborTriangle(
			std::vector<CgalMesh_EPICK::Face_index> &face_list,
			std::vector<bool> &triangle_list,
			const CgalMesh_EPICK &mesh);

		virtual bool isTriangleContourCloseLoop(
			std::vector<CgalMesh_EPICK::Face_index> triangle_contour,
			const CgalMesh_EPICK &mesh,
			const SO::Line &line,
			const SO::Plane &plane);

		virtual void determineBasePlane(
			SO::Partition<CgalMesh_EPICK> &model,
			const SO::Plane &base_plane,
			const SO::Plane &clipping_plane);

		virtual void extractFacetsFromMeshToNewMesh(
			std::vector<CgalMesh_EPICK::Face_index> facets, 
			const CgalMesh_EPICK &mesh_in, 
			CgalMesh_EPICK &mesh_out);

		virtual bool isTwoMeshesTouchEachOther(
			const CgalMesh_EPICK &mesh_1,
			const CgalMesh_EPICK &mesh_2);

		virtual void combineTwoMeshes(
			CgalMesh_EPICK &mesh_to_combined,
			CgalMesh_EPICK mesh_to_be_added);

		virtual CgalMesh_EPICK::Vertex_index addVertextoMesh(
			CgalPoint_EPICK point,
			CgalMesh_EPICK &mesh);

		int clipping_time;
		SO::Partition<CgalMesh_EPICK> *mp_partition;
		SO::PartitionCollection<CgalMesh_EPICK> m_partitions;
	};
}

#endif  // SYNSLICERENGINE_ALGORITHM_MESHCLIPPER_H_