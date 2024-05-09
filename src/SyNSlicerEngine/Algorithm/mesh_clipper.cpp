#include "mesh_clipper.h"

using SyNSlicerEngine::Algorithm::MeshClipper;

MeshClipper::MeshClipper()
	: clipping_time(0)
	, mp_partition(nullptr)
{

}

MeshClipper::~MeshClipper()
{ 

}

void MeshClipper::setPartition(SO::Partition<CgalMesh_EPICK> *p_partition)
{
	mp_partition = p_partition;
}

bool MeshClipper::clipWithInfinitePlane(const SO::Plane &plane, SO::PartitionCollection<CgalMesh_EPICK> &result)
{
	result.clear();

	SO::Partition<CgalMesh_EPICK> &partition = *mp_partition;

	if (partition.getBaseContours().isIntersectedWithPlane(plane))
	{
		return false;
	};

	SO::Plane clip_plane = plane;

	if (clip_plane.getPositionOfPointWrtPlane(partition.getBaseContours().centroid()) == 1)
	{
		clip_plane.setNormal(-clip_plane.getNormal());
	}

	CgalMesh_EPICK pos_mesh = partition.getEPICKMesh();
	CgalMesh_EPICK neg_mesh = partition.getEPICKMesh();

	CgalPlane_EPICK cgal_plane(clip_plane.a(), clip_plane.b(), clip_plane.c(), clip_plane.d());

	CGAL::Polygon_mesh_processing::clip(neg_mesh, cgal_plane, CGAL::parameters::clip_volume(true));
	CGAL::Polygon_mesh_processing::clip(pos_mesh, cgal_plane.opposite(), CGAL::parameters::clip_volume(true));

	pos_mesh.collect_garbage();
	neg_mesh.collect_garbage();

	if (pos_mesh.is_empty())
	{
		return false;
	}

	SO::Partition<CgalMesh_EPICK> up_partition(pos_mesh);
	up_partition.setBasePlane(clip_plane);
	up_partition.addLock(clipping_time);
	SO::Partition<CgalMesh_EPICK> low_partition(neg_mesh);
	low_partition.setBasePlane(partition.getBasePlane());
	low_partition.addKey(clipping_time);

	clipping_time += 1;

	result.addPartition(low_partition);
	result.addPartition(up_partition);

	return true;
}

bool MeshClipper::clipWithFinitePlane(const SO::Line &line, const Eigen::Vector3d &camera_position, const SO::Plane &clipping_plane, SO::PartitionCollection<CgalMesh_EPICK> &result)
{
	return false;
}
