#include "printing_sequence_determinator_debug.h"

using SyNSlicerEngine::GUI::PrintingSequenceDeterminatorDebug;

PrintingSequenceDeterminatorDebug::PrintingSequenceDeterminatorDebug(
	SO::PartitionCollection<CgalMesh_EPICK> &partition_list, SO::Nozzle nozzle, vtkRenderer *renderer)
	: SA::PrintingSequenceDeterminator(partition_list, nozzle)
	, m_drawer(renderer)
{

}

PrintingSequenceDeterminatorDebug::~PrintingSequenceDeterminatorDebug()
{

}

bool PrintingSequenceDeterminatorDebug::isSweptVolumeIntersectBuildPlate()
{
	SO::Plane plane;
	double distance = 0.0;
	for (size_t i = 0; i < m_swept_volume_list.size(); i++)
	{
		CgalMesh_EPICK cgal_mesh = m_swept_volume_list[i];
		for (CgalMesh_EPICK::Vertex_index vid: cgal_mesh.vertices())
		{
			CgalPoint_EPICK cgal_point = cgal_mesh.point(vid);
			Eigen::Vector3d point(cgal_point.x(), cgal_point.y(), cgal_point.z());

			distance = plane.getDistanceFromPointToPlane(point);
			if (distance < 0)
			{
				m_drawer.drawMesh(cgal_mesh, "Mesh");
				m_drawer.setOpacity("Mesh", 0.2);
				return true;
			}
		}
	}
	return false;
}