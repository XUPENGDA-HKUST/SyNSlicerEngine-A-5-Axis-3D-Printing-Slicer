#include "swept_volume_calculater.h"

using SyNSlicerEngine::Algorithm::SweptVolumwCalculator;

SweptVolumwCalculator::SweptVolumwCalculator(const SO::Partition<CgalMesh_EPICK> &partition, vtkRenderer *p_renderer)
	: m_partition(partition)
{
	mp_printing_layer = nullptr;;
	mp_renderer = p_renderer;
	m_swept_volume_list.clear();
	std::vector<CgalPoint_EPICK> pointcloud;
	for (size_t i = 0; i < m_partition.getPrintingLayers().getNumberOfLayers(); i++)
	{
		Eigen::Vector3d normal = m_partition.getPrintingLayers().getLayer(i).getSlicingPlane().getNormal();
		for (size_t j = 0; j < m_partition.getPrintingLayers().getLayer(i).getNumberOfContours(); j++)
		{
			std::vector<Eigen::Vector3d> contour = m_partition.getPrintingLayers().getLayer(i).getContour(j).get();
			std::vector<CgalPoint_EPICK> temp_pointcloud = calculatePointCloud(contour, normal);
			pointcloud.insert(pointcloud.end(), temp_pointcloud.begin(), temp_pointcloud.end());
		}

		for (size_t j = 0; j < m_partition.getPrintingLayers().getLayer(i).getSupportStructureContours().numberOfPolygons(); j++)
		{
			std::vector<Eigen::Vector3d> contour = m_partition.getPrintingLayers().getLayer(i).getSupportStructureContours()[j].get();
			std::vector<CgalPoint_EPICK> temp_pointcloud = calculatePointCloud(contour, normal);
			pointcloud.insert(pointcloud.end(), temp_pointcloud.begin(), temp_pointcloud.end());
		}
	}
	CgalMesh_EPICK sm;
	CGAL::convex_hull_3(pointcloud.begin(), pointcloud.end(), sm);
	m_swept_volume_list.push_back(sm);
}

SweptVolumwCalculator::~SweptVolumwCalculator()
{
}

std::vector<CgalMesh_EPICK> SweptVolumwCalculator::getSweptVolume()
{
	return m_swept_volume_list;
}

bool SweptVolumwCalculator::makeVectorUnit(Eigen::Vector3d &vector)
{
	if (vector.norm() < 1e-6)
	{
		return false;
	}
	else
	{
		vector = vector / vector.norm();
		return true;
	}
}

CgalMesh_EPICK SweptVolumwCalculator::calculateConvexHull(std::vector<Eigen::Vector3d> contour, Eigen::Vector3d normal)
{
	std::vector<CgalPoint_EPICK> points;
	points = this->calculatePointCloud(contour, normal);
	CgalMesh_EPICK sm;
	CGAL::convex_hull_3(points.begin(), points.end(), sm);
	return sm;
}

std::vector<CgalPoint_EPICK> SweptVolumwCalculator::calculatePointCloud(std::vector<Eigen::Vector3d> contour, Eigen::Vector3d normal)
{
	std::vector<CgalPoint_EPICK> points;

	for (int j = 0; j < contour.size(); j++)
	{
		Eigen::Vector3d prev_point;
		Eigen::Vector3d cuurent_point;
		Eigen::Vector3d next_point;
		Eigen::Vector3d point_offset_direction;
		if (j == 0)
		{
			prev_point = contour.back();
			cuurent_point = contour[j];
			next_point = contour[j + 1];
		}
		else if (j == contour.size() - 1)
		{
			prev_point = contour[j - 1];
			cuurent_point = contour[j];
			next_point = contour.front();
		}
		else
		{
			prev_point = contour[j - 1];
			cuurent_point = contour[j];
			next_point = contour[j + 1];
		}

		Eigen::Vector3d v1 = next_point - cuurent_point;
		Eigen::Vector3d v2 = cuurent_point - prev_point;

		if (!this->makeVectorUnit(v1) || !this->makeVectorUnit(v2))
		{
			continue;
		}

		if (v1.cross(v2).dot(normal) > 0)
		{
			point_offset_direction = v1 - v2;
		}
		else
		{
			point_offset_direction = -(v1 - v2);
		}
		makeVectorUnit(point_offset_direction);
		makeVectorUnit(normal);

		Eigen::Vector3d offset_point = cuurent_point + 10 * normal + 5 * point_offset_direction;
		points.push_back(CgalPoint_EPICK(cuurent_point[0], cuurent_point[1], cuurent_point[2]));
		points.push_back(CgalPoint_EPICK(offset_point[0], offset_point[1], offset_point[2]));
	}

	return points;
}
