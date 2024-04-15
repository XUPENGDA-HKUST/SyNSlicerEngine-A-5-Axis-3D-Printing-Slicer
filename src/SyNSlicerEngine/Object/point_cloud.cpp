#include "point_cloud.h"

using SyNSlicerEngine::Object::PointCloud;

PointCloud::PointCloud()
{
	m_status = Empty;
}

PointCloud::PointCloud(const PointCloud &other)
{
	*this = other;
}

PointCloud::~PointCloud()
{

}

void PointCloud::reset()
{
	m_points.clear();
	m_status = Empty;
}

void PointCloud::addPoint(Eigen::Vector3d point)
{
	m_points.push_back(point);
	updateBound(point);
}

void PointCloud::getBound(double bound[6])
{
	for (int i = 0; i < 6; i++)
	{
		bound[i] = m_bound[i];
	}
}

std::vector<Eigen::Vector3d> PointCloud::points()
{
	return m_points;
}

int PointCloud::size() const
{
	return m_points.size();
}

void PointCloud::updateBound(Eigen::Vector3d point)
{
	if (m_status == Empty)
	{
		m_bound[0] = point[0];
		m_bound[1] = point[0];
		m_bound[2] = point[1];
		m_bound[3] = point[1];
		m_bound[4] = point[2];
		m_bound[5] = point[2];
		m_status = FirstPointAdded;
	}
	else
	{
		if (point[0] < m_bound[0]) { m_bound[0] = point[0];};
		if (point[0] > m_bound[1]) { m_bound[1] = point[0];};
		if (point[1] < m_bound[2]) { m_bound[2] = point[1];};
		if (point[1] > m_bound[3]) { m_bound[3] = point[1];};
		if (point[2] < m_bound[4]) { m_bound[4] = point[2];};
		if (point[2] > m_bound[5]) { m_bound[5] = point[2];};
	}
}

Eigen::Vector3d PointCloud::operator[](int index)
{
	assert(index >= 0 && index < m_points.size());
	return m_points[index];
}

const Eigen::Vector3d &PointCloud::operator[](int index) const
{
	assert(index >= 0 && index < m_points.size());
	return m_points[index];
}

PointCloud &PointCloud::operator=(const PointCloud &other)
{
	m_points = other.m_points;
	for (size_t i = 0; i < 6; i++)
	{
		m_bound[i] = other.m_bound[i];
	}
	m_status = other.m_status;
	return *this;
}