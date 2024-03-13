#include "tool_point.h"

using SyNSlicerEngine::Object::ToolPoint;

ToolPoint::ToolPoint()
	: m_position(Eigen::Vector3d(0, 0, 0))
	, m_normal(Eigen::Vector3d(0, 0, 1))
	, m_extrusion(0.0)
	, m_local_layer_thickness(0.0)
{

}

ToolPoint::ToolPoint(const ToolPoint &other)
{
	*this = other;
}

ToolPoint::ToolPoint(Eigen::Vector3d position, Eigen::Vector3d nomral, double extrusion, double layer_thickness)
	: m_position(position)
	, m_normal(nomral)
	, m_extrusion(extrusion)
	, m_local_layer_thickness(layer_thickness)
{
	assert(m_position[2] >= 0);
	assert(m_normal[2] > 0);
	assert(m_local_layer_thickness >= 0);
}

ToolPoint::~ToolPoint()
{
}

void ToolPoint::setPosition(const Eigen::Vector3d &position)
{
	m_position = position;
}

const Eigen::Vector3d &ToolPoint::getPosition() const
{
	return m_position;
}

void ToolPoint::setNormal(const Eigen::Vector3d &normal)
{
	m_normal = normal;
}

const Eigen::Vector3d &ToolPoint::getNormal() const
{
	return m_normal;
}

void ToolPoint::setExtrusion(double extrusion)
{
	m_extrusion = extrusion;
}

const double &ToolPoint::getExtrusion() const
{
	return m_extrusion;
}

void ToolPoint::setLayerThickness(double local_layer_thickness)
{
	m_local_layer_thickness = local_layer_thickness;
}

const double &ToolPoint::getLayerThickness() const
{
	return m_local_layer_thickness;
}

const double ToolPoint::operator[](int index) const
{
	assert(index >= 0 && index <= 7);
	switch (index)
	{
	case 0:
		return m_position[0];
	case 1:
		return m_position[1];
	case 2:
		return m_position[2];
	case 3:
		return m_normal[0];
	case 4:
		return m_normal[1];
	case 5:
		return m_normal[2];
	case 6:
		return m_extrusion;
	case 7:
		return m_local_layer_thickness;
	default:
		break;
	}
	return std::numeric_limits<double>::min();
}

ToolPoint &ToolPoint::operator=(const ToolPoint &other)
{
	m_position = other.m_position;
	m_normal = other.m_normal;
	m_extrusion = other.m_extrusion;
	m_local_layer_thickness = other.m_local_layer_thickness;
	assert(m_position[2] >= 0);
	assert(m_normal[2] > 0);
	assert(m_local_layer_thickness >= 0);
	return *this;
}