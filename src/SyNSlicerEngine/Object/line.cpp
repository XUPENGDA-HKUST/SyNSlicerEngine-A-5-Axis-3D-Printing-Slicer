#include "line.h"

using namespace SyNSlicerEngine::Object;

Line::Line()
	: m_source(Eigen::Vector3d(0, 0, 0))
	, m_target(Eigen::Vector3d(0, 0, 1))
	, m_direction(Eigen::Vector3d(0, 0, 1))
	, m_length(1.0)
	, m_is_valid(true)
{

}

Line::Line(const Line &other)
{
	*this = other;
}

Line::Line(const Eigen::Vector3d &source, const Eigen::Vector3d &target)
{
	setLine(source, target);
}

Line::Line(const Eigen::Vector3d &source, const Eigen::Vector3d &direction, double length)
{
	setLine(source, direction, length);
}

Line::Line(CgalMesh_EPICK::Halfedge_index he, CgalMesh_EPICK &mesh)
{
	auto &s = mesh.point(mesh.source(he));
	auto &t = mesh.point(mesh.target(he));
	Eigen::Vector3d source(s.x(), s.y(), s.z());
	Eigen::Vector3d target(t.x(), t.y(), t.z());
	setLine(source, target);
}

Line::Line(CgalMesh_EPECK::Halfedge_index he, CgalMesh_EPECK &mesh)
{
	auto &s = mesh.point(mesh.source(he));
	auto &t = mesh.point(mesh.target(he));
	Eigen::Vector3d source(
		CGAL::to_double(s.x()), 
		CGAL::to_double(s.y()), 
		CGAL::to_double(s.z()));
	Eigen::Vector3d target(
		CGAL::to_double(t.x()),
		CGAL::to_double(t.y()),
		CGAL::to_double(t.z()));
	setLine(source, target);
}

Line::~Line()
{
}

bool Line::isValid() const
{
	if (isnan(m_source[0])) { return false; };
	if (isnan(m_source[1])) { return false; };
	if (isnan(m_source[2])) { return false; };
	if (isnan(m_target[0])) { return false; };
	if (isnan(m_target[1])) { return false; };
	if (isnan(m_target[2])) { return false; };
	return true;
}

void Line::printInfo()
{
	printf("Source: %f %f %f, Target: %f %f %f, Length: %f\n", m_source[0], m_source[1], m_source[2], m_target[0], m_target[1], m_target[2], m_length);
}

void Line::setLine(const Eigen::Vector3d &source, const Eigen::Vector3d &target)
{
	m_source = source;
	m_target = target;
	m_length = (m_target - m_source).norm();
	if (m_length != 0)
	{
		m_direction = (m_target - m_source) / m_length;
	};
	m_is_valid = this->isValid();
}

void Line::setLine(const Eigen::Vector3d &source, const Eigen::Vector3d &direction, double length)
{
	m_source = source;
	m_direction = direction;
	if (m_direction.norm() != 0)
	{
		m_direction = m_direction / m_direction.norm();
	};
	m_length = length;
	m_target = m_source + m_length * m_direction;
	m_is_valid = this->isValid();
}

const Eigen::Vector3d &Line::getSource() const
{
	return m_source;
}

const Eigen::Vector3d &Line::getTarget() const
{
	return m_target;
}

const Eigen::Vector3d &Line::getDirection() const
{
	return m_direction;
}

Eigen::Vector3d Line::getMiddle() const
{
	return (m_source + m_target) / 2;
}

const double Line::getLength() const
{
	return m_length;
}

void Line::reverseDirection()
{
	Eigen::Vector3d temp = m_source;
	m_source = m_target;
	m_target = temp;
	m_direction = -m_direction;
}

double Line::getDistanceOfPoint(const Eigen::Vector3d &point) const
{	
	assert(m_is_valid);
	Eigen::Vector3d v0(point - m_source);
	double t = v0.dot(m_direction);
	Eigen::Vector3d v1 = m_source + t * m_direction;
	return (point - v1).norm();
}

double Line::getDistanceFromPointToLineSegment(const Eigen::Vector3d &point) const
{
	assert(m_is_valid);
	double distance = 0.0;
	Eigen::Vector3d v0(point - this->m_source);
	double t = v0.dot(this->m_direction);
	if (t >= 0 && t <= this->m_length)
	{
		Eigen::Vector3d v1 = this->m_source + t * this->m_direction;
		distance = (point - v1).norm();
		return distance;
	}
	else if (t < 0)
	{
		distance = (point - this->m_source).norm();
		return distance;
	}
	else
	{
		distance = (point - this->m_target).norm();
		return distance;
	}
}

Eigen::Vector3d Line::getProjectionOfPointOntoRay(const Eigen::Vector3d &point)
{
	assert(m_is_valid);
	Eigen::Vector3d result;
	Eigen::Vector3d v0(point - this->m_source);
	double t = v0.dot(this->m_direction);
	result = this->m_source + t * this->m_direction;
	return result;
}

bool Line::isProjectionOfPointLieOnLineSegment(const Eigen::Vector3d &point) const
{
	Eigen::Vector3d v0(point - this->getSource());
	double distance = v0.dot(this->getDirection());
	if (distance >= 0 && distance <= this->getLength())
	{
		return true;
	}
	else
	{
		return false;
	}
}

Line &Line::operator=(const Line &other)
{
	setLine(other.m_source, other.m_target);
	return *this;
}
