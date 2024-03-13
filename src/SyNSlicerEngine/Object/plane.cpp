#include "plane.h"

using namespace SyNSlicerEngine::Object;

Plane::Plane()
	: m_origin(0, 0, 0)
	, m_normal(0, 0, 1)
	, m_distance(0.0)
{ 

}

Plane::Plane(const Plane &other)
{
	*this = other;
}

Plane::Plane(const Eigen::Vector3d &input_origin, const Eigen::Vector3d &input_normal)
{
	for (size_t i = 0; i < 3; i++)
	{
		m_origin[i] = input_origin[i];
		m_normal[i] = input_normal[i];
	}
	if (m_normal.norm() != 0)
	{
		m_normal = m_normal / m_normal.norm();
	}
	m_distance = -m_origin.dot(m_normal);
}

Plane::Plane(double a, double b, double c, double d)
{
	setPlaneInGeneralForm(a, b, c, d);
}

Plane::~Plane()
{

}

bool Plane::isValid()
{
	if (isnan(m_origin[0])) { return false; };
	if (isnan(m_origin[1])) { return false; };
	if (isnan(m_origin[2])) { return false; };
	if (isnan(m_normal[0])) { return false; };
	if (isnan(m_normal[0])) { return false; };
	if (isnan(m_normal[0])) { return false; };
	return true;
}

void Plane::printInfo()
{
	printf("Origin: %f %f %f, Normal: %f %f %f\n", 
		m_origin[0], m_origin[1], m_origin[2], 
		m_normal[0], m_normal[1], m_normal[2]);
}

void Plane::setOrigin(const Eigen::Vector3d &input_origin)
{
	m_origin = input_origin;
	m_distance = -m_origin.dot(m_normal);
}

const Eigen::Vector3d &Plane::getOrigin() const
{
	return m_origin;
}

void Plane::setNormal(const Eigen::Vector3d &input_normal)
{
	m_normal = input_normal/ input_normal.norm(); //make sure unit vector
	m_distance = -m_origin.dot(m_normal);
}

const Eigen::Vector3d &Plane::getNormal() const
{
	return m_normal;
}

void Plane::setPlaneInGeneralForm(double a, double b, double c, double d)
{
	Eigen::Vector3d n(a, b, c);
	if (n.norm() != 0)
	{
		d = d / n.norm();
		n = n / n.norm();
	};
	m_normal = n;
	m_distance = d;
	findPointLocatedOnPlane();
}

void Plane::offset(double input_distance)
{
	Eigen::Vector3d new_origin = m_origin + input_distance * m_normal;
	setOrigin(new_origin);
}

const double Plane::a() const
{
	return m_normal[0];
}

const double Plane::b() const
{
	return m_normal[1];
}

const double Plane::c() const
{
	return m_normal[2];
}

const double Plane::d() const
{
	return m_distance;
}

bool Plane::isPointOnPlane(const Eigen::Vector3d &point, double epsilon) const
{
	double distance = this->getDistanceFromPointToPlane(point);
	if (abs(distance) < epsilon)
	{
		return true;
	}
	return false;
}

bool Plane::isPointOnPlane(const CgalPoint_EPICK &point, double epsilon) const
{
	Eigen::Vector3d temp_point(point.x(), point.y(), point.z());
	double distance = this->getDistanceFromPointToPlane(temp_point);
	if (abs(distance) < epsilon)
	{
		return true;
	}
	return false;
}

double Plane::getDistanceFromPointToPlane(const Eigen::Vector3d &point) const
{
	Eigen::Vector3d v0 = point - m_origin;
	double distance = v0.dot(m_normal);
	return distance;
}

double Plane::getDistanceFromPointToPlane(const CgalPoint_EPICK &point) const
{
	Eigen::Vector3d temp_point(point.x(), point.y(), point.z());
	Eigen::Vector3d v0 = temp_point - m_origin;
	double distance = v0.dot(m_normal);
	return distance;
}

Eigen::Vector3d Plane::getProjectionOfPointOntoPlane(const Eigen::Vector3d &point) const
{
	Eigen::Vector3d v0 = point - m_origin;
	double distance = v0.dot(m_normal);
	Eigen::Vector3d point_projection = v0 - distance * m_normal + m_origin;
	assert(abs(this->getDistanceFromPointToPlane(point_projection)) < 1e-6);
	return point_projection;
}

bool Plane::isLineOnPlane(const SO::Line &line, double epsilon) const
{
	return isPointOnPlane(line.getSource(), epsilon) &&
		isPointOnPlane(line.getTarget(), epsilon);
}

bool Plane::isIntersectedWithPlane(const Plane &other) const
{
	Eigen::Vector3d n12 = m_normal.cross(other.m_normal);
	if (n12.norm() < 1e-6)
	{
		return false;
	}
	else
	{
		return true;
	}
}

Plane &Plane::operator=(const Plane &other)
{
	setOrigin(other.m_origin);
	setNormal(other.m_normal);
	return *this;
}

bool Plane::operator==(const Plane &other)
{
	if (abs(m_normal[0] - other.m_normal[0]) < 1e-6 &&
		abs(m_normal[1] - other.m_normal[1]) < 1e-6 &&
		abs(m_normal[2] - other.m_normal[2]) < 1e-6 &&
		abs(m_distance - other.m_distance) < 1e-6)
	{
		return true;
	}
	return false;
}

bool Plane::operator==(const Plane &other) const
{
	if (abs(m_normal[0] - other.m_normal[0]) < 1e-6 &&
		abs(m_normal[1] - other.m_normal[1]) < 1e-6 &&
		abs(m_normal[2] - other.m_normal[2]) < 1e-6 &&
		abs(m_distance - other.m_distance) < 1e-6)
	{
		return true;
	}
	return false;
}

bool Plane::operator!=(const Plane &other)
{
	return !operator==(other);
}

bool Plane::operator!=(const Plane &other) const
{
	return !operator==(other);
}

Eigen::Vector3d Plane::findPointLocatedOnPlane() 
{
	srand((unsigned)time(NULL));

	bool done = false;

	while (!done)
	{
		int x = rand() % 10;
		int y = rand() % 10;
		int z = rand() % 10;

		Eigen::Vector3d point(x, y, z);
		double numerator = abs(point.dot(m_normal) + m_distance);
		double denominator = sqrt(pow(m_normal[0], 2) + pow(m_normal[1], 2) + pow(m_normal[2], 2));
		double D = numerator / denominator;
		if (abs(D - 0.0) < 1e-6)
		{
			
		}
		else
		{
			m_origin = point - D * -m_normal;
			done = true;
		}
	}
	return m_origin;
}
