#include "plane.h"

using namespace SyNSlicerEngine::Object;

Plane::Plane()
	: m_origin(0, 0, 0)
	, m_normal(0, 0, 1)
	, m_distance(0.0)
	, m_is_valid(true)
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

Plane::Plane(std::string file_name)
{
	this->load(file_name);
}

Plane::~Plane()
{

}

bool Plane::save(std::string file_name)
{
	std::ofstream myfile(file_name);

	if (myfile.is_open()) {
		myfile << -2 << " " << m_origin[0] << " " << m_origin[1] << " " << m_origin[2] << "\n";
		myfile << -1 << " " << m_normal[0] << " " << m_normal[1] << " " << m_normal[2] << "\n";
		myfile.close();
		return true;
	}
	else 
	{
		return false;
	}
}

bool Plane::load(std::string file_name)
{
	std::ifstream inputFile(file_name);
	if (inputFile.is_open()) {
		std::string line;
		Eigen::Vector3d temp_point;
		int current_index = 0;
		while (std::getline(inputFile, line))
		{
			std::sscanf(line.c_str(), "%d %lf %lf %lf", &current_index, &temp_point[0], &temp_point[1], &temp_point[2]);
			if (current_index == -2)
			{
				this->setOrigin(temp_point);
				continue;
			}
			if (current_index == -1)
			{
				this->setNormal(temp_point);
				continue;
			}
		}
		inputFile.close();
		return true;
	}
	else
	{
		return false;
	}
}

bool Plane::isValid()
{
	if (isnan(m_origin[0])) { return false; };
	if (isnan(m_origin[1])) { return false; };
	if (isnan(m_origin[2])) { return false; };
	if (isnan(m_normal[0])) { return false; };
	if (isnan(m_normal[1])) { return false; };
	if (isnan(m_normal[2])) { return false; };
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
	m_is_valid = this->isValid();
}

const Eigen::Vector3d &Plane::getOrigin() const
{
	return m_origin;
}

void Plane::setNormal(const Eigen::Vector3d &input_normal)
{
	m_normal = input_normal/ input_normal.norm(); //make sure unit vector
	m_distance = -m_origin.dot(m_normal);
	m_is_valid = this->isValid();
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
	m_is_valid = this->isValid();
}

void Plane::offset(double input_distance)
{
	assert(m_is_valid);
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
	assert(m_is_valid);
	double distance = this->getDistanceFromPointToPlane(point);
	if (abs(distance) < epsilon)
	{
		return true;
	}
	return false;
}

bool Plane::isPointOnPlane(const CgalPoint_EPICK &point, double epsilon) const
{
	assert(m_is_valid);
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
	assert(m_is_valid);
	Eigen::Vector3d v0 = point - m_origin;
	double distance = v0.dot(m_normal);
	return distance;
}

double Plane::getDistanceFromPointToPlane(const CgalPoint_EPICK &point) const
{
	assert(m_is_valid);
	Eigen::Vector3d temp_point(point.x(), point.y(), point.z());
	Eigen::Vector3d v0 = temp_point - m_origin;
	double distance = v0.dot(m_normal);
	return distance;
}

int Plane::getPositionOfPointWrtPlane(const Eigen::Vector3d &point, double epsilon) const
{
	assert(m_is_valid);
	if (this->isPointOnPlane(point, epsilon))
	{
		return 0;
	}
	else if (this->getDistanceFromPointToPlane(point) > 0.0)
	{
		return 1;
	}
	else
	{
		return -1;
	}
}

Eigen::Vector3d Plane::getProjectionOfPointOntoPlane(const Eigen::Vector3d &point) const
{
	assert(m_is_valid);
	Eigen::Vector3d v0 = point - m_origin;
	double distance = v0.dot(m_normal);
	Eigen::Vector3d point_projection = v0 - distance * m_normal + m_origin;
	assert(abs(this->getDistanceFromPointToPlane(point_projection)) < 1e-6);
	return point_projection;
}

bool Plane::getMostPositivePoint(const std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &result) const
{
	assert(m_is_valid);
	double max = 0.0;
	double distance = 0.0;
	bool result_found = false;
	for (auto &point :points)
	{
		distance = this->getDistanceFromPointToPlane(point);
		if (distance > max)
		{
			max = distance;
			result = point;
			result_found = true;
		}
	}
	return result_found;
}

bool Plane::getMostNegativePoint(const std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &result) const
{
	assert(m_is_valid);
	double min = 0.0;
	double distance = 0.0;
	bool result_found = false;
	for (auto &point : points)
	{
		distance = this->getDistanceFromPointToPlane(point);
		if (distance < min)
		{
			min = distance;
			result = point;
			result_found = true;
		}
	}
	return result_found;
}

bool Plane::isLineOnPlane(const SO::Line &line, double epsilon) const
{
	assert(m_is_valid && line.isValid());
	return isPointOnPlane(line.getSource(), epsilon) &&
		isPointOnPlane(line.getTarget(), epsilon);
}

bool Plane::isIntersectedWithLine(const SO::Line &line) const
{
	assert(m_is_valid && line.isValid());
	int a = this->getPositionOfPointWrtPlane(line.getSource());
	int b = this->getPositionOfPointWrtPlane(line.getTarget());

	if (a * b != 1)
	{
		return true;
	};

	return false;
}

bool Plane::isIntersectedWithLine(const SO::Line &line, Eigen::Vector3d &point) const
{
	assert(m_is_valid && line.isValid());
	if (this->isIntersectedWithLine(line))
	{
		double denominator = this->m_normal.dot(line.getDirection());
		double t = (this->m_origin - line.getSource()).dot(this->m_normal) / denominator;
		point = line.getSource() + t * line.getDirection();
		return true;
	}
	return false;
}

bool Plane::isIntersectedWithRay(const SO::Line &ray) const
{
	assert(m_is_valid && ray.isValid());
	double denominator = this->m_normal.dot(ray.getDirection());
	if (abs(denominator) > 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Plane::isIntersectedWithRay(const SO::Line &ray, Eigen::Vector3d &point) const
{
	assert(m_is_valid && ray.isValid());
	double denominator = this->m_normal.dot(ray.getDirection());
	if (abs(denominator) > 0)
	{
		double denominator = this->m_normal.dot(ray.getDirection());
		double t = (this->m_origin - ray.getSource()).dot(this->m_normal) / denominator;
		point = ray.getSource() + t * ray.getDirection();
		return true;
	}
	else
	{
		return false;
	}
}

bool Plane::isIntersectedWithPlane(const Plane &other) const
{
	assert(m_is_valid && other.m_is_valid);
	Eigen::Vector3d n12 = this->m_normal.cross(other.m_normal);
	if (n12.norm() < 1e-6)
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool Plane::isIntersectedWithPlane(const Plane &other, SO::Line &intersecting_line) const
{
	assert(m_is_valid && other.m_is_valid);
	Eigen::Vector3d n12 = this->m_normal.cross(other.m_normal);
	if (n12.norm() < 1e-6)
	{
		intersecting_line.setLine(this->m_origin, other.m_origin);
		return false;
	}
	else
	{
		double numerator_1 = -this->d() * pow(other.getNormal().norm(), 2);
		double numerator_2 = -other.d() * this->m_normal.dot(other.getNormal());
		double denominator = pow(n12.norm(), 2);
		double a1 = (numerator_1 - numerator_2) / denominator;

		numerator_1 = -other.d() * pow(this->m_normal.norm(), 2);
		numerator_2 = -this->d() * this->m_normal.dot(other.getNormal());
		double a2 = (numerator_1 - numerator_2) / denominator;

		Eigen::Vector3d line_source = a1 * this->m_normal + a2 * other.getNormal();
		Eigen::Vector3d line_direction = n12 / n12.norm();
		intersecting_line = SO::Line(line_source, line_direction, 1);
		return true;
	}
}

double Plane::getAngleOfRotation(const Plane &destination) const
{
	assert(m_is_valid && destination.m_is_valid);
	double rotation_angle = this->m_normal.dot(destination.m_normal) / (this->m_normal.norm() * destination.m_normal.norm());
	if (rotation_angle > 1.0 || rotation_angle < -1.0)
	{
		return 0;
	}
	else
	{
		rotation_angle = acos(rotation_angle);
		return rotation_angle;
	}
}

Eigen::Vector3d Plane::getAxisOfRotation(const Plane &destination) const
{
	assert(m_is_valid && destination.m_is_valid);
	Eigen::Vector3d axis_of_rotation = this->m_normal.cross(destination.m_normal);
	axis_of_rotation = axis_of_rotation / axis_of_rotation.norm();
	return axis_of_rotation;
}

Eigen::Transform<double, 3, Eigen::Affine> Plane::getTransformationMatrix(const Plane &destination) const
{
	assert(m_is_valid && destination.m_is_valid);
	Eigen::Transform<double, 3, Eigen::Affine> transformation_matrix;
	// If loop for getting rid of transformation matrix with Nan
	// It happens if v1 and v2 pointing to the same direction
	if (getAngleOfRotation(destination) != 0)
	{
		transformation_matrix = Eigen::AngleAxis<double>(
			getAngleOfRotation(destination),
			getAxisOfRotation(destination));
	}
	else
	{
		transformation_matrix.setIdentity();
	}
	return transformation_matrix;
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
	assert(m_is_valid);
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
