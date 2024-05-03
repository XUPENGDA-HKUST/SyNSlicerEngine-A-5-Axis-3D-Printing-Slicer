#include "triangle.h"

using SyNSlicerEngine::Object::Triangle;

Triangle::Triangle()
{
}

Triangle::Triangle(const Triangle &other)
{
}

Triangle::Triangle(Eigen::Vector3d v0, Eigen::Vector3d v1, Eigen::Vector3d v2)
{
	m_vertices[0] = v0;
	m_vertices[1] = v1;
	m_vertices[2] = v2;
}

Triangle::Triangle(const CgalMesh_EPICK::Face_index &f, const CgalMesh_EPICK &mesh)
{
	int i = 0;
	for (auto v : mesh.vertices_around_face(mesh.halfedge(f)))
	{
		auto &p = mesh.point(v);
		m_vertices[i] = Eigen::Vector3d(p.x(), p.y(), p.z());
		i += 1;
	}
}

Triangle::~Triangle()
{

}

Eigen::Vector3d &Triangle::v0()
{
	return m_vertices[0];
}

const Eigen::Vector3d &Triangle::v0() const
{
	return m_vertices[0];
}

Eigen::Vector3d &Triangle::v1()
{
	return m_vertices[1];
}

const Eigen::Vector3d &Triangle::v1() const
{
	return m_vertices[1];
}

Eigen::Vector3d &Triangle::v2()
{
	return m_vertices[2];
}

const Eigen::Vector3d &Triangle::v2() const
{
	return m_vertices[2];
}

double Triangle::getArea()
{
	Eigen::Vector3d v0 = m_vertices[1] - m_vertices[0];
	Eigen::Vector3d v1 = m_vertices[2] - m_vertices[0];
	double area = 0.5 * v0.cross(v1).norm();
	return area;
}

Eigen::Vector3d Triangle::getNormal()
{
	Eigen::Vector3d v0 = m_vertices[1] - m_vertices[0];
	Eigen::Vector3d v1 = m_vertices[2] - m_vertices[0];
	Eigen::Vector3d normal = v0.cross(v1);
	if (normal.norm() > 1e-6)
	{
		normal = normal / normal.norm();
	}
	return normal;
}

double Triangle::getOverhangingAngle(const SO::Plane &plane)
{
	Eigen::Vector3d face_normal = this->getNormal();
	Eigen::Vector3d plane_normal = -plane.getNormal();
	double overhanging_angle = face_normal.dot(plane_normal);

	if (overhanging_angle <= -1.0)
	{
		overhanging_angle = M_PI;
	}
	else if (overhanging_angle >= 1.0)
	{
		overhanging_angle = 0;
	}
	else
	{
		overhanging_angle = acos(overhanging_angle);
	}

	return overhanging_angle;
}

bool Triangle::isOneOfTheVerticesOnPlane(const SO::Plane &plane, double epsilon) const
{
	bool result = false;
	for (auto &vertex : m_vertices)
	{
		result = result || plane.isPointOnPlane(vertex, epsilon);
	}
	return result;
}

Triangle &Triangle::operator=(const Triangle &other)
{
	for (int i = 0; i < 3; i++)
	{
		m_vertices[i] = other.m_vertices[i];
	}
	return *this;
}
