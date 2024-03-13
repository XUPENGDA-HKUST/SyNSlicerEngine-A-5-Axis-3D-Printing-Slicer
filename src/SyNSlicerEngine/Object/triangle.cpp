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
	m_v0 = v0;
	m_v1 = v1;
	m_v2 = v2;
}

Triangle::Triangle(CgalMesh_EPICK::Face_index f, CgalMesh_EPICK &mesh)
{
	std::vector<Eigen::Vector3d> vertices;
	for (auto v : mesh.vertices_around_face(mesh.halfedge(f)))
	{
		auto &p = mesh.point(v);
		vertices.emplace_back(Eigen::Vector3d(p.x(), p.y(), p.z()));
	}
	m_v0 = vertices[0];
	m_v1 = vertices[1];
	m_v2 = vertices[2];
}

Triangle::~Triangle()
{

}

double Triangle::getArea()
{
	Eigen::Vector3d v0 = m_v1 - m_v0;
	Eigen::Vector3d v1 = m_v2 - m_v0;
	double area = 0.5 * v1.cross(v0).norm();
	return area;
}

Eigen::Vector3d Triangle::getNormal()
{
	Eigen::Vector3d v0 = m_v1 - m_v0;
	Eigen::Vector3d v1 = m_v2 - m_v0;
	Eigen::Vector3d normal = v1.cross(v0);
	if (normal.norm() > 1e-6)
	{
		normal = normal / normal.norm();
	}
	return normal;
}

double Triangle::getOverhangingAngle(const SO::Plane &plane)
{
	Eigen::Vector3d face_normal =this->getNormal();
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

bool Triangle::isOneOfTheVerticesOnPlane(const SO::Plane &plane)
{
	return plane.isPointOnPlane(m_v0)|| 
		plane.isPointOnPlane(m_v1)|| 
		plane.isPointOnPlane(m_v2);
}

Triangle &Triangle::operator=(const Triangle &other)
{
	m_v0 = other.m_v0;
	m_v1 = other.m_v1;
	m_v2 = other.m_v2;
	return *this;
}
