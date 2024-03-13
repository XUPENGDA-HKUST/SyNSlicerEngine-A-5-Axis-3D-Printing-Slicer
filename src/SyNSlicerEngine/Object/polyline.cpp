#include "polyline.h"

using SyNSlicerEngine::Object::Polyline;

Polyline::Polyline()
{

}

Polyline::Polyline(const Polyline &other)
{
	*this = other;
}

Polyline::Polyline(const std::vector<Eigen::Vector3d> &polyline)
{
	m_polyline = polyline;
}

Polyline::~Polyline()
{

}

void Polyline::reset()
{
	m_polyline.clear();
}

void Polyline::addPoint(const Eigen::Vector3d &point)
{
	m_polyline.push_back(point);
}

bool Polyline::isClosed()
{
	if ((m_polyline.back() - m_polyline.front()).norm() < 1e-6)
	{
		return true;
	}
	return false;
}

void Polyline::closePolyline()
{
	if (this->isClosed() == false)
	{
		m_polyline.emplace_back(m_polyline.front());
	}
}

double Polyline::getLength()
{
	double distance = 0.0;
	int j = 0;
	for (int i = 1; i < m_polyline.size(); ++i)
	{
		distance = (m_polyline[i] - m_polyline[j]).norm();
	};
	return distance;
}

const int Polyline::size() const
{
	return m_polyline.size();
}

Eigen::Vector3d &Polyline::operator[](unsigned int index)
{
	return m_polyline[index];
}

const Eigen::Vector3d Polyline::operator[](unsigned int index) const
{
	return m_polyline[index];
}

const std::vector<Eigen::Vector3d> Polyline::get() const
{
	return m_polyline;
}

Polyline &Polyline::operator=(const Polyline &other)
{
	m_polyline = other.m_polyline;
	return *this;
}
