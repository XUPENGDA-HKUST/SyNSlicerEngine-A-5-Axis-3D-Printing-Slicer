#include "polyline_collection.h"

using SyNSlicerEngine::Object::Polyline;
using SyNSlicerEngine::Object::PolylineCollection;

PolylineCollection::PolylineCollection()
{

}

PolylineCollection::PolylineCollection(const PolylineCollection &other)
{
	*this = other;
}

PolylineCollection::~PolylineCollection()
{

}

void PolylineCollection::addPolyline(const Polyline &polyline)
{
	m_polylines.emplace_back(polyline);
}

void PolylineCollection::addPolylines(const PolylineCollection &other)
{
	for (int i = 0; i < other.size(); i++)
	{
		m_polylines.push_back(other[i]);
	}
}

const int PolylineCollection::size() const
{
	return m_polylines.size();
}

int PolylineCollection::numberOfPolylines() const
{
	return m_polylines.size();
}

void PolylineCollection::closePolylines()
{
	for (auto &polyline : m_polylines)
	{
		polyline.closePolyline();
	}
}

void PolylineCollection::setPolyline(int index, const Polyline &polyline)
{
	assert(index >= 0 && index < m_polylines.size());
	m_polylines[index] = polyline;
}

const std::vector<SyNSlicerEngine::Object::Polyline> &PolylineCollection::get() const
{
	return m_polylines;
}

Polyline &PolylineCollection::operator[](unsigned int index)
{
	assert(index >= 0 && index < m_polylines.size());
	return m_polylines[index];
}

const Polyline &PolylineCollection::operator[](unsigned int index) const
{
	assert(index >= 0 && index < m_polylines.size());
	return m_polylines[index];
}

PolylineCollection &PolylineCollection::operator=(const PolylineCollection &other)
{
	m_polylines = other.m_polylines;
	return *this;
}
