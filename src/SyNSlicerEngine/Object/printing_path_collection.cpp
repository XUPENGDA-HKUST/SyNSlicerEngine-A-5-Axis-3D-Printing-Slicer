#include "printing_path_collection.h"

using SyNSlicerEngine::Object::PrintingPathCollection;

PrintingPathCollection::PrintingPathCollection()
{
}

PrintingPathCollection::~PrintingPathCollection()
{
}

void PrintingPathCollection::reset()
{
	m_surface.reset();
	m_wall.clear();
	m_bottom.clear();
	m_top.clear();
	m_bottom_top_union.reset();
	m_infill.reset();
}

SO::PolygonCollection &PrintingPathCollection::getSurface()
{
	return m_surface;
}

std::vector<SO::PolygonCollection> &PrintingPathCollection::getWall()
{
	return m_wall;
}

std::vector<SO::PolygonCollection> &PrintingPathCollection::getBottom()
{
	return m_bottom;
}

std::vector<SO::PolygonCollection> &PrintingPathCollection::getTop()
{
	return m_top;
}

SO::PolygonCollection &PrintingPathCollection::getBottomTopUnion()
{
	return m_bottom_top_union;
}

SO::PolygonCollection &PrintingPathCollection::getInfill()
{
	return m_infill;
}
