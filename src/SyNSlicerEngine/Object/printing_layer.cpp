#include "printing_layer.h"

using SyNSlicerEngine::Object::PrintingLayer;

PrintingLayer::PrintingLayer()
{
}

PrintingLayer::PrintingLayer(const PrintingLayer &other)
{
	*this = other;
}

PrintingLayer::PrintingLayer(const SO::Plane &slicing_plane, const SO::Plane &prev_slicing_plane)
{
	m_slicing_plane = slicing_plane;
	m_prev_slicing_plane = prev_slicing_plane;
}

PrintingLayer::PrintingLayer(const SO::PolygonCollection &contours, const SO::Plane &slicing_plane, const SO::Plane &prev_slicing_plane)
{
	m_contours = contours;
	m_slicing_plane = slicing_plane;
	m_prev_slicing_plane = prev_slicing_plane;
}

PrintingLayer::~PrintingLayer()
{

}

int PrintingLayer::getNumberOfContours()
{
	return m_contours.numberOfPolygons();
}

SO::Polygon &PrintingLayer::getContour(int index)
{
	return m_contours[index];
}

SO::PolygonCollection &PrintingLayer::getContours()
{
	return m_contours;
}

void PrintingLayer::setSupportStructureContours(const PolygonCollection &input_support_structure_contours)
{
	m_support_structure_contours = input_support_structure_contours;
}

void PrintingLayer::addSupportStructureContours(PolygonCollection &input_support_structure_contours)
{
	for (int i = 0; i < input_support_structure_contours.numberOfPolygons(); i++)
	{
		m_support_structure_contours.addPolygon(input_support_structure_contours[i]);
	}
}

SO::PolygonCollection &PrintingLayer::getSupportStructureContours()
{
	return m_support_structure_contours;
}

Eigen::Vector3d &PrintingLayer::getOrigin()
{
	return m_origin_for_infill_alignment;
}

SO::Line &PrintingLayer::getDirection1()
{
	return m_direction_1_for_infill_alignment;
}

SO::Line &PrintingLayer::getDirection2()
{
	return m_direction_2_for_infill_alignment;
}

SO::PrintingPathCollection &PrintingLayer::getPrintingPaths()
{
	return m_printing_paths;
}

SO::PrintingPathCollection &PrintingLayer::getPrintingPathsForSupport()
{
	return m_printing_paths_support;
}

void PrintingLayer::setToolpaths(const ToolpathCollection &toolpaths)
{
	m_toolpaths = toolpaths;
}

const SO::ToolpathCollection &PrintingLayer::getToolpaths() const
{
	return m_toolpaths;
}

void PrintingLayer::clearToolpath()
{
	m_toolpaths.reset();
}

SO::Plane PrintingLayer::getSlicingPlane()
{
	return m_slicing_plane;
}

SO::Plane PrintingLayer::getPrevSlicingPlane()
{
	return m_prev_slicing_plane;
}

PrintingLayer &PrintingLayer::operator=(const PrintingLayer &other)
{
	m_contours = other.m_contours;
	m_support_structure_contours = other.m_support_structure_contours;
	m_slicing_plane = other.m_slicing_plane;
	m_prev_slicing_plane = other.m_prev_slicing_plane;
	m_printing_paths = other.m_printing_paths;
	m_printing_paths_support = other.m_printing_paths_support;
	m_toolpaths = other.m_toolpaths;
	return *this;
}
