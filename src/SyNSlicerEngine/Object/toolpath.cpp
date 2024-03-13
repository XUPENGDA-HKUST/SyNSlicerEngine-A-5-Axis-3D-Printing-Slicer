#include "toolpath.h"

using SyNSlicerEngine::Object::Toolpath;

Toolpath::Toolpath()
{
}

Toolpath::Toolpath(const Toolpath &other)
{
	*this = other;
}

Toolpath::~Toolpath()
{
}

void Toolpath::addToolPoint(const ToolPoint &tool_point)
{
	ToolPoint temp(tool_point);
	if (temp[2] < 0)
	{
		std::cout << m_tool_path.size() << std::endl;
	}
	m_tool_path.push_back(tool_point);
}

void Toolpath::setToolPoint(int index, const ToolPoint &tool_point)
{
	assert(index >= 0 && index < m_tool_path.size());
	m_tool_path[index] = tool_point;
}

void Toolpath::setExtrusion(int index, double extrusion)
{
	m_tool_path[index].setExtrusion(extrusion);
}

void Toolpath::reset()
{
	m_tool_path.clear();
}

const int Toolpath::size() const
{
	return m_tool_path.size();
}

const SO::ToolPoint &Toolpath::back() const
{
	return m_tool_path.back();
}

SO::ToolPoint &Toolpath::operator[](int index)
{
	assert(index >= 0 && index < m_tool_path.size());
	return m_tool_path[index];
}

const SO::ToolPoint &Toolpath::operator[](int index) const
{
	assert(index >= 0 && index < m_tool_path.size());
	return m_tool_path[index];
}

Toolpath &Toolpath::operator=(const Toolpath &other)
{
	m_tool_path = other.m_tool_path;
	return *this;
}
