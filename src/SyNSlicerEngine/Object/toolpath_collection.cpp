#include "toolpath_collection.h"

using SyNSlicerEngine::Object::ToolpathCollection;

ToolpathCollection::ToolpathCollection()
{

}

ToolpathCollection::ToolpathCollection(const ToolpathCollection &other)
{
	*this = other;
}

ToolpathCollection::~ToolpathCollection()
{

}

void ToolpathCollection::setNumberOfToolPath(int number)
{
	m_tool_paths.reserve(number);
}

void ToolpathCollection::addToolPath(const Toolpath &tool_path)
{
	m_tool_paths.push_back(tool_path);
}

const unsigned int ToolpathCollection::size() const
{
	return m_tool_paths.size();
}

const SO::Toolpath &ToolpathCollection::operator[](unsigned int index) const
{
	assert(index >= 0 && index < m_tool_paths.size());
	return m_tool_paths[index];
}

ToolpathCollection &ToolpathCollection::operator=(const ToolpathCollection &other)
{
	m_tool_paths = other.m_tool_paths;
	return *this;
}

void ToolpathCollection::reset()
{
	m_tool_paths.clear();
}
