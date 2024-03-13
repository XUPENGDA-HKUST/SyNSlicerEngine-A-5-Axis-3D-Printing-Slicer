#include "printing_layer_collection.h"

using SyNSlicerEngine::Object::PrintingLayerCollection;

PrintingLayerCollection::PrintingLayerCollection()
{

}

PrintingLayerCollection::PrintingLayerCollection(const PrintingLayerCollection &other)
{
	*this = other;
}

PrintingLayerCollection::~PrintingLayerCollection()
{

}

void PrintingLayerCollection::addPrintingLayer(PrintingLayer printing_layer)
{
	m_printing_layers.push_back(printing_layer);
	layer_status.resize(m_printing_layers.size());
}

void PrintingLayerCollection::setPrintingLayer(int index, const PrintingLayer &printing_layer)
{
	assert(index < m_printing_layers.size());
	m_printing_layers[index] = printing_layer;
}

void PrintingLayerCollection::setLayerStatus(std::vector<bool> in_layer_status)
{
	layer_status.clear();
	layer_status = in_layer_status;
	if (layer_status.size()!= m_printing_layers.size())
	{
		std::cout << "sth WRONG!" << std::endl;
	}
}

bool PrintingLayerCollection::isValid()
{
	for (int i = 0; i < layer_status.size(); i++)
	{
		if (layer_status[i] == true)
		{
			return false;
		}
	}
	return true;
}

void PrintingLayerCollection::generateToolpath(int toolpath_type)
{
	Eigen::Vector3d zigzag_direction(1, 0, 0);
	/*
	for (int i = 0; i < m_printing_layers.size(); i++)
	//for (int i = 0; i < 88; i++)
	{
		ToolpathGenerator toolpath_generator(m_printing_layers[i]);
		if (toolpath_type == PathPlanning::ToolpathType::ContourParallel)
		{
			toolpath_generator.generateContourParallelPath();
			toolpath_generator.generateContourParallelPathForSupport(2);
			toolpath_generator.generatToolPath();
		}
		else if(toolpath_type == PathPlanning::ToolpathType::Zigzag)
		{			
			toolpath_generator.generateZigzagPath(zigzag_direction, 1);
			toolpath_generator.generateContourParallelPathForSupport(2);
			toolpath_generator.generatToolPath();
			zigzag_direction = toolpath_generator.getZigzagNormal();
		}
		else if(toolpath_type == PathPlanning::ToolpathType::Zigzag)
		{
			double bound[6] = { -100,100,-100,100,0,100 };
			toolpath_generator.generateSparseInfillPath(bound, 50, 1);
			toolpath_generator.generateContourParallelPathForSupport(2);
			toolpath_generator.generatToolPath();
		}
	}
	*/
}

const int PrintingLayerCollection::size() const
{
	return m_printing_layers.size();
}

const int PrintingLayerCollection::getNumberOfLayers() const
{
	return m_printing_layers.size();
}

SO::PrintingLayer PrintingLayerCollection::getLayer(int index)
{
	return m_printing_layers[index];
}

SO::PrintingLayer &PrintingLayerCollection::operator[](int index)
{
	return m_printing_layers[index];
}

PrintingLayerCollection &PrintingLayerCollection::operator=(const PrintingLayerCollection &other)
{
	m_contours = PolylineCollection();
	m_contours_error = PolylineCollection();
	m_printing_layers = other.m_printing_layers;
	layer_status = other.layer_status;
	return *this;
}
