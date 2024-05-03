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
		if (layer_status[i] == false)
		{
			return false;
		}
	}
	return true;
}

void PrintingLayerCollection::update()
{
	for (auto &layer: m_printing_layers)
	{
		for (auto &contour : layer.getContours().get())
		{
			m_contours.addPolyline(contour.get());
		}		
	}

	m_contours.closePolylines();

	for (auto &layer : m_printing_layers)
	{
		for (auto &contour : layer.getSupportStructureContours().get())
		{
			m_support.addPolyline(contour.get());
		}
	}

	m_support.closePolylines();
}

SO::PolylineCollection PrintingLayerCollection::getContours() const
{
	return m_contours;
}

SO::PolylineCollection PrintingLayerCollection::getErrorContours() const
{
	return m_contours_error;
}

SO::PolylineCollection PrintingLayerCollection::getSupportContours() const
{
	return m_support;
}

const int PrintingLayerCollection::size() const
{
	return m_printing_layers.size();
}

const int PrintingLayerCollection::getNumberOfLayers() const
{
	return m_printing_layers.size();
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
