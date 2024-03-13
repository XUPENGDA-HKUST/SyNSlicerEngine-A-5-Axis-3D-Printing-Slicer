#ifndef SYNSLICERENGINE_OBJECT_PRINTINGLAYERCOLLECTION_H_
#define SYNSLICERENGINE_OBJECT_PRINTINGLAYERCOLLECTION_H_

#include <list>
#include <vector>

#include <CGAL_CORE_CLASS>

#include "Object/printing_layer.h"
#include "Object/polyline_collection.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object {

	//!  This class is used to store all the printing layers in a Mesh
	/*!
	*/
	class PrintingLayerCollection
	{
	public:
		PrintingLayerCollection();
		PrintingLayerCollection(const PrintingLayerCollection &other);
		~PrintingLayerCollection();

		void addPrintingLayer(SO::PrintingLayer printing_layer);
		void setPrintingLayer(int index, const SO::PrintingLayer &printing_layer);

		void setLayerStatus(std::vector<bool> in_layer_status);
		bool isValid();

		void generateToolpath(int toolpath_type = 0);

		const int size() const;
		const int getNumberOfLayers() const;
		SO::PrintingLayer getLayer(int index);
		SO::PrintingLayer &operator[](int index);

		PrintingLayerCollection &operator=(const PrintingLayerCollection &other);

	private:
		std::vector<SO::PrintingLayer> m_printing_layers;
		std::vector<bool> layer_status;

		SO::PolylineCollection m_contours;
		SO::PolylineCollection m_contours_error;
		SO::PolylineCollection m_support;
	};
}

#endif  // SYNSLICERENGINE_OBJECT_PRINTINGLAYERCOLLECTION_H_