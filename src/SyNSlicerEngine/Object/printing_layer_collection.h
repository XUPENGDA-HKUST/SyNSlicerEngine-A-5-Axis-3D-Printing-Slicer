#ifndef SYNSLICERENGINE_OBJECT_PRINTINGLAYERCOLLECTION_H_
#define SYNSLICERENGINE_OBJECT_PRINTINGLAYERCOLLECTION_H_

#include <list>
#include <vector>

#include <CGAL_CORE_CLASS>

#include "Object/printing_layer.h"
#include "Object/polyline_collection.h"
#include "Object/polygon_collection.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object {

	//!  Container store all the printing layers of a partition.
	class PrintingLayerCollection
	{
	public:
		//! Default constructor.
		PrintingLayerCollection();

		//! Copy constructor.
		PrintingLayerCollection(const PrintingLayerCollection &other);

		//! Destructor.
		~PrintingLayerCollection();

		//! Add a printing layer to this contatiner.
		/*!
			\param[in] printing_layer A printing layer.
		*/
		void addPrintingLayer(SO::PrintingLayer printing_layer);

		//! Replace the n-th printing layer with a new one.
		/*!
			\param[in] index			n.
			\param[in] printing_layer	Another printing layer.
		*/
		void setPrintingLayer(int index, const SO::PrintingLayer &printing_layer);

		//! Set layer status.
		/*!
			\brief	True is that printing layer is valid.
			\param[in] in_layer_status	Status of all the priting layers.
		*/
		void setLayerStatus(std::vector<bool> in_layer_status);

		//! Check if all printing layers are valid.
		/*!
			\return \b True if all printing layer are valid. \n \b False if not.
		*/
		bool isValid();

		//! Update.
		/*!
			\brief	Group all the contours from difference printing layers together to improve visualization efficiency.
		*/
		void update();

		//! Get all contours that are valid.
		/*!
			\return \b SO::PolylineCollection Contours that are valid.
		*/
		SO::PolylineCollection getContours() const;

		//! Get all contours that are not valid.
		/*!
			\return \b SO::PolylineCollection Contours that are not valid.
		*/
		SO::PolylineCollection getErrorContours() const;

		//! Get all contours of the support structure.
		/*!
			\return \b SO::PolylineCollection Contours of the support structure.
		*/
		SO::PolylineCollection getSupportContours() const;

		//! Get number of printing layers in the container.
		/*!
			\return \b int Number of printing layers.
		*/
		const int size() const;

		//! Get number of printing layers in the container.
		/*!
			\return \b int Number of printing layers.
		*/
		const int getNumberOfLayers() const;

		//! Get the n-th layer in the container.
		/*!
			\param[in] index	n.
		*/
		SO::PrintingLayer getLayer(int index);

		//! Get the n-th layer in the container.
		/*!
			\param[in] index	n.
		*/
		SO::PrintingLayer &operator[](int index);

		//! Copy assignment operator.
		PrintingLayerCollection &operator=(const PrintingLayerCollection &other);

	protected:
		//! Store all the printing layers.
		std::vector<SO::PrintingLayer> m_printing_layers;
		
		//! Store the status of each layers.
		std::vector<bool> layer_status;

		//! Store all contours that are valid.
		SO::PolylineCollection m_contours;

		//! Store all contours that are not valid.
		SO::PolylineCollection m_contours_error;

		//! Store all contours of the support structure.
		SO::PolylineCollection m_support;
	};
}

#endif  // SYNSLICERENGINE_OBJECT_PRINTINGLAYERCOLLECTION_H_