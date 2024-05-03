#ifndef SYNSLICERENGINE_OBJECT_POLYLINECOLLECTION_H_
#define SYNSLICERENGINE_OBJECT_POLYLINECOLLECTION_H_

#include "Object/polyline.h"

//! A namespace used to store the Class related to data storage
namespace SyNSlicerEngine::Object {

	//! This class is used to store multiple polylines.
	class PolylineCollection
	{
	public:
		//! Default constructor.
		PolylineCollection();

		//! Copy constructor.
		PolylineCollection(const PolylineCollection &other);

		//! Destructor.
		~PolylineCollection();
		
		//! Add a polyline to this container.
		/*!
			\param polyline A polyline.
		*/
		void addPolyline(const Polyline &polyline);

		//! Add multiple polylines to this container.
		/*!
			\param other Another PolylineCollection.
		*/
		void addPolylines(const PolylineCollection &other);

		//! Get numbers of polylines in this container.
		/*!
			\return \b const \b in Numbers of polylines.
		*/
		const int size() const;

		//! Get numbers of polylines in this container.
		/*!
			\return \b const \b in Numbers of polylines.
		*/
		int numberOfPolylines() const;

		//! Close all polylines if they are open.
		void closePolylines();

		//! Replace the n-th polyline to another polyline.
		/*!
			\param index	n.
			\param polyline	Another polyline. 
		*/
		void setPolyline(int index, const Polyline &polyline);

		//! Get all the polylines.
		/*!
			\return \b const \b std::vector<Polyline> All polylines.
		*/
		const std::vector<Polyline> &get() const;

		//! Get the n-th polyline.
		/*!
			\param index	n.
		*/
		Polyline &operator[](unsigned int index);

		//! Get the n-th polyline.
		/*!
			\param index	n.
		*/
		const Polyline &operator[](unsigned int index) const;

		//! Copy assignment operator.
		PolylineCollection &operator=(const PolylineCollection &other);

	protected:
		//! Store all the polylines.
		std::vector<Polyline> m_polylines;
	};
}

#endif  // SYNSLICERENGINE_OBJECT_POLYLINECOLLECTION_H_