#ifndef SYNSLICERENGINE_OBJECT_POLYLINECOLLECTION_H_
#define SYNSLICERENGINE_OBJECT_POLYLINECOLLECTION_H_

#include "Object/polyline.h"

//! A namespace used to store the Class related to data storage
namespace SyNSlicerEngine::Object {

	//! This class is used to store multiple polylines.
	class PolylineCollection
	{
	public:
		PolylineCollection();
		PolylineCollection(const PolylineCollection &other);
		~PolylineCollection();
		
		void setNumberOfPolyline(int number);
		void addPolyline(const Polyline &polyline);
		void addPolylines(const PolylineCollection &other);

		const int size() const;
		unsigned int getNumberOfPolyline();

		void closePolylines();

		void setPolyline(int index, const Polyline &polyline);

		const std::vector<Polyline> &get() const;
		Polyline &operator[](unsigned int index);
		const Polyline &operator[](unsigned int index) const;

		PolylineCollection &operator=(const PolylineCollection &other);

	protected:
		std::vector<Polyline> m_polylines;

	};
}

#endif  // SYNSLICERENGINE_OBJECT_POLYLINECOLLECTION_H_