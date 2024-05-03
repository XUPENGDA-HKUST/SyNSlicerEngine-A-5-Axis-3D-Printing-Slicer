#ifndef SYNSLICERENGINE_OBJECT_POLYLINE_H_
#define SYNSLICERENGINE_OBJECT_POLYLINE_H_

namespace SyNSlicerEngine::Object {

	//!  This class defines a polyline.
	class Polyline
	{
	public:
		//! Default constructor.
		Polyline();

		//! Copy constructor.
		Polyline(const Polyline &other);

		//! Constructor.
		/*!
			\brief	construct this by inputting a set of points.
			\param[in]	polyline	A set of points.
		*/
		Polyline(const std::vector<Eigen::Vector3d> &polyline);

		//! Destructor.
		~Polyline();
		
		//! Restore to default value.
		void reset();

		//! Add point to polyline.
		/*!
			\param[in] point A point.
		*/
		void push_back(const Eigen::Vector3d &point);

		//! Remove the last point from polyline.
		void pop_back();

		//! Check if this polyline is closed.
		/*!
			\return \b True if the polyline is closed \n \b False if the polyline is open.
		*/
		bool isClosed();

		//! Close the polyline.
		void closePolyline();

		//! Get the length of the polyline.
		/*!
			\return \b double Length.
		*/
		double getLength();

		//! Get number of points in the polyline.
		/*!
			\return \b int Number of points.
		*/
		int numberOfPoints() const;

		//! Get the n-th point in the polyline.
		/*!
			\param[in] index Index.
			\return \b Eigen::Vector3d The n-th point.
		*/
		Eigen::Vector3d &operator[](unsigned int index);

		//! Get the n-th point in the polyline.
		/*!
			\param[in] index Index.
			\return \b Eigen::Vector3d The n-th point.
		*/
		const Eigen::Vector3d operator[](unsigned int index) const;

		//! Get all the points in the polyline.
		/*!
			\return \b std::vector<Eigen::Vector3d> The n-th point.
		*/
		const std::vector<Eigen::Vector3d> get() const;

		//! Copy assignment operator.
		Polyline &operator=(const Polyline &other);

	protected:
		//! Store all the points.
		std::vector<Eigen::Vector3d> m_polyline;
	};
}

#endif  // SYNSLICERENGINE_OBJECT_POLYLINE_H_