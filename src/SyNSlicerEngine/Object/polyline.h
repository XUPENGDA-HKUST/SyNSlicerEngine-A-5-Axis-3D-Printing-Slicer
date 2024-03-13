#ifndef SYNSLICERENGINE_OBJECT_POLYLINE_H_
#define SYNSLICERENGINE_OBJECT_POLYLINE_H_

namespace SyNSlicerEngine::Object {

	//!  This class defines a polyline.
	class Polyline
	{
	public:

		Polyline();
		Polyline(const Polyline &other);
		Polyline(const std::vector<Eigen::Vector3d> &polyline);
		~Polyline();
		
		void reset();
		void addPoint(const Eigen::Vector3d &point);

		bool isClosed();
		void closePolyline();
		double getLength();

		const int size() const;
		Eigen::Vector3d &operator[](unsigned int index);
		const Eigen::Vector3d operator[](unsigned int index) const;
		const std::vector<Eigen::Vector3d> get() const;
		Polyline &operator=(const Polyline &other);

	private:
		std::vector<Eigen::Vector3d> m_polyline;
	};
}

#endif  // SYNSLICERENGINE_OBJECT_POLYLINE_H_